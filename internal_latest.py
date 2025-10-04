#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
sensors_service.py
------------------

This script connects to a PX4 flight stack via MAVSDK, collects a set of
telemetry values (battery, GPS, position, velocity, attitude, flight status,
health, RC status, heading and wind), publishes the latest snapshot as a
JSON file, prints a human‑readable summary to the terminal every second,
and exposes a FastAPI endpoint at `/sensors` returning the raw snapshot.

It is completely self‑contained and does not depend on any external modules
from this repository.  Simply run it with the appropriate MAVSDK URL and
JSON output path.  For example:

    python sensors_service.py --url udp://:14540 --hz 1.0 --json mavsdk_sensor_snapshot.json

The service will print sensor readings to the console, update the specified
JSON file, and serve the latest snapshot via HTTP at http://localhost:8001/sensors.
"""

import argparse
import asyncio
import json
import math
import signal
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

try:
    # MAVSDK is required for communicating with PX4
    from mavsdk import System
    from mavsdk.telemetry import FlightMode, FixType
except ImportError:
    System = None  # type: ignore
    FlightMode = None  # type: ignore
    FixType = None  # type: ignore

import uvicorn
from fastapi import FastAPI


def now_iso() -> str:
    """Return the current time in ISO8601 format with UTC offset."""
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def fmt(value: Optional[Any], nd: int = 2, unit: str = "", percent: bool = False) -> str:
    """Format a numeric value for display.  None becomes 'N/A'."""
    if value is None:
        return "N/A"
    try:
        v = float(value)
    except Exception:
        return str(value)
    if percent:
        return f"{v:.{nd}f}%"
    return f"{v:.{nd}f}{unit}"


def quat_to_euler_deg(w: float, x: float, y: float, z: float) -> Dict[str, float]:
    """Convert a quaternion into Euler angles (degrees)."""
    # roll (x)
    t0 = 2.0 * (w * x + y * z)
    t1 = w * w - x * x - y * y - z * z
    roll = math.degrees(math.atan2(t0, t1))
    # pitch (y)
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else (-1.0 if t2 < -1.0 else t2)
    pitch = math.degrees(math.asin(t2))
    # yaw (z)
    t3 = 2.0 * (w * z + x * y)
    t4 = w * w - x * x - y * y - z * z
    yaw = math.degrees(math.atan2(t3, t4))
    return {"roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw}


class TelemetryMonitor:
    """
    Collect telemetry data from a PX4 vehicle over MAVSDK and maintain a
    snapshot dictionary that is updated in real time.  The monitor runs
    multiple asynchronous tasks concurrently to subscribe to different
    telemetry streams.  The snapshot can be queried from other threads or
    processes (e.g. via FastAPI).
    """

    def __init__(self, url: str, hz: float, json_path: Path) -> None:
        self.url = url
        self.hz = max(0.2, float(hz))
        self.json_path = json_path
        if System is None:
            raise RuntimeError("mavsdk is not installed; cannot monitor PX4 sensors")
        self.drone = System()
        self._stop = asyncio.Event()
        # Snapshot dict to store the latest telemetry values
        self.snap: Dict[str, Any] = {
            "timestamp": now_iso(),
            "battery": {},
            "gps": {},
            "position": {},
            "velocity_ned": {},
            "attitude": {},
            "health": {},
            "status": {},
            "rc": {},
            "heading_deg": None,
            "wind": {},
        }
        self._reading_count = 0

    async def connect(self) -> None:
        """Connect to the PX4 vehicle and wait for a heartbeat."""
        await self.drone.connect(system_address=self.url)
        # Wait for connection state to become connected
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break

    async def _set_rates(self) -> None:
        """Set conservative telemetry rates; skip failures."""
        t = self.drone.telemetry
        async def try_rate(setter: str, value: float) -> None:
            fn = getattr(t, setter, None)
            if fn is None:
                return
            try:
                await fn(value)
            except Exception:
                pass
        # Set a few key rates (Hz)
        await try_rate("set_rate_battery", 5.0)
        await try_rate("set_rate_gps_info", 2.0)
        await try_rate("set_rate_position", 5.0)
        await try_rate("set_rate_velocity_ned", 5.0)
        await try_rate("set_rate_attitude_quaternion", 10.0)
        await try_rate("set_rate_health", 1.0)
        await try_rate("set_rate_rc_status", 1.0)
        await try_rate("set_rate_heading", 2.0)
        await try_rate("set_rate_wind", 1.0)

    async def _battery(self) -> None:
        async for b in self.drone.telemetry.battery():
            self.snap["battery"] = {
                "voltage_v": round(getattr(b, "voltage_v", None) or 0.0, 2),
                "current_a": (
                    round(getattr(b, "current_a", 0.0), 2)
                    if getattr(b, "current_a", None) is not None
                    else None
                ),
                "remaining": (
                    getattr(b, "remaining_percent", None)
                ),
            }

    async def _gps(self) -> None:
        async for gi in self.drone.telemetry.gps_info():
            self.snap["gps"] = {
                "num_satellites": getattr(gi, "num_satellites", None),
                "fix_type": (
                    getattr(gi, "fix_type", None).name if getattr(gi, "fix_type", None) is not None else None
                ),
            }

    async def _position(self) -> None:
        async for pos in self.drone.telemetry.position():
            self.snap["position"] = {
                "lat_deg": getattr(pos, "latitude_deg", None),
                "lon_deg": getattr(pos, "longitude_deg", None),
                "abs_alt_m": getattr(pos, "absolute_altitude_m", None),
                "rel_alt_m": getattr(pos, "relative_altitude_m", None),
            }

    async def _velocity(self) -> None:
        async for v in self.drone.telemetry.velocity_ned():
            self.snap["velocity_ned"] = {
                "north_m_s": getattr(v, "north_m_s", None),
                "east_m_s": getattr(v, "east_m_s", None),
                "down_m_s": getattr(v, "down_m_s", None),
            }

    async def _attitude(self) -> None:
        async for q in self.drone.telemetry.attitude_quaternion():
            w, x, y, z = getattr(q, "w", 1.0), getattr(q, "x", 0.0), getattr(q, "y", 0.0), getattr(q, "z", 0.0)
            euler = quat_to_euler_deg(w, x, y, z)
            self.snap["attitude"] = {
                "quaternion": {"w": w, "x": x, "y": y, "z": z},
                "euler_deg": euler,
            }

    async def _health(self) -> None:
        async for h in self.drone.telemetry.health():
            self.snap["health"] = {
                "local_position_ok": getattr(h, "is_local_position_ok", None),
                "global_position_ok": getattr(h, "is_global_position_ok", None),
                "home_position_ok": getattr(h, "is_home_position_ok", None),
            }

    async def _status(self) -> None:
        async def armed_loop() -> None:
            async for a in self.drone.telemetry.armed():
                self.snap.setdefault("status", {})["armed"] = bool(a)

        async def in_air_loop() -> None:
            async for ia in self.drone.telemetry.in_air():
                self.snap.setdefault("status", {})["in_air"] = bool(ia)

        async def mode_loop() -> None:
            async for fm in self.drone.telemetry.flight_mode():
                self.snap.setdefault("status", {})["flight_mode"] = (
                    fm.name if fm is not None else None
                )

        await asyncio.gather(armed_loop(), in_air_loop(), mode_loop())

    async def _rc(self) -> None:
        try:
            async for rc in self.drone.telemetry.rc_status():
                self.snap["rc"] = {
                    "available": getattr(rc, "is_available", None),
                    "signal_strength_percent": getattr(rc, "signal_strength_percent", None),
                }
        except Exception:
            pass

    async def _heading(self) -> None:
        try:
            async for hd in self.drone.telemetry.heading():
                self.snap["heading_deg"] = getattr(hd, "heading_deg", None)
        except Exception:
            pass

    async def _wind(self) -> None:
        try:
            async for w in self.drone.telemetry.wind():
                self.snap["wind"] = {
                    "speed_m_s": getattr(w, "speed_m_s", None),
                    "direction_deg": getattr(w, "direction_deg", None),
                }
        except Exception:
            pass

    async def _printer(self) -> None:
        """Write the snapshot to JSON and print a formatted summary every second."""
        interval = max(1.0 / self.hz, 0.1)
        while not self._stop.is_set():
            # Update timestamp and reading counter
            self.snap["timestamp"] = now_iso()
            self._reading_count += 1
            # Write JSON snapshot (convert non‑serializable to None)
            try:
                safe = json.loads(json.dumps(self.snap, default=lambda o: None))
                with open(self.json_path, "w", encoding="utf-8") as f:
                    json.dump(safe, f, indent=2)
            except Exception as exc:
                print(f"[WARN] JSON write failed: {exc}")
            # Print to terminal
            print("\x1b[2J\x1b[H", end="")
            print(self._render_summary(), flush=True)
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=interval)
            except asyncio.TimeoutError:
                pass

    def _render_summary(self) -> str:
        """Return a human‑readable summary of the current snapshot."""
        snap = self.snap
        # Format time string (HH:MM.SS)
        try:
            ts_str = snap.get("timestamp")
            ts_dt = datetime.fromisoformat(ts_str.replace("Z", "+00:00")).astimezone(timezone.utc)
            time_str = ts_dt.strftime("%H:%M.%S")
        except Exception:
            time_str = datetime.now(timezone.utc).strftime("%H:%M.%S")
        count_str = f"{self._reading_count:03d}" if self._reading_count < 1000 else str(self._reading_count)
        # Battery
        b = snap.get("battery", {})
        batt_rem = b.get("remaining")
        battery_charge = fmt(
            batt_rem * 100.0 if isinstance(batt_rem, (int, float)) else None,
            nd=1,
            unit="%",
        )
        battery_voltage = fmt(b.get("voltage_v"), nd=2, unit=" V")
        battery_current = fmt(b.get("current_a"), nd=2, unit=" A")
        # GPS
        g = snap.get("gps", {})
        gps_fix = g.get("fix_type", "N/A") or "N/A"
        gps_sats = g.get("num_satellites")
        gps_sats_str = str(gps_sats) if gps_sats is not None else "N/A"
        gps_signal = "Unknown"
        # Position
        p = snap.get("position", {})
        pos_lat = fmt(p.get("lat_deg"), nd=6)
        pos_lon = fmt(p.get("lon_deg"), nd=6)
        pos_alt = fmt(p.get("rel_alt_m"), nd=2, unit=" m")
        # Velocity
        v = snap.get("velocity_ned", {})
        vel_n = fmt(v.get("north_m_s"), nd=2, unit=" m/s")
        vel_e = fmt(v.get("east_m_s"), nd=2, unit=" m/s")
        vel_d = fmt(v.get("down_m_s"), nd=2, unit=" m/s")
        # Attitude
        a = snap.get("attitude", {}).get("euler_deg", {})
        att_roll = fmt(a.get("roll_deg"), nd=1, unit="°")
        att_pitch = fmt(a.get("pitch_deg"), nd=1, unit="°")
        att_yaw = fmt(a.get("yaw_deg"), nd=1, unit="°")
        # Flight status
        s = snap.get("status", {})
        stat_mode = s.get("flight_mode", "N/A")
        stat_armed = str(s.get("armed", "N/A"))
        stat_in_air = str(s.get("in_air", "N/A"))
        # Health
        h = snap.get("health", {})
        def hstr(val: Optional[Any]) -> str:
            if val is None:
                return "N/A"
            return "OK" if bool(val) else "Not OK"
        health_local = hstr(h.get("local_position_ok"))
        health_global = hstr(h.get("global_position_ok"))
        health_home = hstr(h.get("home_position_ok"))
        # RC
        rc = snap.get("rc", {})
        rc_avail = str(rc.get("available", "N/A"))
        rc_strength = fmt(rc.get("signal_strength_percent"), nd=0, unit="%")
        # Navigation
        nav_heading = fmt(snap.get("heading_deg"), nd=1, unit="°")
        w = snap.get("wind", {})
        nav_wind_speed = fmt(w.get("speed_m_s"), nd=1, unit=" m/s")
        nav_wind_dir = fmt(w.get("direction_deg"), nd=0, unit="°")
        # Compose lines
        lines = []
        lines.append(f"Sensor Reading {count_str}:")
        lines.append(f"Time: {time_str}\n")
        lines.append("Battery Status:")
        lines.append(f"Charge Level: {battery_charge}")
        lines.append(f"Voltage: {battery_voltage}")
        lines.append(f"Current: {battery_current}\n")
        lines.append("GPS Information:")
        lines.append(f"Fix Status: {gps_fix}")
        lines.append(f"Satellites: {gps_sats_str}")
        lines.append(f"Signal Quality: {gps_signal}\n")
        lines.append("Position Data:")
        lines.append(f"Latitude: {pos_lat}")
        lines.append(f"Longitude: {pos_lon}")
        lines.append(f"Altitude (Relative): {pos_alt}\n")
        lines.append("Velocity Vectors:")
        lines.append(f"North: {vel_n}")
        lines.append(f"East: {vel_e}")
        lines.append(f"Down: {vel_d}\n")
        lines.append("Attitude Information:")
        lines.append(f"Roll: {att_roll}")
        lines.append(f"Pitch: {att_pitch}")
        lines.append(f"Yaw: {att_yaw}\n")
        lines.append("Flight Status:")
        lines.append(f"Mode: {stat_mode}")
        lines.append(f"Armed: {stat_armed}")
        lines.append(f"In Air: {stat_in_air}\n")
        lines.append("System Health:")
        lines.append(f"Local Position: {health_local}")
        lines.append(f"Global Position: {health_global}")
        lines.append(f"Home Position: {health_home}\n")
        lines.append("Remote Control:")
        lines.append(f"Available: {rc_avail}")
        lines.append(f"Signal Strength: {rc_strength}\n")
        lines.append("Navigation:")
        lines.append(f"Heading: {nav_heading}")
        lines.append(f"Wind Speed: {nav_wind_speed}")
        lines.append(f"Wind Direction: {nav_wind_dir}")
        return "\n".join(lines)

    async def run(self) -> None:
        """Start monitoring tasks and wait until the monitor is stopped."""
        # Connect and set rates
        await self.connect()
        await self._set_rates()
        # Create tasks for each telemetry stream
        tasks = [
            asyncio.create_task(self._battery()),
            asyncio.create_task(self._gps()),
            asyncio.create_task(self._position()),
            asyncio.create_task(self._velocity()),
            asyncio.create_task(self._attitude()),
            asyncio.create_task(self._health()),
            asyncio.create_task(self._status()),
            asyncio.create_task(self._rc()),
            asyncio.create_task(self._heading()),
            asyncio.create_task(self._wind()),
            asyncio.create_task(self._printer()),
        ]
        # Setup signal handlers
        loop = asyncio.get_event_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, self._stop.set)
            except NotImplementedError:
                pass
        # Wait until stop event
        await self._stop.wait()
        # Cancel tasks on stop
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


def create_app(monitor: TelemetryMonitor) -> FastAPI:
    """Create a FastAPI application exposing the /sensors endpoint."""
    app = FastAPI()
    @app.get("/sensors")
    async def get_sensors() -> Dict[str, Any]:  # pragma: no cover
        try:
            # Convert to JSON serializable object (NaN -> None)
            return json.loads(json.dumps(monitor.snap, default=lambda o: None))
        except Exception as exc:
            return {"error": str(exc)}
    return app


async def run_service(url: str, hz: float, json_path: Path, host: str, port: int) -> None:
    """Run the telemetry monitor and web server concurrently."""
    monitor = TelemetryMonitor(url=url, hz=hz, json_path=json_path)
    app = create_app(monitor)
    # Start monitor and server concurrently
    monitor_task = asyncio.create_task(monitor.run())
    server_config = uvicorn.Config(app=app, host=host, port=port, log_level="info", lifespan="on")
    server = uvicorn.Server(server_config)
    server_task = asyncio.create_task(server.serve())
    done, pending = await asyncio.wait(
        [monitor_task, server_task], return_when=asyncio.FIRST_COMPLETED
    )
    for task in pending:
        task.cancel()
    await asyncio.gather(*pending, return_exceptions=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="PX4 telemetry service with formatted output and API")
    parser.add_argument("--url", default="udp://:14540", help="MAVSDK URL, e.g. udp://:14540 or serial:///dev/ttyACM0:115200")
    parser.add_argument("--hz", type=float, default=1.0, help="Refresh rate in Hz for updating and printing")
    parser.add_argument("--json", type=Path, default=Path("mavsdk_sensor_snapshot.json"), help="Path to write latest snapshot JSON")
    parser.add_argument("--host", default="0.0.0.0", help="Host for FastAPI server")
    parser.add_argument("--port", type=int, default=8001, help="Port for FastAPI server")
    args = parser.parse_args()
    if System is None:
        print("mavsdk is not installed; please install mavsdk==1.1.0 or later")
        return
    try:
        asyncio.run(run_service(url=args.url, hz=args.hz, json_path=args.json, host=args.host, port=args.port))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
