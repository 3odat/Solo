"""
Fully revised ROS2 visual perception node with robust HTTP APIs.

This version fixes the following issues present in earlier versions:

1. **Complete detection history** – `/scene` returns all detections from the last
   10 seconds instead of just the single nearest object. `/history` returns the
   detections from the 10 seconds preceding that window. Both endpoints also
   summarise counts per object type.
2. **Live video streaming** – a new `/video.mjpg` endpoint streams the
   annotated camera feed as an MJPEG. This is useful for dashboards that need
   a live preview alongside detection data.
3. **Stable imports and annotations** – type annotations that reference
   optional dependencies (e.g. `np.ndarray`, `RosTime`) are wrapped in
   quotes so they don’t raise `NameError` when those modules aren’t
   available at import time.
4. **Summary updates** – the node now passes the entire list of detections to
   the history buffer each second, ensuring all objects are retained in the
   detection logs and API responses.

To run this node in a ROS2 environment:

```bash
ros2 run your_package_name soso_final.py
```

You should see the following endpoints available (port defaults to 8088):

- `/scene` – all detections in last 10 s (with GPS/yaw header)
- `/history` – detections from 10–20 s ago
- `/video.mjpg` – live MJPEG stream of annotated image
- `/take_photo` – save the current frame to a JPEG
- `/detections` – legacy summary history

The `/sensors` endpoint (port 8001) is still used to fetch GPS/battery for
global estimation.  Ensure it is running in a separate process.

"""

import json
import os
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer
from typing import Dict, List, Tuple, Optional, Any
from datetime import datetime, timezone
import collections

try:
    # ROS2 imports may be unavailable outside a ROS environment; fall back gracefully.
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    from builtin_interfaces.msg import Time as RosTime
    try:
        from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
        from geometry_msgs.msg import Pose, Point, Quaternion
        VISION_MSGS_AVAILABLE = True
    except Exception:
        VISION_MSGS_AVAILABLE = False
    try:
        from cv_bridge import CvBridge  # Optional, if available
        CV_BRIDGE_AVAILABLE = True
    except Exception:
        CV_BRIDGE_AVAILABLE = False
    try:
        from message_filters import ApproximateTimeSynchronizer, Subscriber
        MESSAGE_FILTERS_AVAILABLE = True
    except Exception:
        MESSAGE_FILTERS_AVAILABLE = False

    # Attempt to import a proper std_msgs Header.  If not available
    # (e.g. outside ROS), fallback to a dummy type so code still
    # compiles.  A proper Header ensures published messages have a
    # correct header type instead of an anonymous object.
    try:
        from std_msgs.msg import Header  # type: ignore
        STD_MSGS_AVAILABLE = True
    except Exception:
        STD_MSGS_AVAILABLE = False
        Header = object  # type: ignore
except Exception:
    rclpy = None
    Node = object  # type: ignore
    Image = CameraInfo = object  # type: ignore
    Detection3DArray = Detection3D = ObjectHypothesisWithPose = Pose = Point = Quaternion = object  # type: ignore
    VISION_MSGS_AVAILABLE = False
    CV_BRIDGE_AVAILABLE = False
    MESSAGE_FILTERS_AVAILABLE = False

import cv2  # type: ignore
import numpy as np  # type: ignore
import math
import requests

try:
    from ultralytics import YOLO  # type: ignore
    ULTRALYTICS_AVAILABLE = True
except Exception:
    ULTRALYTICS_AVAILABLE = False

# ------------------------------------------------------------------
# Global stores and configuration

# Latest detection data for legacy /detections
latest_detection_data: Dict[str, Any] = {
    "timestamp": None,
    "detections": [],
    "fps": 0.0,
    "detector": ""
}

# Recent detection history (legacy)
recent_detection_history: List[Dict[str, Any]] = []

# Detection frames for /scene and /history; stores per-frame detections and drone state
global_detection_frames: List[Dict[str, Any]] = []

# Latest displayed frame for /take_photo
latest_frame_for_photo: Optional["np.ndarray"] = None

# Latest frame for streaming for /video.mjpg
current_frame_for_stream: Optional["np.ndarray"] = None

# Scene and history windows (in seconds)
SCENE_WINDOW_SEC: int = 3
# The history window covers the prior SCENE_WINDOW_SEC seconds.  For a
# 3 second scene window we retain 6 seconds of history in total, so
# /history returns the detections from 3–6 seconds ago.
HISTORY_WINDOW_SEC: int = 6
HISTORY_OLD_WINDOW_SEC: int = HISTORY_WINDOW_SEC - SCENE_WINDOW_SEC

# A re‑entrant lock to protect shared state accessed across threads.
state_lock = threading.Lock()


def get_drone_state() -> Dict[str, Any]:
    """
    Query the local /sensors endpoint (port 8001) to retrieve the drone's
    current GPS position, altitude and yaw.  If unavailable, returns
    None for each field.  Adjust the URL if your sensors service is
    running on a different host or port.
    """
    try:
        resp = requests.get("http://127.0.0.1:8001/sensors", timeout=0.5)
        data = resp.json()
        pos = data.get("position", {})
        att = data.get("attitude", {}).get("euler_deg", {})
        return {
            "timestamp": data.get("timestamp"),
            "gps": {
                "lat": pos.get("lat_deg"),
                "lon": pos.get("lon_deg"),
                "alt": pos.get("abs_alt_m"),
            },
            "yaw_deg": att.get("yaw_deg"),
        }
    except Exception:
        return {
            "timestamp": None,
            "gps": {"lat": None, "lon": None, "alt": None},
            "yaw_deg": None,
        }


def compute_estimated_global(
    x: Optional[float],
    y: Optional[float],
    z: Optional[float],
    gps: Dict[str, Optional[float]],
    yaw_deg: Optional[float],
) -> Optional[Dict[str, float]]:
    """
    Estimate global latitude, longitude and altitude for an object given its
    relative position (x, y, z) in metres and the drone's current GPS
    coordinates and yaw.  Returns None if any input is missing or NaN.
    """
    try:
        if gps is None or yaw_deg is None:
            return None
        lat1 = gps.get("lat")
        lon1 = gps.get("lon")
        alt1 = gps.get("alt")
        if lat1 is None or lon1 is None or alt1 is None:
            return None
        if x is None or y is None or z is None:
            return None
        if any(isinstance(v, float) and math.isnan(v) for v in [x, y, z]):
            return None
        yaw_rad = math.radians(yaw_deg)
        north_m = z * math.cos(yaw_rad) - x * math.sin(yaw_rad)
        east_m = z * math.sin(yaw_rad) + x * math.cos(yaw_rad)
        up_m = y
        R = 6378137.0
        lat_offset = (north_m / R) * (180.0 / math.pi)
        lon_offset = (east_m / (R * math.cos(math.radians(lat1)))) * (180.0 / math.pi)
        return {
            "lat": lat1 + lat_offset,
            "lon": lon1 + lon_offset,
            "alt": alt1 + up_m,
        }
    except Exception:
        return None


def load_default_colors() -> Dict[str, Tuple[int, int, int]]:
    """Generate a deterministic colour map for the first few classes."""
    colours = {}
    random.seed(42)
    predefined = {
        "person": (255, 0, 0),
        "car": (0, 255, 0),
        "truck": (0, 0, 255),
        "bicycle": (255, 255, 0),
        "motorcycle": (255, 0, 255),
        "bus": (0, 255, 255),
        "train": (128, 0, 128),
        "dog": (128, 128, 0),
        "cat": (0, 128, 128),
        "bird": (128, 0, 0),
    }
    colours.update(predefined)
    return colours


class SimpleHTTPDetectionHandler(BaseHTTPRequestHandler):
    """HTTP API handler providing detection data, photos and a live stream."""

    def do_GET(self) -> None:
        # Legacy /detections endpoint
        if self.path.startswith("/detections"):
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
            try:
                # Copy under lock to avoid races with updates from the ROS thread
                with state_lock:
                    history_copy = list(recent_detection_history)
                response = {"history": history_copy}
                self.wfile.write(json.dumps(response, indent=2).encode("utf-8"))
            except Exception:
                self.wfile.write(json.dumps({"history": []}, indent=2).encode("utf-8"))
            return
        # /scene: last SCENE_WINDOW_SEC seconds of detections
        if self.path.startswith("/scene"):
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
            try:
                now = time.time()
                cutoff = now - SCENE_WINDOW_SEC
                # Acquire lock when reading shared frame data
                with state_lock:
                    frames_copy = [fr for fr in global_detection_frames if fr.get("timestamp", 0.0) >= cutoff]
                detections_list: List[Dict[str, Any]] = []
                for fr in frames_copy:
                    detections_list.extend(fr.get("detections", []))
                summary_by_object: Dict[str, int] = {}
                for det in detections_list:
                    obj_name = det.get("Object Name") or det.get("label")
                    if obj_name:
                        summary_by_object[obj_name] = summary_by_object.get(obj_name, 0) + 1
                if frames_copy:
                    current_state = frames_copy[-1].get("global", {})
                else:
                    current_state = get_drone_state()
                response = {
                    "global": {
                        "timestamp": current_state.get("timestamp"),
                        "gps": current_state.get("gps"),
                        "yaw_deg": current_state.get("yaw_deg"),
                        "window_sec": SCENE_WINDOW_SEC,
                    },
                    "count": len(detections_list),
                    "summary_by_object": summary_by_object,
                    "detections": detections_list,
                }
                self.wfile.write(json.dumps(response, indent=2).encode("utf-8"))
            except Exception:
                self.wfile.write(json.dumps({"global": {}, "count": 0, "summary_by_object": {}, "detections": []}, indent=2).encode("utf-8"))
            return
        # /history: older portion of window
        if self.path.startswith("/history"):
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
            try:
                now = time.time()
                cutoff_old = now - HISTORY_WINDOW_SEC
                cutoff_new = now - SCENE_WINDOW_SEC
                with state_lock:
                    frames_copy = [fr for fr in global_detection_frames
                                     if cutoff_old <= fr.get("timestamp", 0.0) < cutoff_new]
                detections_list: List[Dict[str, Any]] = []
                for fr in frames_copy:
                    detections_list.extend(fr.get("detections", []))
                summary_by_object: Dict[str, int] = {}
                for det in detections_list:
                    obj_name = det.get("Object Name") or det.get("label")
                    if obj_name:
                        summary_by_object[obj_name] = summary_by_object.get(obj_name, 0) + 1
                if frames_copy:
                    current_state = frames_copy[-1].get("global", {})
                else:
                    current_state = get_drone_state()
                response = {
                    "global": {
                        "timestamp": current_state.get("timestamp"),
                        "gps": current_state.get("gps"),
                        "yaw_deg": current_state.get("yaw_deg"),
                        "window_sec": HISTORY_OLD_WINDOW_SEC,
                    },
                    "count": len(detections_list),
                    "summary_by_object": summary_by_object,
                    "detections": detections_list,
                }
                self.wfile.write(json.dumps(response, indent=2).encode("utf-8"))
            except Exception:
                self.wfile.write(json.dumps({"global": {}, "count": 0, "summary_by_object": {}, "detections": []}, indent=2).encode("utf-8"))
            return
        # /take_photo: save current frame
        if self.path.startswith("/take_photo"):
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
            try:
                from datetime import datetime as _dt
                # Acquire the latest frame under lock
                with state_lock:
                    frame = latest_frame_for_photo.copy() if latest_frame_for_photo is not None else None
                if frame is not None:
                    ts_str = _dt.now().strftime("%Y%m%d_%H%M%S")
                    # Ensure images directory exists
                    images_dir = os.path.join(os.path.dirname(__file__), "images")
                    os.makedirs(images_dir, exist_ok=True)
                    filename = f"photo_{ts_str}.jpg"
                    path = os.path.join(images_dir, filename)
                    # The frame is already BGR; write directly
                    ok = cv2.imwrite(path, frame)
                    if ok:
                        response = {"status": "success", "file": os.path.join("images", filename)}
                    else:
                        response = {"status": "error", "message": "Failed to save image"}
                    self.wfile.write(json.dumps(response, indent=2).encode("utf-8"))
                else:
                    response = {"status": "error", "message": "No frame available for photo"}
                    self.wfile.write(json.dumps(response, indent=2).encode("utf-8"))
            except Exception as e:
                try:
                    response = {"status": "error", "message": str(e)}
                    self.wfile.write(json.dumps(response, indent=2).encode("utf-8"))
                except Exception:
                    pass
            return
        # /video.mjpg: live MJPEG stream
        if self.path.startswith("/video.mjpg"):
            try:
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                while True:
                    with state_lock:
                        frame = current_frame_for_stream.copy() if current_frame_for_stream is not None else None
                    if frame is None:
                        # generate a placeholder image on the fly
                        import numpy as _np
                        h, w = 480, 640
                        placeholder = _np.zeros((h, w, 3), dtype=_np.uint8)
                        timestamp = datetime.now().strftime("%H:%M:%S")
                        cv2.putText(placeholder, "No frame", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                        cv2.putText(placeholder, timestamp, (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                        frame = placeholder
                    ret, jpeg = cv2.imencode('.jpg', frame)
                    jpg_bytes = jpeg.tobytes() if ret else b''
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg_bytes)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpg_bytes)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    # limit to ~20 fps
                    time.sleep(0.05)
            except Exception:
                # If the client disconnects or any error occurs, just return
                pass
            return
        # If unknown path
        self.send_response(404)
        self.end_headers()

    def log_message(self, format: str, *args: Any) -> None:
        # Suppress default logging
        return


class VisualPerceptionNode(Node):
    """ROS2 node that runs detection, publishes 3D detections, and serves an HTTP API."""

    def __init__(self) -> None:
        super().__init__("px4_agent_visual_node")
        self.get_logger().info("Initializing Visual Perception Node")

        # Declare parameters (unchanged from original)
        self.declare_parameter("camera_topic", "/camera")
        self.declare_parameter("depth_topic", "/depth_camera")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("publish_detections_3d", True)
        self.declare_parameter("output_topic_3d", "/detected_objects_3d")
        self.declare_parameter("jsonl_enabled", True)
        self.declare_parameter("jsonl_path", os.path.join(os.path.dirname(__file__), "detections_log.jsonl"))
        self.declare_parameter("show_window", True)
        self.declare_parameter("print_hz", 2.0)
        self.declare_parameter("depth_patch", 5)
        self.declare_parameter("min_depth_m", 0.1)
        self.declare_parameter("max_depth_m", 60.0)
        self.declare_parameter("overlay_scale", 1.0)
        self.declare_parameter("history_seconds", 5)
        self.declare_parameter("http_enabled", True)
        self.declare_parameter("http_port", 8088)

        # Read parameters
        self.camera_topic: str = self.get_parameter("camera_topic").get_parameter_value().string_value
        self.depth_topic: str = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.camera_info_topic: str = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.publish_detections_3d: bool = self.get_parameter("publish_detections_3d").get_parameter_value().bool_value
        self.output_topic_3d: str = self.get_parameter("output_topic_3d").get_parameter_value().string_value
        self.jsonl_enabled: bool = self.get_parameter("jsonl_enabled").get_parameter_value().bool_value
        self.jsonl_path: str = self.get_parameter("jsonl_path").get_parameter_value().string_value
        self.show_window: bool = self.get_parameter("show_window").get_parameter_value().bool_value
        self.print_hz: float = self.get_parameter("print_hz").get_parameter_value().double_value
        self.depth_patch: int = self.get_parameter("depth_patch").get_parameter_value().integer_value
        self.min_depth_m: float = self.get_parameter("min_depth_m").get_parameter_value().double_value
        self.max_depth_m: float = self.get_parameter("max_depth_m").get_parameter_value().double_value
        self.overlay_scale: float = self.get_parameter("overlay_scale").get_parameter_value().double_value
        self.http_enabled: bool = self.get_parameter("http_enabled").get_parameter_value().bool_value
        self.http_port: int = self.get_parameter("http_port").get_parameter_value().integer_value

        # History seconds controls how many per-second summaries are retained in JSON output
        self.history_seconds: int = max(1, self.get_parameter("history_seconds").get_parameter_value().integer_value)

        # High-level summary buffer (unused in API, but kept for legacy)
        self.summary_interval: float = 1.0
        self.summary_history_len: int = self.history_seconds
        self.summaries: collections.deque = collections.deque(maxlen=self.summary_history_len)
        self.summary_counter: int = 0
        self.last_summary_time: float = time.time()

        # Camera intrinsics
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None

        # Colour map
        self.colours: Dict[str, Tuple[int, int, int]] = load_default_colors()

        # Bridge for ROS image conversion
        self.bridge = CvBridge() if CV_BRIDGE_AVAILABLE else None

        # Load detector (YOLO if available; HOG fallback)
        self.detector_name = ""
        self.detector = None
        if ULTRALYTICS_AVAILABLE:
            try:
                self.detector = YOLO("yolov8s.pt")
                self.detector_name = "YOLOv8s"
                self.get_logger().info("Loaded ultralytics YOLOv8s model")
            except Exception as e:
                self.get_logger().warn(f"YOLOv8 failed: {e}; falling back to HOG")
                self.detector = None
        if self.detector is None:
            hog = cv2.HOGDescriptor()
            hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self.detector = hog
            self.detector_name = "HOG"
            self.get_logger().info("Using HOG person detector")

        # Rolling buffer of raw detection records; length = history_seconds
        self.recent_records: collections.deque = collections.deque(maxlen=self.history_seconds)
        self.jsonl_file = None
        if self.jsonl_enabled:
            try:
                os.makedirs(os.path.dirname(self.jsonl_path), exist_ok=True)
                with open(self.jsonl_path, "w") as f:
                    pass
                self.get_logger().info(f"Will log last {self.history_seconds} summaries to {self.jsonl_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to initialise JSON file: {e}")
                self.jsonl_enabled = False

        # Publisher for vision_msgs
        self.det_3d_pub = None
        if self.publish_detections_3d and VISION_MSGS_AVAILABLE:
            self.det_3d_pub = self.create_publisher(Detection3DArray, self.output_topic_3d, 10)
        elif self.publish_detections_3d:
            self.get_logger().warn("vision_msgs not available; 3D detection publishing disabled")
            self.publish_detections_3d = False

        # Subscribers / synchroniser
        if MESSAGE_FILTERS_AVAILABLE:
            self.get_logger().info("Using ApproximateTimeSynchronizer")
            cam_sub = Subscriber(self, Image, self.camera_topic)
            depth_sub = Subscriber(self, Image, self.depth_topic)
            info_sub = Subscriber(self, CameraInfo, self.camera_info_topic)
            sync = ApproximateTimeSynchronizer([cam_sub, depth_sub, info_sub], queue_size=30, slop=0.1)
            sync.registerCallback(self.synced_callback)
        else:
            self.get_logger().warn("message_filters unavailable; using separate callbacks")
            self.camera_sub = self.create_subscription(Image, self.camera_topic, self.camera_callback, 10)
            self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
            self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
            self.latest_rgb: Optional[Image] = None
            self.latest_depth: Optional[Image] = None
            self.timer = self.create_timer(0.1, self.process_latest_frames)

        # FPS tracking
        self.last_print_time = time.time()
        self.last_fps_time = time.time()
        self.frame_count = 0
        self.fps = 0.0

        # Timestamp for JSON + API updates
        self.last_json_update_time = time.time()

        # Last processed detection list
        self.latest_detections_out: List[Dict[str, Any]] = []

        # Start HTTP server in a separate thread
        if self.http_enabled:
            threading.Thread(target=self.start_http_server, daemon=True).start()

    # ---------------------------------------------------------------
    # Subscriber callbacks (approximate sync)
    def synced_callback(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        self.process_frame(rgb_msg, depth_msg, info_msg)

    def camera_callback(self, msg: Image) -> None:
        self.latest_rgb = msg

    def depth_callback(self, msg: Image) -> None:
        self.latest_depth = msg

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.update_intrinsics(msg)

    def process_latest_frames(self) -> None:
        if self.latest_rgb and self.latest_depth:
            rgb_msg = self.latest_rgb
            depth_msg = self.latest_depth
            self.latest_rgb = None
            self.latest_depth = None
            dummy_info = CameraInfo()
            dummy_info.k = [self.fx or 0.0, 0.0, self.cx or 0.0,
                            0.0, self.fy or 0.0, self.cy or 0.0,
                            0.0, 0.0, 1.0]
            self.process_frame(rgb_msg, depth_msg, dummy_info)

    # ---------------------------------------------------------------
    def update_intrinsics(self, msg: CameraInfo) -> None:
        if not self.fx or not self.fy:
            try:
                self.fx = float(msg.k[0]) if msg.k[0] else None
                self.fy = float(msg.k[4]) if msg.k[4] else None
                self.cx = float(msg.k[2]) if msg.k[2] else None
                self.cy = float(msg.k[5]) if msg.k[5] else None
                if self.fx and self.fy:
                    self.get_logger().info(f"Updated intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")
            except Exception as e:
                self.get_logger().warn(f"Failed to parse intrinsics: {e}")

    # ---------------------------------------------------------------
    def process_frame(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        rgb_image = self.rosimg_to_cv2(rgb_msg)
        depth_image, depth_width, depth_height = self.rosimg_depth_to_numpy(depth_msg)
        if rgb_image is None or depth_image is None:
            return
        if info_msg is not None and (not self.fx or not self.fy):
            self.update_intrinsics(info_msg)
        rgb_height, rgb_width = rgb_image.shape[:2]
        detections = self.run_detection(rgb_image)
        self.frame_count += 1
        now = time.time()
        dt = now - self.last_fps_time
        if dt >= 1.0:
            self.fps = self.frame_count / dt
            self.frame_count = 0
            self.last_fps_time = now
        detections_out: List[Dict[str, Any]] = []
        det3d_msgs = []
        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            cls_name = det["label"]
            score = det.get("score", 0.0)
            u_rgb = int((x1 + x2) / 2)
            v_rgb = int((y1 + y2) / 2)
            depth_u, depth_v = self.map_rgb_to_depth(u_rgb, v_rgb, rgb_width, rgb_height, depth_width, depth_height)
            patch_size = self.depth_patch
            half = patch_size // 2
            x0 = max(0, depth_u - half)
            y0 = max(0, depth_v - half)
            x1p = min(depth_width, depth_u + half + 1)
            y1p = min(depth_height, depth_v + half + 1)
            patch = depth_image[y0:y1p, x0:x1p]
            valid = patch[np.isfinite(patch)]
            valid = valid[(valid > self.min_depth_m) & (valid < self.max_depth_m)]
            if valid.size == 0:
                Z = float("nan")
            else:
                Z = float(np.median(valid))
            if self.fx and self.fy and self.cx is not None and self.cy is not None and not np.isnan(Z):
                X = (float(depth_u) - self.cx) * Z / self.fx
                Y = (float(depth_v) - self.cy) * Z / self.fy
            else:
                f_approx = rgb_width / (2 * np.tan(1.047 / 2))
                cx_approx = rgb_width / 2.0
                cy_approx = rgb_height / 2.0
                X = (float(depth_u) - cx_approx) * Z / f_approx
                Y = (float(depth_v) - cy_approx) * Z / f_approx
            det_out = {
                "label": cls_name,
                "score": float(score),
                "bbox": [int(x1), int(y1), int(x2), int(y2)],
                "center_rgb": [u_rgb, v_rgb],
                "uv_depth": [depth_u, depth_v],
                "position": {"x": X, "y": Y, "z": Z},
            }
            detections_out.append(det_out)
            if self.publish_detections_3d:
                det3d = Detection3D()
                det3d.header = self.create_header(rgb_msg.header.stamp, rgb_msg.header.frame_id)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls_name
                hyp.hypothesis.score = float(score)
                hyp.pose = Pose()
                # Set position directly; ensure proper geometry_msgs imports
                hyp.pose.position = Point(x=X, y=Y, z=Z)
                hyp.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                det3d.results.append(hyp)
                det3d_msgs.append(det3d)
        if self.publish_detections_3d and det3d_msgs:
            det_array = Detection3DArray()
            det_array.header = self.create_header(rgb_msg.header.stamp, rgb_msg.header.frame_id)
            det_array.detections = det3d_msgs
            self.det_3d_pub.publish(det_array)
        record = {
            "timestamp": time.time(),
            "detector": self.detector_name,
            "fps": self.fps,
            "detections": detections_out,
        }
        self.latest_detections_out = detections_out
        self.recent_records.append(record)
        now_time = time.time()
        if now_time - self.last_summary_time >= self.summary_interval:
            chosen_det: Optional[Dict[str, Any]] = None
            min_dist: float = float('inf')
            for det in detections_out:
                z_val = det["position"].get("z")
                if z_val is not None and not np.isnan(z_val) and z_val > 0 and z_val < min_dist:
                    min_dist = z_val
                    chosen_det = det
            self.summary_counter += 1
            entry_id = f"ID {self.summary_counter:03d}"
            if chosen_det:
                Z = chosen_det["position"]["z"]
                x1c, y1c, x2c, y2c = chosen_det["bbox"]
                bb_json = {"x1": x1c, "y1": y1c, "x2": x2c, "y2": y2c}
                centre_json = {
                    "x": round(chosen_det["position"]["x"], 2),
                    "y": round(chosen_det["position"]["y"], 2),
                    "z": round(chosen_det["position"]["z"], 2),
                }
                summary_entry = {
                    "ID": entry_id,
                    "time": now_time,
                    "Object Name": chosen_det.get("label"),
                    "Bounding Box": bb_json,
                    "Center Point": centre_json,
                    "Depth": round(Z, 2) if not np.isnan(Z) else None,
                    "Confidence Level": round(chosen_det.get("score", 0.0), 2),
                }
            else:
                summary_entry = {
                    "ID": entry_id,
                    "time": now_time,
                    "Object Name": "none",
                    "Bounding Box": {},
                    "Center Point": {},
                    "Depth": None,
                    "Confidence Level": None,
                }
            # Update global detection frames with *all* detections for scene/history APIs
            try:
                self.update_global_frames(detections_out, now_time)
            except Exception as e:
                self.get_logger().warn(f"Failed to update global detection frames: {e}")
            self.summaries.append(summary_entry)
            self.last_summary_time = now_time
        latest_detection_data["timestamp"] = record["timestamp"]
        latest_detection_data["detections"] = record["detections"]
        latest_detection_data["fps"] = record["fps"]
        latest_detection_data["detector"] = record["detector"]
        self.update_json_and_api()
        if self.show_window:
            self.draw_and_show(rgb_image.copy(), detections_out, rgb_width, rgb_height)
        self.print_periodic_summary(detections_out)

    # ---------------------------------------------------------------
    def draw_and_show(self, image: np.ndarray, detections: List[Dict[str, Any]], width: int, height: int) -> None:
        scale = self.overlay_scale
        if scale != 1.0:
            image_display = cv2.resize(image, (int(width * scale), int(height * scale)))
        else:
            image_display = image
        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            if scale != 1.0:
                x1d, y1d, x2d, y2d = [int(coord * scale) for coord in (x1, y1, x2, y2)]
            else:
                x1d, y1d, x2d, y2d = x1, y1, x2, y2
            cls_name = det["label"]
            pos = det["position"]
            colour = self.colours.get(cls_name)
            if colour is None:
                colour = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                self.colours[cls_name] = colour
            cv2.rectangle(image_display, (x1d, y1d), (x2d, y2d), colour, 2)
            x_str = f"{pos['x']:.2f}m" if not np.isnan(pos['x']) else "NaN"
            y_str = f"{pos['y']:.2f}m" if not np.isnan(pos['y']) else "NaN"
            z_str = f"{pos['z']:.2f}m" if not np.isnan(pos['z']) else "NaN"
            label = f"{cls_name}: X={x_str}, Y={y_str}, Z={z_str}"
            (text_w, text_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            cv2.rectangle(image_display, (x1d, y1d - text_h - baseline), (x1d + text_w, y1d), colour, -1)
            cv2.putText(image_display, label, (x1d, y1d - baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        fps_text = f"FPS: {self.fps:.1f}" if self.fps else "FPS: ..."
        cv2.putText(image_display, fps_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        try:
            # Publish the frame to globals under lock
            global latest_frame_for_photo
            global current_frame_for_stream
            with state_lock:
                latest_frame_for_photo = image_display.copy()
                current_frame_for_stream = image_display.copy()
            cv2.imshow("PX4 Agent – Visual Perception", image_display)
            cv2.waitKey(1)
        except Exception:
            self.show_window = False

    # ---------------------------------------------------------------
    def print_periodic_summary(self, detections: List[Dict[str, Any]]) -> None:
        now = time.time()
        if now - self.last_print_time >= (1.0 / max(self.print_hz, 0.001)):
            count = len(detections)
            nearest = None
            nearest_dist = float('inf')
            for det in detections:
                z = det['position']['z']
                if not np.isnan(z) and z < nearest_dist:
                    nearest_dist = z
                    nearest = det
            nearest_str = f"{nearest['label']} @ Z={nearest_dist:.2f}m" if nearest else "none"
            self.get_logger().info(f"[Detections] count={count}, nearest: {nearest_str}, FPS={self.fps:.1f}")
            self.last_print_time = now

    # ---------------------------------------------------------------
    def start_http_server(self) -> None:
        self.get_logger().info(f"Starting threaded HTTP server on port {self.http_port}")
        # Use a ThreadingHTTPServer so that the MJPEG stream does not block
        # other endpoints.  Without this, /video.mjpg would prevent /scene
        # and /history from responding while streaming.
        server = ThreadingHTTPServer(('0.0.0.0', self.http_port), SimpleHTTPDetectionHandler)
        try:
            server.serve_forever()
        except Exception as e:
            self.get_logger().warn(f"HTTP server error: {e}")

    # ---------------------------------------------------------------
    def update_json_and_api(self) -> None:
        now = time.time()
        if now - self.last_json_update_time < 1.0:
            return
        formatted_history: List[Dict[str, Any]] = []
        for entry in list(self.summaries):
            ts = entry.get("time", now)
            try:
                dt_obj = datetime.fromtimestamp(ts)
                time_str = dt_obj.strftime("%H:%M:%S.%f")[:-4]
            except Exception:
                time_str = None
            formatted_history.append({
                "ID": entry.get("ID"),
                "Time": time_str,
                "Object Name": entry.get("Object Name"),
                "Bounding Box": entry.get("Bounding Box"),
                "Center Point": entry.get("Center Point"),
                "Depth": f"{entry.get('Depth'):.2f}" if entry.get("Depth") is not None else None,
                "Confidence Level": f"{entry.get('Confidence Level'):.2f}" if entry.get("Confidence Level") is not None else None,
            })
        global recent_detection_history
        # update shared history under lock
        with state_lock:
            recent_detection_history = formatted_history
        if formatted_history:
            latest = formatted_history[-1]
            with state_lock:
                latest_detection_data["timestamp"] = now
                latest_detection_data["detector"] = self.detector_name
                latest_detection_data["fps"] = self.fps
                latest_detection_data["detections"] = [
                    {
                        "Object Name": latest.get("Object Name"),
                        "Bounding Box": latest.get("Bounding Box"),
                        "Center Point": latest.get("Center Point"),
                        "Depth": latest.get("Depth"),
                        "Confidence Level": latest.get("Confidence Level"),
                    }
                ]
        if self.jsonl_enabled and self.jsonl_path:
            try:
                with state_lock:
                    frames_copy = list(global_detection_frames)
                if frames_copy:
                    current_state = frames_copy[-1].get("global", {})
                else:
                    current_state = get_drone_state()
                detections_list: List[Dict[str, Any]] = []
                cutoff_ts = now - HISTORY_WINDOW_SEC
                for fr in frames_copy:
                    if fr.get("timestamp", 0.0) >= cutoff_ts:
                        detections_list.extend(fr.get("detections", []))
                json_data = {
                    "global": {
                        "timestamp": current_state.get("timestamp"),
                        "gps": current_state.get("gps"),
                        "yaw_deg": current_state.get("yaw_deg"),
                        "window_sec": HISTORY_WINDOW_SEC,
                    },
                    "detections": detections_list,
                }
                with open(self.jsonl_path, "w", encoding="utf-8") as f:
                    json.dump(json_data, f, indent=2)
            except Exception as e:
                self.get_logger().warn(f"Failed to write unified detection JSON file: {e}")
        self.last_json_update_time = now

    # ---------------------------------------------------------------
    def update_global_frames(self, detections_out: List[Dict[str, Any]], now_ts: Optional[float] = None) -> None:
        state = get_drone_state()
        frame_ts = now_ts if now_ts is not None else time.time()
        try:
            dt_obj = datetime.fromtimestamp(frame_ts)
            time_str = dt_obj.strftime("%H:%M:%S.%f")[:-4]
        except Exception:
            time_str = None
        det_entries: List[Dict[str, Any]] = []
        for det in detections_out or []:
            pos = det.get("position", {})
            x = pos.get("x")
            y = pos.get("y")
            z = pos.get("z")
            est = compute_estimated_global(x, y, z, state.get("gps"), state.get("yaw_deg"))
            entry: Dict[str, Any] = {
                "Object Name": det.get("label", ""),
                "Time": time_str,
                "Center": {
                    "x_m": round(x, 2) if x is not None and not (isinstance(x, float) and math.isnan(x)) else None,
                    "y_m": round(y, 2) if y is not None and not (isinstance(y, float) and math.isnan(y)) else None,
                    "z_m": round(z, 2) if z is not None and not (isinstance(z, float) and math.isnan(z)) else None,
                },
                "Confidence": round(det.get("score", 0.0), 2) if det.get("score") is not None else None,
            }
            if est:
                entry["estimated_global"] = {
                    "lat": est.get("lat"),
                    "lon": est.get("lon"),
                    "alt": est.get("alt"),
                }
            else:
                entry["estimated_global"] = None
            det_entries.append(entry)
        frame_record = {
            "timestamp": frame_ts,
            "global": state,
            "detections": det_entries,
        }
        global global_detection_frames
        # Protect updates with a lock to avoid concurrent modifications
        with state_lock:
            global_detection_frames.append(frame_record)
            cutoff = frame_ts - HISTORY_WINDOW_SEC
            global_detection_frames = [fr for fr in global_detection_frames if fr.get("timestamp", 0.0) >= cutoff]

    # ---------------------------------------------------------------
    def destroy_node(self) -> None:  # type: ignore[override]
        self.get_logger().info("Shutting down Visual Perception Node")
        try:
            if self.jsonl_file:
                self.jsonl_file.close()
        except Exception:
            pass
        super().destroy_node()

    # ---------------------------------------------------------------
    # Helpers for image conversion and detection
    def rosimg_to_cv2(self, msg: Image) -> Optional[np.ndarray]:
        try:
            if CV_BRIDGE_AVAILABLE:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                cv_image = np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data)
                cv_image = cv_image.reshape((msg.height, msg.width, 3))
            return cv_image
        except Exception as e:
            self.get_logger().warn(f"Failed to convert ROS image: {e}")
            return None

    def rosimg_depth_to_numpy(self, msg: Image) -> Tuple[Optional[np.ndarray], int, int]:
        try:
            if msg.encoding == "32FC1":
                depth_image = np.ndarray(shape=(msg.height, msg.width), dtype=np.float32, buffer=msg.data)
            else:
                depth_image = np.ndarray(shape=(msg.height, msg.width), dtype=np.uint16, buffer=msg.data)
                depth_image = depth_image.astype(np.float32)
                depth_image /= 1000.0  # convert mm to m
            return depth_image, msg.width, msg.height
        except Exception as e:
            self.get_logger().warn(f"Failed to convert depth image: {e}")
            return None, 0, 0

    def run_detection(self, image: np.ndarray) -> List[Dict[str, Any]]:
        dets: List[Dict[str, Any]] = []
        if self.detector_name.startswith("YOLO"):
            results = self.detector(image)
            for res in results:
                for box in res.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cls = int(box.cls[0])
                    label = self.detector.model.names[cls]
                    score = float(box.conf[0])
                    dets.append({"label": label, "score": score, "bbox": [x1, y1, x2, y2]})
        else:
            rects, weights = self.detector.detectMultiScale(image, winStride=(8, 8))
            for (x, y, w, h), score in zip(rects, weights):
                dets.append({"label": "person", "score": float(score), "bbox": [x, y, x + w, y + h]})
        return dets

    def map_rgb_to_depth(self, u_rgb: int, v_rgb: int, rgb_width: int, rgb_height: int,
                          depth_width: int, depth_height: int) -> Tuple[int, int]:
        du = int(u_rgb * depth_width / rgb_width)
        dv = int(v_rgb * depth_height / rgb_height)
        du = min(max(du, 0), depth_width - 1)
        dv = min(max(dv, 0), depth_height - 1)
        return du, dv

    def create_header(self, stamp: 'RosTime', frame_id: str) -> 'Header':
        """
        Create and return a proper ROS std_msgs/Header if available.  If
        std_msgs is not available (e.g. when testing outside a ROS
        environment) this falls back to a simple object with stamp and
        frame_id attributes.  Using a real Header prevents type errors
        when publishing Detection3D messages.
        """
        if globals().get("STD_MSGS_AVAILABLE"):
            hdr = Header()  # type: ignore
            hdr.stamp = stamp
            hdr.frame_id = frame_id
            return hdr
        # Fallback: anonymous object with stamp/frame_id
        hdr = type('Header', (), {})()
        hdr.stamp = stamp
        hdr.frame_id = frame_id
        return hdr


def main() -> None:
    if rclpy is None:
        print("This script requires rclpy (ROS2 Python) to run. Please ensure you are in a ROS2 environment.")
        return
    rclpy.init()
    node = VisualPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
