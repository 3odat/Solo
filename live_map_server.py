#!/usr/bin/env python3
# live_map_server.py (v6: field-path compatible + breadcrumb)
import json, sys, webbrowser, urllib.parse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import urlopen
from urllib.error import URLError

PORT = 8002
SENSORS_BASE = "http://localhost:8001/sensors"

HTML = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<title>UAV Live Map</title>
<meta name="viewport" content="width=device-width,initial-scale=1.0"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  html,body,#map { height:100%; margin:0; }
  #hud {
    position:absolute; top:12px; left:12px; z-index:1000;
    background:rgba(0,0,0,.6); color:#fff; padding:8px 10px; border-radius:10px;
    font:14px/1.2 system-ui, -apple-system, Segoe UI, Roboto, Arial; white-space:nowrap;
    box-shadow:0 4px 14px rgba(0,0,0,.25);
  }
  .leaflet-container { background:#f5f6f8; }
</style>
</head>
<body>
<div id="map"></div>
<div id="hud">Connecting…</div>

<script>
const get = (o, p, d=undefined) => p.split('.').reduce((a,k)=> (a&&a[k]!=null)?a[k]:d, o);
function num(v, def=NaN){ if(typeof v==='number') return v; if(typeof v==='string'){const m=v.match(/-?\d+(\.\d+)?/); return m?parseFloat(m[0]):def;} return def; }
const clamp360 = d => ((d%360)+360)%360;
function distMeters(a,b){ const R=6371000, dLat=(b[0]-a[0])*Math.PI/180, dLon=(b[1]-a[1])*Math.PI/180;
  const lat1=a[0]*Math.PI/180, lat2=b[0]*Math.PI/180;
  const h=Math.sin(dLat/2)**2 + Math.cos(lat1)*Math.cos(lat2)*Math.sin(dLon/2)**2;
  return 2*R*Math.asin(Math.sqrt(h));
}

// Map
const map = L.map('map').setView([47.3967, 8.5497], 19);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{
  attribution:'© OpenStreetMap contributors'
}).addTo(map);

// Trail
let trail = [];
const poly = L.polyline(trail, { color:'red', weight:3 }).addTo(map);
const MAX_POINTS = 40000;

// Inline-SVG marker with a rotatable needle group
const svgHTML = `<svg viewBox="0 0 40 40" width="40" height="40" style="overflow:visible">
  <circle cx="20" cy="20" r="18" fill="rgba(255,0,0,0.08)" stroke="rgba(0,0,0,0.15)" stroke-width="1"/>
  <g id="needle" transform="rotate(0 20 20)"><polygon points="20,2 30,30 20,24 10,30" fill="red"/></g>
</svg>`;
const arrowIcon = L.divIcon({ className:'uav-icon', html: svgHTML, iconSize:[40,40], iconAnchor:[20,20] });
const marker = L.marker([47.3967, 8.5497], { icon: arrowIcon }).addTo(map);
function rotateMarker(deg){ const el=marker.getElement(); if(!el) return; const g=el.querySelector('#needle'); if(!g) return; g.setAttribute('transform', `rotate(${clamp360(deg)} 20 20)`); }

// HUD
const hud = document.getElementById('hud');
function setHUD(o){
  const {mode='—', batt='—', sats='—', ts='', pts=0, head='—', lat='—', lon='—', d='—', srcPos='?', srcHead='?'} = o||{};
  const latS = (typeof lat==='number')? lat.toFixed(6) : lat;
  const lonS = (typeof lon==='number')? lon.toFixed(6) : lon;
  hud.textContent = `Mode:${mode} | Batt:${batt}% | Sats:${sats} | Pts:${pts} | Head:${head}° | Δm:${d} | lat:${latS} lon:${lonS} | SRC pos:${srcPos} head:${srcHead} | ${ts}`;
}

// robust field readers (handles both top-level and nested under gps)
function readPosition(data){
  const paths = [
    ['position.lat_deg', 'position.lon_deg', 'position'],
    ['gps.position.lat_deg', 'gps.position.lon_deg', 'gps.position'],
    ['gps.lat_deg', 'gps.lon_deg', 'gps'],
    ['lat_deg', 'lon_deg', '(root)']
  ];
  for (const [plat,plon,tag] of paths){
    const lat = num(get(data, plat));
    const lon = num(get(data, plon));
    if (!Number.isNaN(lat) && !Number.isNaN(lon)) return {lat, lon, src: tag};
  }
  return {lat: NaN, lon: NaN, src: 'not-found'};
}

function readHeading(data){
  const paths = [
    ['heading_deg', 'heading_deg'],
    ['attitude.euler_deg.yaw_deg', 'euler_deg']
  ];
  for (const [p,tag] of paths){
    const h = num(get(data, p));
    if (!Number.isNaN(h)) return {heading: clamp360(h), src: tag};
  }
  return {heading: 0, src: 'fallback0'};
}

// fetch with cache-buster; try direct 8001 first, then proxy
async function fetchSensors(){
  const q = `?ts=${Date.now()}`;
  try {
    const direct = await fetch(`http://localhost:8001/sensors${q}`, {cache:'no-store'});
    if (direct.ok) return await direct.json();
  } catch(e) {}
  const prox = await fetch(`/sensors${q}`, {cache:'no-store'});
  return await prox.json();
}

let firstFix = true;
let lastHere = null;

async function tick(){
  try {
    const data = await fetchSensors();

    const {lat, lon, src: srcPos} = readPosition(data);
    if (Number.isNaN(lat) || Number.isNaN(lon)) { setHUD({mode:get(data,'status.flight_mode','—'), srcPos}); return; }
    const here = [lat, lon];

    const {heading, src: srcHead} = readHeading(data);
    marker.setLatLng(here);
    rotateMarker(heading);

    // Breadcrumb every tick
    trail.push(here);
    if (trail.length > MAX_POINTS) trail.shift();
    poly.setLatLngs(trail);

    // Follow
    if (firstFix) { map.setView(here, 19); firstFix = false; } else { map.panTo(here, {animate:true}); }

    const delta = lastHere ? distMeters(lastHere, here).toFixed(2) : '—';
    lastHere = here;

    setHUD({
      mode: get(data,'status.flight_mode','—'),
      batt: (Number.isNaN(num(get(data,'battery.remaining')))? '—' : num(get(data,'battery.remaining'))),
      sats: get(data,'gps.num_satellites','—'),
      ts: get(data,'timestamp',''),
      pts: trail.length, head: Math.round(heading), lat, lon, d: delta,
      srcPos, srcHead
    });
  } catch (e) {
    hud.textContent = 'Lost connection to /sensors…';
  }
}

setInterval(tick, 1000);
tick();
</script>
</body>
</html>
"""

class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/map"):
            self.send_response(200)
            self.send_header("Content-Type","text/html; charset=utf-8")
            self.send_header("Cache-Control","no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma","no-cache")
            self.send_header("Expires","0")
            self.end_headers()
            self.wfile.write(HTML.encode("utf-8"))
            return

        if self.path.startswith("/sensors"):
            parsed = urllib.parse.urlparse(self.path)
            qs = f"?{parsed.query}" if parsed.query else ""
            target = f"{SENSORS_BASE}{qs}"
            try:
                with urlopen(target, timeout=3) as r:
                    body = r.read()
                self.send_response(200)
                self.send_header("Content-Type","application/json")
                self.send_header("Cache-Control","no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("Pragma","no-cache")
                self.send_header("Expires","0")
                self.end_headers()
                self.wfile.write(body)
            except URLError as e:
                msg = json.dumps({"error": str(e), "target": target}).encode("utf-8")
                self.send_response(502)
                self.send_header("Content-Type","application/json")
                self.end_headers()
                self.wfile.write(msg)
            return

        super().do_GET()

if __name__ == "__main__":
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    url = f"http://127.0.0.1:{PORT}/"
    print(f"📡 Serving live map on {url}")
    print(f"↪ Proxying /sensors → {SENSORS_BASE}")
    try: webbrowser.open(url)
    except Exception: pass
    try: server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down…"); server.server_close(); sys.exit(0)

