#!/usr/bin/env python3
# UAV Mission Dashboard — chat-style missions + full telemetry/map/video/gallery
# Run: python3 uav_dashboard_ultra.py  -> http://127.0.0.1:8900/

import json, sys, urllib.parse, webbrowser, threading, mimetypes
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import urlopen, Request
from urllib.error import URLError
from pathlib import Path
from typing import List

# -----------------------------
# Config (adjust if you need)
# -----------------------------
PORT = 8900

SENSORS_URL = "http://localhost:8001/sensors"
SCENE_URL   = "http://localhost:8088/scene"
VIDEO_URL   = "http://localhost:8088/video.mjpg"

# Your FastAPI agent base (the one you shared)
AGENT_BASE  = "http://localhost:8005"

# Images served from local working directory ./images
SCRIPT_DIR  = Path(__file__).resolve().parent
IMAGES_DIR  = SCRIPT_DIR / "images"
ALLOWED_EXT = {".jpg", ".jpeg", ".png", ".gif", ".webp"}

# -----------------------------
# Page
# -----------------------------
HTML = r"""<!doctype html>
<html lang="en" class="h-full">
<head>
<meta charset="utf-8" />
<title>UAV Mission Dashboard</title>
<meta name="viewport" content="width=device-width,initial-scale=1" />
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600&display=swap" rel="stylesheet">
<script src="https://cdn.tailwindcss.com"></script>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  :root { --card:#ffffff; --muted:#64748b; --ink:#0f172a; --glass:rgba(255,255,255,.65); }
  .dark :root { --card:#0b1020; --muted:#94a3b8; --ink:#e2e8f0; --glass:rgba(15,20,35,.6); }
  body { font-family: Inter, ui-sans-serif, system-ui, -apple-system, "Segoe UI", Roboto, Arial; }
  .card { border-radius:16px; box-shadow:0 10px 30px rgba(0,0,0,.06); background:var(--card); }
  .glass { background:var(--glass); backdrop-filter: blur(6px); }
  .scroll-slim::-webkit-scrollbar { height:8px; width:8px; }
  .scroll-slim::-webkit-scrollbar-thumb { background:#e2e8f0; border-radius:8px; }
  .dark .scroll-slim::-webkit-scrollbar-thumb { background:#334155; }
  #toasts { position: fixed; right:16px; top:16px; z-index:9999; display:flex; flex-direction:column; gap:8px; }
  .toast { padding:10px 12px; border-radius:10px; color:var(--ink); background:var(--card); box-shadow:0 10px 30px rgba(0,0,0,.12); border:1px solid rgba(226,232,240,.8); }
  .ok{ border-color:#86efac } .warn{ border-color:#facc15 } .err{ border-color:#fca5a5 }
  #hud-canvas { position:absolute; inset:0; pointer-events:none; }
  #map { width:100%; height:100%; }
  .badge { font-size:12px; padding:2px 8px; border-radius:9999px; }
  .kbd { border:1px solid #cbd5e1; border-bottom-width:2px; padding:1px 6px; border-radius:6px; background:#fff; }
  .dark .kbd { border-color:#334155; background:#0b1220; color:#e2e8f0; }
  .chat-item:hover { background:rgba(148,163,184,.12); }
</style>
</head>
<body class="min-h-full bg-gradient-to-br from-slate-50 to-slate-100 dark:from-[#050816] dark:to-[#0b1020] text-[15px] text-slate-900 dark:text-slate-200">

<div id="toasts"></div>

<header class="sticky top-0 z-20 glass border-b border-white/50 dark:border-white/10">
  <div class="mx-auto max-w-[1500px] px-4 py-3 flex items-center gap-3">
    <div class="w-2 h-2 rounded-full bg-green-500" style="box-shadow:0 0 0 rgba(34,197,94,.7); animation:pulse 2s infinite"></div>
    <h1 class="font-semibold text-lg">UAV Mission Dashboard</h1><span class="text-xs text-slate-500 dark:text-slate-400">• live</span>
    <div class="ml-auto flex items-center gap-2 text-sm">
      <span id="hud-mode" class="badge bg-slate-900/90 text-white dark:bg-slate-100 dark:text-slate-900">Mode: —</span>
      <span id="hud-batt" class="badge bg-emerald-100 text-emerald-700 dark:bg-emerald-900/40 dark:text-emerald-300">Batt: —%</span>
      <span id="hud-sats" class="badge bg-sky-100 text-sky-700 dark:bg-sky-900/40 dark:text-sky-300">Sats: —</span>
      <span id="hud-alt"  class="badge bg-amber-100 text-amber-700 dark:bg-amber-900/40 dark:text-amber-300">Alt: — m</span>
      <span id="hud-spd"  class="badge bg-indigo-100 text-indigo-700 dark:bg-indigo-900/40 dark:text-indigo-300">Speed: — m/s</span>
      <span id="hud-ts"   class="hidden md:inline text-xs text-slate-500 dark:text-slate-400"></span>
      <button id="theme" class="ml-2 text-xs px-2 py-1 rounded-md bg-slate-100 hover:bg-slate-200 dark:bg-[#0f172a] dark:hover:bg-[#13213a]">🌙</button>
    </div>
  </div>
</header>

<!-- Layout: video + map + right panel + chat sidebar -->
<main class="mx-auto max-w-[1500px] px-4 py-4 grid grid-cols-1 xl:grid-cols-[1.05fr_1.05fr_360px_330px] gap-4">

  <!-- Video -->
  <section class="relative card overflow-hidden min-h-[420px]">
    <div class="p-3 flex items-center justify-between border-b border-slate-100/70 dark:border-white/10">
      <div class="flex items-center gap-2"><span class="font-semibold">Live Video</span><span class="text-xs text-slate-500 dark:text-slate-400">/video.mjpg</span></div>
      <div class="text-xs text-slate-500 dark:text-slate-400">Tip: press <span class="kbd">f</span> for fullscreen</div>
    </div>
    <div class="relative h-[calc(100%-48px)]">
      <!-- Fill container without letterboxing -->
      <img id="video" src="/video.mjpg" class="w-full h-full object-cover bg-black select-none" alt="Live video"/>
      <canvas id="hud-canvas"></canvas>
      <div class="absolute top-3 right-3 flex gap-2">
        <button id="video-reload" class="px-3 py-1 text-xs rounded-md bg-slate-900/80 text-white">Reload</button>
        <button id="video-full"   class="px-3 py-1 text-xs rounded-md bg-slate-900/80 text-white">Fullscreen</button>
      </div>
    </div>
  </section>

  <!-- Map -->
  <section class="card overflow-hidden min-h-[420px]">
    <div class="p-3 flex items-center justify-between border-b border-slate-100/70 dark:border-white/10">
      <div class="flex items-center gap-2"><span class="font-semibold">Live Map</span><span class="text-xs text-slate-500 dark:text-slate-400">breadcrumb • heading • detection markers</span></div>
      <div class="flex items-center gap-2 text-xs">
        <button id="recenter" class="px-2 py-1 rounded-md bg-slate-100 hover:bg-slate-200 dark:bg-[#0f172a] dark:hover:bg-[#13213a]">Center</button>
        <button id="zoom-in"  class="px-2 py-1 rounded-md bg-slate-100 hover:bg-slate-200 dark:bg-[#0f172a] dark:hover:bg-[#13213a]">＋</button>
        <button id="zoom-out" class="px-2 py-1 rounded-md bg-slate-100 hover:bg-slate-200 dark:bg-[#0f172a] dark:hover:bg-[#13213a]">－</button>
      </div>
    </div>
    <div id="map" class="h-[calc(100%-48px)]"></div>
  </section>

  <!-- Panels -->
  <aside class="card overflow-hidden">
    <div class="p-3 border-b border-slate-100/70 dark:border-white/10 flex items-center gap-2">
      <span class="font-semibold">Panels</span>
      <button id="toggle-telemetry" class="ml-auto text-xs px-3 py-1 rounded-md bg-slate-100 hover:bg-slate-200 dark:bg-[#0f172a] dark:hover:bg-[#13213a]">Telemetry</button>
      <button id="toggle-detections" class="text-xs px-3 py-1 rounded-md bg-slate-100 hover:bg-slate-200 dark:bg-[#0f172a] dark:hover:bg-[#13213a]">Detections</button>
      <button id="toggle-gallery"   class="text-xs px-3 py-1 rounded-md bg-slate-100 hover:bg-slate-200 dark:bg-[#0f172a] dark:hover:bg-[#13213a]">Gallery</button>
    </div>
    <div class="p-3 space-y-3 overflow-auto scroll-slim" style="max-height: calc(100vh - 200px)">
      <!-- Telemetry -->
      <div id="panel-telemetry" class="glass rounded-xl p-3 hidden">
        <h3 class="font-semibold mb-2">Telemetry</h3>
        <div id="telemetry-grid" class="grid grid-cols-2 gap-2 text-sm"></div>
        <div class="mt-3">
          <canvas id="batt-spark" class="w-full" height="40"></canvas>
          <div class="text-[11px] text-slate-500 dark:text-slate-400 mt-1">Battery (last 2 min)</div>
        </div>
      </div>
      <!-- Detections -->
      <div id="panel-detections" class="glass rounded-xl p-3 hidden">
        <h3 class="font-semibold mb-2">Detections</h3>
        <div id="detections-list" class="space-y-2 max-h-[360px] overflow-auto scroll-slim text-sm"></div>
      </div>
      <!-- Gallery -->
      <div id="panel-gallery" class="glass rounded-xl p-3 hidden">
        <div class="flex items-center justify-between">
          <h3 class="font-semibold">Gallery</h3>
          <button id="refresh-gallery" class="text-xs px-2 py-1 rounded-md bg-slate-100 hover:bg-slate-200 dark:bg-[#0f172a] dark:hover:bg-[#13213a]">Refresh</button>
        </div>
        <div id="gallery-grid" class="mt-2 grid grid-cols-3 gap-2 max-h-[360px] overflow-auto scroll-slim"></div>
        <div class="mt-2 text-xs text-slate-500 dark:text-slate-400">Served from local <code>./images</code>.</div>
      </div>
    </div>
  </aside>

  <!-- Chats sidebar (GPT-style history) -->
  <aside class="card overflow-hidden">
    <div class="p-3 border-b border-slate-100/70 dark:border-white/10 flex items-center gap-2">
      <span class="font-semibold">Chats</span>
      <div class="ml-auto flex gap-2 text-xs">
        <button id="chats-refresh" class="px-2 py-1 rounded-md bg-slate-100 hover:bg-slate-200 dark:bg-[#0f172a] dark:hover:bg-[#13213a]">Refresh</button>
        <button id="chats-clear-done" class="px-2 py-1 rounded-md bg-slate-100 hover:bg-slate-200 dark:bg-[#0f172a] dark:hover:bg-[#13213a]">Clear done</button>
      </div>
    </div>
    <div id="chats-list" class="p-3 space-y-2 overflow-auto scroll-slim" style="max-height: calc(100vh - 200px)"></div>
  </aside>

</main>

<!-- Mission Output -->
<section id="mission-output-wrap" class="mx-auto max-w-[1100px] px-4 mb-4">
  <div class="card border border-slate-200/70 dark:border-white/10">
    <div class="p-3 flex items-center gap-2 border-b border-slate-100/70 dark:border-white/10">
      <div class="font-semibold">Mission</div>
      <span id="mo-title" class="text-xs px-2 py-1 rounded-md bg-slate-100 dark:bg-[#0f172a]">—</span>
      <span id="mo-status" class="text-xs px-2 py-1 rounded-md bg-slate-100 dark:bg-[#0f172a]">—</span>
      <span id="mo-photos" class="text-xs px-2 py-1 rounded-md bg-slate-100 dark:bg-[#0f172a]">photos: 0</span>
      <div class="ml-auto flex items-center gap-2 text-xs">
        <button id="mo-copy"    class="px-2 py-1 rounded-md bg-slate-100 hover:bg-slate-200 dark:bg-[#0f172a] dark:hover:bg-[#13213a]">Copy</button>
        <button id="mo-clear"   class="px-2 py-1 rounded-md bg-slate-100 hover:bg-slate-200 dark:bg-[#0f172a] dark:hover:bg-[#13213a]">Clear</button>
        <button id="mo-download" class="px-2 py-1 rounded-md bg-slate-100 hover:bg-slate-200 dark:bg-[#0f172a] dark:hover:bg-[#13213a]">Download .txt</button>
      </div>
    </div>
    <pre id="mission-output" class="px-3 py-3 whitespace-pre-wrap text-sm overflow-auto scroll-slim" style="max-height: 45vh; font-family: ui-monospace, Menlo, Consolas, monospace;"></pre>
  </div>
</section>

<!-- Prompt bar -->
<form id="prompt-form" class="fixed bottom-0 inset-x-0 z-30">
  <div class="mx-auto max-w-[1100px] px-4 pb-4">
    <div class="glass card border border-white/60 dark:border-white/10 flex items-center gap-2 p-2">
      <button type="button" id="mic-btn" class="p-2 rounded-full hover:bg-white/70 dark:hover:bg-white/10" title="Voice">🎤</button>
      <input id="prompt-input" class="flex-1 px-3 py-2 bg-transparent outline-none" placeholder="Enter mission… e.g., survey grid at 30m, photo on detection, RTL" />
      <button class="px-3 py-2 rounded-lg bg-indigo-600 text-white hover:bg-indigo-700">Send</button>
    </div>
  </div>
</form>

<script>
/* ---------- utils ---------- */
const $ = s => document.querySelector(s);
const get = (o,p,d=undefined)=>p.split('.').reduce((a,k)=>a&&a[k]!=null?a[k]:d,o);
function num(v, d=NaN){ if(typeof v==='number') return v; if(typeof v==='string'){ const m=v.match(/-?\\d+(\\.\\d+)?/); return m?parseFloat(m[0]):d; } return d; }
const fmt = n => (typeof n==='number' && !Number.isNaN(n)) ? n.toFixed(2) : '—';
function toast(msg, kind='ok'){ const t=document.createElement('div'); t.className='toast '+kind; t.textContent=msg; $('#toasts').appendChild(t); setTimeout(()=>t.remove(), 3500); }
function q(url){ return url + (url.includes('?')?'&':'?') + 'ts=' + Date.now(); }
function esc(s){ return (s||'').replace(/[<>&]/g, c=>({'<':'&lt;','>':'&gt;','&':'&amp;'}[c])); }

/* ---------- theme ---------- */
const themeBtn = $('#theme');
function setTheme(mode){ if(mode==='dark'){ document.documentElement.classList.add('dark'); themeBtn.textContent='☀️'; localStorage.setItem('theme','dark'); } else { document.documentElement.classList.remove('dark'); themeBtn.textContent='🌙'; localStorage.setItem('theme','light'); } }
setTheme(localStorage.getItem('theme')||'light');
themeBtn.onclick = ()=> setTheme(document.documentElement.classList.contains('dark')?'light':'dark');

/* ---------- map/video/HUD ---------- */
const v = $('#video'), hud = $('#hud-canvas');
function sizeHUDOnce(){ hud.width = v.clientWidth; hud.height = v.clientHeight; }
let hudSized=false; v.addEventListener('load', ()=>{ if(!hudSized){ sizeHUDOnce(); hudSized=true; } }); window.addEventListener('resize', ()=>{ hudSized=false; sizeHUDOnce(); hudSized=true; });
$('#video-reload').onclick = ()=>{ v.src=''; setTimeout(()=>{ v.src=q('/video.mjpg'); }, 50); }; $('#video-full').onclick = ()=>{ if(v.requestFullscreen) v.requestFullscreen(); };

function drawHUD(roll=0,pitch=0,heading=0,spd=0,alt=null){
  const ctx=hud.getContext('2d'); if(!ctx) return; const w=hud.width, h=hud.height; ctx.clearRect(0,0,w,h);
  const cx=w/2, cy=h/2; ctx.save(); ctx.translate(cx,cy); ctx.rotate(-roll*Math.PI/180); ctx.translate(0, pitch*2);
  ctx.strokeStyle='rgba(255,255,255,.9)'; ctx.lineWidth=2; ctx.beginPath(); ctx.moveTo(-w,0); ctx.lineTo(w,0); ctx.stroke(); ctx.restore();
  ctx.fillStyle='rgba(0,0,0,.65)'; ctx.fillRect(cx-110, 8, 220, 24); ctx.fillStyle='#fff'; ctx.font='12px Inter,system-ui'; ctx.textAlign='center';
  ctx.fillText(`HDG ${Math.round((heading%360+360)%360)}° | SPD ${fmt(spd)} m/s ${alt!=null? '| ALT '+fmt(alt)+' m':''}`, cx, 25);
}

/* ---------- map ---------- */
const map = L.map('map', { zoomControl:false }).setView([47.3967, 8.5497], 19);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution:'© OpenStreetMap contributors' }).addTo(map);
const trail = L.polyline([], { color:'red', weight:3 }).addTo(map);
const detLayer = L.layerGroup().addTo(map);
const arrowIcon = L.divIcon({ className:'uav-icon', html:`<svg viewBox="0 0 40 40" width="40" height="40"><circle cx="20" cy="20" r="18" fill="rgba(239,68,68,.1)" stroke="rgba(0,0,0,0.15)" stroke-width="1"/><g id="needle" transform="rotate(0 20 20)"><polygon points="20,2 30,30 20,24 10,30" fill="red"/></g></svg>`, iconSize:[40,40], iconAnchor:[20,20] });
const marker = L.marker([47.3967, 8.5497], { icon: arrowIcon }).addTo(map);
function rotateMarker(deg){ const el=marker.getElement(); if(!el) return; const g=el.querySelector('#needle'); if(g) g.setAttribute('transform',`rotate(${((deg%360)+360)%360} 20 20)`); }
$('#zoom-in').onclick = ()=> map.zoomIn(); $('#zoom-out').onclick = ()=> map.zoomOut();
$('#recenter').onclick = ()=> { if (trailPts.length) map.setView(trailPts[trailPts.length-1], map.getZoom()); };

/* ---------- panels toggles ---------- */
const toggles = { 'toggle-telemetry':'panel-telemetry', 'toggle-detections':'panel-detections', 'toggle-gallery':'panel-gallery' };
Object.entries(toggles).forEach(([btn,id])=> $('#'+btn).onclick = ()=> $('#'+id).classList.toggle('hidden') );

/* ---------- sensors/scene ---------- */
async function jget(path){ const r = await fetch(q(path), { cache:'no-store' }); if(!r.ok) throw new Error('HTTP '+r.status); return await r.json(); }

function readPos(data){
  const cands = [['position.lat_deg','position.lon_deg'],['gps.position.lat_deg','gps.position.lon_deg'],['gps.lat_deg','gps.lon_deg'],['lat_deg','lon_deg']];
  for(const [pa,po] of cands){ const la=num(get(data,pa)), lo=num(get(data,po)); if(!Number.isNaN(la)&&!Number.isNaN(lo)) return [la,lo]; } return [NaN,NaN];
}
function readHeading(data){ let h=num(get(data,'heading_deg')); if(Number.isNaN(h)) h=num(get(data,'attitude.euler_deg.yaw_deg')); return Number.isNaN(h)?0:h; }
function updateHUDTop(d){
  $('#hud-mode').textContent = 'Mode: '+(get(d,'status.flight_mode','—'));
  const batt=num(get(d,'battery.remaining')); $('#hud-batt').textContent='Batt: ' + (Number.isNaN(batt)?'—':batt) + '%';
  $('#hud-sats').textContent='Sats: '+(get(d,'gps.num_satellites','—'));
  const alt=num(get(d,'position.rel_alt_m'),NaN)||num(get(d,'gps.position.rel_alt_m'),NaN)||num(get(d,'gps.rel_alt_m'),NaN);
  $('#hud-alt').textContent='Alt: '+(Number.isNaN(alt)?'—':fmt(alt))+' m';
  const vn=num(get(d,'velocity_ned.north_m_s'),0), ve=num(get(d,'velocity_ned.east_m_s'),0), vd=num(get(d,'velocity_ned.down_m_s'),0);
  $('#hud-spd').textContent='Speed: '+fmt(Math.hypot(vn,ve,vd))+' m/s';
  $('#hud-ts').textContent=get(d,'timestamp','');
}
function renderTelemetryGrid(d){
  const rows=[['Armed',String(get(d,'status.armed','—'))],['In Air',String(get(d,'status.in_air','—'))],['Fix',get(d,'gps.fix_type','—')],['Heading°',fmt(num(get(d,'heading_deg'))||num(get(d,'attitude.euler_deg.yaw_deg')))],['Lat',fmt(num(get(d,'position.lat_deg'))||num(get(d,'gps.position.lat_deg'))||num(get(d,'gps.lat_deg')))],['Lon',fmt(num(get(d,'position.lon_deg'))||num(get(d,'gps.position.lon_deg'))||num(get(d,'gps.lon_deg')))],['Abs Alt (m)',fmt(num(get(d,'position.abs_alt_m'))||num(get(d,'gps.position.abs_alt_m')))],['Rel Alt (m)',fmt(num(get(d,'position.rel_alt_m'))||num(get(d,'gps.position.rel_alt_m')))],['Voltage (V)',fmt(num(get(d,'battery.voltage_v')))],['Current (A)',fmt(num(get(d,'battery.current_a')))],['RC avail',String(get(d,'rc.available','—'))],['Home OK',String(get(d,'health.home_position_ok','—'))]];
  $('#telemetry-grid').innerHTML = rows.map(([k,v])=>`<div class="p-2 rounded-lg border border-slate-200/70 dark:border-white/10"><div class="text-[11px] text-slate-500 dark:text-slate-400">${k}</div><div class="text-sm font-medium">${v}</div></div>`).join('');
}

/* battery sparkline */
const battCanvas=$('#batt-spark'); const bctx=battCanvas.getContext('2d'); const battPts=[];
function drawBattSpark(){ const w=battCanvas.clientWidth||320; battCanvas.width=w; battCanvas.height=40; bctx.clearRect(0,0,w,40); if(battPts.length<2) return; const min=Math.min(...battPts), max=Math.max(...battPts); const xs=i=>(i/(battPts.length-1))*(w-4)+2; const ys=v=>36-((v-min)/(max-min||1))*30; bctx.strokeStyle='#22c55e'; bctx.lineWidth=2; bctx.beginPath(); battPts.forEach((v,i)=>{ const x=xs(i), y=ys(v); if(i===0) bctx.moveTo(x,y); else bctx.lineTo(x,y); }); bctx.stroke(); }

/* loops */
let trailPts=[];
async function tickSensors(){ try{ const d=await jget('/sensors'); updateHUDTop(d); renderTelemetryGrid(d);
  const [lat,lon]=readPos(d); if(!Number.isNaN(lat)&&!Number.isNaN(lon)){ const here=[lat,lon]; marker.setLatLng(here); rotateMarker(readHeading(d)); trailPts.push(here); if(trailPts.length>50000) trailPts.shift(); trail.setLatLngs(trailPts); if(trailPts.length===1) map.setView(here,19); }
  const roll=num(get(d,'attitude.euler_deg.roll_deg'),0), pitch=num(get(d,'attitude.euler_deg.pitch_deg'),0), head=readHeading(d); const vn=num(get(d,'velocity_ned.north_m_s'),0), ve=num(get(d,'velocity_ned.east_m_s'),0), vd=num(get(d,'velocity_ned.down_m_s'),0); const spd=Math.hypot(vn,ve,vd); const alt=num(get(d,'position.rel_alt_m'),NaN)||num(get(d,'gps.position.rel_alt_m'),NaN)||null; drawHUD(roll,pitch,head,spd,alt);
  const batt=num(get(d,'battery.remaining')); if(!Number.isNaN(batt)){ battPts.push(batt); if(battPts.length>120) battPts.shift(); drawBattSpark(); }
}catch(e){} }
function drawDetectionsOnMap(scene){ detLayer.clearLayers(); let dets=get(scene,'detections',[]); if(!Array.isArray(dets) && typeof dets==='object') dets=Object.values(dets); if(!dets||dets.length===0) return; dets.slice(0,200).forEach(d=>{ const gps=get(d,'estimated_global')||get(d,'GPS'); const la=num(get(gps,'lat')), lo=num(get(gps,'lon')); if(Number.isNaN(la)||Number.isNaN(lo)) return; const name=d['Object Name']||d.class||d.label||'obj'; const conf=num(d['Confidence']||d['Confidence Level']||d.confidence,NaN); const m=L.circleMarker([la,lo],{radius:6,color:'#ef4444',fillColor:'#ef4444',fillOpacity:.75}); m.bindTooltip(`${name}${Number.isNaN(conf)?'':` (${conf.toFixed(2)})`}`); detLayer.addLayer(m); }); }
async function tickScene(){ try{ const s=await jget('/scene'); drawDetectionsOnMap(s); const list=$('#detections-list'); let dets=get(s,'detections',[]); if(!Array.isArray(dets) && typeof dets==='object') dets=Object.values(dets); list.innerHTML=(dets||[]).slice(0,80).map((d,i)=>{ const name=d['Object Name']||d.class||d.label||'object'; const conf=num(d['Confidence']||d['Confidence Level']||d.confidence,NaN); const gps=get(d,'estimated_global')||get(d,'GPS')||{}; const gtxt=(gps&&(gps.lat!=null&&gps.lon!=null))?`lat:${fmt(num(gps.lat))} lon:${fmt(num(gps.lon))}`:''; return `<div class="p-2 border border-slate-200/70 dark:border-white/10 rounded-lg"><div class="flex items-center justify-between"><div class="font-medium">${esc(name)}</div><div class="text-xs text-slate-500">#${i+1}</div></div><div class="text-xs text-slate-600 dark:text-slate-400">conf: ${Number.isNaN(conf)?'—':conf.toFixed(2)}</div>${gtxt? `<div class="text-xs text-slate-600 dark:text-slate-400">${gtxt}</div>`:''}</div>`; }).join('') || '<div class="text-xs text-slate-500">No detections.</div>'; }catch(e){} }
setInterval(tickSensors,1000); setInterval(tickScene,1000);

/* ---------- gallery (local ./images) ---------- */
async function loadGallery(){
  const grid=$('#gallery-grid');
  let files=[];
  try{ const r = await fetch(q('/images/list')); if(r.ok) files = await r.json(); }catch(e){}
  files = Array.isArray(files)? files : [];
  files.sort().reverse(); // newest first if names are timestamped
  grid.innerHTML = files.slice(0,18).map(src=>`<a href="/images/${encodeURIComponent(src)}" target="_blank"><img src="/images/${encodeURIComponent(src)}" class="w-full h-24 object-cover rounded-lg border border-slate-200/70 dark:border-white/10"/></a>`).join('') || `<div class="text-xs text-slate-500">No images.</div>`;
}
$('#refresh-gallery').onclick = ()=> loadGallery();
setInterval(loadGallery, 15000); loadGallery();

/* ---------- Chats: per-mission isolation ---------- */
const chatsList = $('#chats-list');
const missions = new Map();          // id -> {id, prompt, status, transcript, photos[], from, started, summary}
const pollers  = new Map();          // id -> active poller flag
let activeId   = null;

function renderChats(){
  const items = [...missions.values()].sort((a,b)=>b.started-a.started);
  chatsList.innerHTML = items.map(m => {
    const status = m.status || 'running';
    const badge  = status==='running' ? 'bg-yellow-100 text-yellow-700 dark:bg-yellow-900/40 dark:text-yellow-300'
                 : status==='error'   ? 'bg-rose-100 text-rose-700 dark:bg-rose-900/40 dark:text-rose-300'
                 : 'bg-emerald-100 text-emerald-700 dark:bg-emerald-900/40 dark:text-emerald-300';
    const sel    = m.id===activeId ? 'ring-2 ring-indigo-400' : '';
    const short  = esc(m.prompt.length>48 ? m.prompt.slice(0,48)+'…' : m.prompt);
    return `<div class="chat-item p-2 rounded-lg border border-slate-200/70 dark:border-white/10 ${sel}" data-mid="${m.id}">
      <div class="text-sm font-medium">${short}</div>
      <div class="mt-1 flex items-center gap-2 text-xs">
        <span class="badge ${badge}">${status}</span>
        <span class="text-slate-500">photos: ${m.photos.length}</span>
      </div>
    </div>`;
  }).join('') || '<div class="text-xs text-slate-500">No chats yet. Send a mission below.</div>';
  // click handlers
  [...chatsList.querySelectorAll('[data-mid]')].forEach(el=>{
    el.onclick = ()=> setActive(el.getAttribute('data-mid'));
  });
}

function setActive(mid){
  activeId = mid;
  renderChats();
  renderActiveMission();
}

function renderActiveMission(){
  const pre = $('#mission-output'), st=$('#mo-status'), ph=$('#mo-photos'), mt=$('#mo-title');
  const m = missions.get(activeId);
  if(!m){ mt.textContent='—'; st.textContent='—'; ph.textContent='photos: 0'; pre.textContent=''; return; }
  mt.textContent = m.prompt ? (m.prompt.length>64? m.prompt.slice(0,64)+'…' : m.prompt) : '—';
  st.textContent = m.status || 'running';
  ph.textContent = 'photos: ' + m.photos.length;
  let text = m.transcript || '';
  if (m.summary) {
    text += '\nMISSION SUMMARY:\n' + JSON.stringify(m.summary, null, 2);
  }
  pre.textContent = text || '(no output yet)';
}

/* start + poll */
async function startMission(prompt){
  const body = { prompt };
  const res  = await fetch('/mission/start', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body) });
  const data = await res.json();
  if (!res.ok || !data.mission_id) throw new Error(data.error || ('HTTP '+res.status));
  const id = data.mission_id;
  missions.set(id, { id, prompt, status:'running', transcript:'', photos:[], from:0, started: Date.now(), summary:null });
  setActive(id);
  renderChats();
  pollMission(id);
}

async function pollMission(id){
  if (pollers.get(id)) return;
  pollers.set(id, true);
  try{
    while (true) {
      const m = missions.get(id); if(!m) break;
      const r = await fetch(`/mission/${id}/events?from=${m.from}`);
      const d = await r.json();
      if (!r.ok) throw new Error(d.error || ('HTTP '+r.status));
      const evs = d.events || [];
      for (const ev of evs) {
        if (ev.type === 'oplog') { m.transcript += (ev.line + '\n'); }
        else if (ev.type === 'photo') { m.photos.push(ev.file); }
        else if (ev.type === 'status') { m.status = ev.status.status || 'running'; }
        else if (ev.type === 'plain_reply') { m.transcript += (ev.text + '\n'); m.status = 'done'; }
        else if (ev.type === 'summary') { /* just note presence; we'll fetch full summary below */ }
        else if (ev.type === 'error') { m.status = 'error'; }
      }
      m.from = d.to || m.from;
      if (d.status && d.status !== 'running') {
        // fetch summary once
        const sr = await fetch(`/mission/${id}/summary`); const sd = await sr.json();
        if (sd.status === 'error') { m.status = 'error'; }
        else { m.status = sd.status || 'done'; if (sd.summary) m.summary = sd.summary; }
        renderChats(); if (id===activeId) renderActiveMission();
        break;
      }
      // refresh UI lightly
      if (id===activeId) renderActiveMission();
      await new Promise(res => setTimeout(res, 500));
    }
  }catch(e){ const m=missions.get(id); if(m){ m.status='error'; } toast('Agent error: '+e.message,'err'); }
  finally { pollers.delete(id); renderChats(); if (id===activeId) renderActiveMission(); }
}

/* prompt form */
$('#prompt-form').addEventListener('submit', async (e)=>{
  e.preventDefault();
  const btn = e.submitter || e.target.querySelector('button[type=submit]');
  const input = $('#prompt-input');
  const text = (input.value || '').trim();
  if (!text) return;
  const old = btn.textContent; btn.disabled = true; btn.textContent = 'Running…'; input.disabled = true;
  try{ await startMission(text); toast('Mission started','ok'); }
  catch(err){ toast('Start failed: '+err.message,'err'); }
  finally { btn.disabled = false; btn.textContent = old; input.disabled = false; input.value=''; }
});

/* console buttons (active mission only) */
$('#mo-copy').onclick = async ()=>{ const m=missions.get(activeId); const t=(m&&((m.transcript||'')+(m.summary? '\nMISSION SUMMARY:\n'+JSON.stringify(m.summary,null,2):'')))||''; try{ await navigator.clipboard.writeText(t); toast('Transcript copied','ok'); }catch{ toast('Copy failed','err'); } };
$('#mo-clear').onclick = ()=>{ const m=missions.get(activeId); if(m){ m.transcript=''; renderActiveMission(); } };
$('#mo-download').onclick = ()=>{ const m=missions.get(activeId); if(!m) return; const t=(m.transcript||'')+(m.summary? '\nMISSION SUMMARY:\n'+JSON.stringify(m.summary,null,2):''); const blob = new Blob([t], {type:'text/plain'}); const a=document.createElement('a'); a.href=URL.createObjectURL(blob); a.download=`mission_${m.id}.txt`; document.body.appendChild(a); a.click(); a.remove(); };

/* maintenance */
$('#chats-refresh').onclick = ()=>{ renderChats(); renderActiveMission(); };
$('#chats-clear-done').onclick = ()=>{ [...missions.values()].forEach(m=>{ if(m.status!=='running') missions.delete(m.id); }); if(!missions.has(activeId)) activeId=null; renderChats(); renderActiveMission(); };
</script>

</body>
</html>
"""

# -----------------------------
# HTTP Server with proxies
# -----------------------------
class Handler(SimpleHTTPRequestHandler):
    def _send_ok(self, ctype="text/html; charset=utf-8"):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control","no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma","no-cache")
        self.send_header("Expires","0")
        self.end_headers()

    def _proxy(self, target_base, default_ctype="application/json"):
        parsed = urllib.parse.urlparse(self.path)
        qs = f"?{parsed.query}" if parsed.query else ""
        target = f"{target_base}{qs}"
        try:
            with urlopen(Request(target, headers={"User-Agent":"UAVDash/1.0"}), timeout=30) as r:
                body = r.read()
                ctype = r.info().get("Content-Type", default_ctype)
                self._send_ok(ctype)
                self.wfile.write(body)
        except URLError as e:
            msg = json.dumps({"error": str(e), "target": target}).encode()
            self.send_response(502); self.send_header("Content-Type","application/json"); self.end_headers(); self.wfile.write(msg)

    def _proxy_stream(self, target):
        try:
            with urlopen(Request(target, headers={"User-Agent":"UAVDash/1.0"}), timeout=30) as r:
                ctype = r.info().get("Content-Type","multipart/x-mixed-replace;boundary=frame")
                self._send_ok(ctype)
                while True:
                    chunk = r.read(16384)
                    if not chunk: break
                    self.wfile.write(chunk)
        except Exception as e:
            try:
                self.send_response(502); self.send_header("Content-Type","text/plain; charset=utf-8"); self.end_headers()
                self.wfile.write(f"Stream error: {e}".encode())
            except Exception:
                pass

    # ---- Local images
    def _list_images(self) -> List[str]:
        files = []
        if IMAGES_DIR.exists():
            for p in sorted(IMAGES_DIR.iterdir()):
                if p.is_file() and p.suffix.lower() in ALLOWED_EXT:
                    files.append(p.name)
        return files

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send_ok(); self.wfile.write(HTML.encode("utf-8")); return

        # Proxies
        if self.path.startswith("/sensors"):    return self._proxy(SENSORS_URL, "application/json")
        if self.path.startswith("/scene"):      return self._proxy(SCENE_URL,   "application/json")
        if self.path.startswith("/video.mjpg"): return self._proxy_stream(VIDEO_URL)

        # Mission GET passthroughs
        if self.path.startswith("/mission/"):
            parsed = urllib.parse.urlparse(self.path)
            # /mission/<id>/events or /mission/<id>/summary
            return self._proxy(f"{AGENT_BASE}{parsed.path}?{parsed.query}" if parsed.query else f"{AGENT_BASE}{parsed.path}")

        # Local images
        if self.path == "/images/list":
            files = self._list_images()
            self._send_ok("application/json")
            self.wfile.write(json.dumps(files).encode("utf-8"))
            return

        if self.path.startswith("/images/"):
            name = self.path.split("/", 2)[-1]
            if ".." in name or name.startswith("/"):
                self.send_error(400, "bad path"); return
            file_path = IMAGES_DIR / name
            if not file_path.exists() or not file_path.is_file():
                self.send_error(404, "not found"); return
            ctype = mimetypes.guess_type(file_path.name)[0] or "application/octet-stream"
            self._send_ok(ctype)
            self.wfile.write(file_path.read_bytes())
            return

        self.send_response(404); self.end_headers(); self.wfile.write(b"Not found.")

    def _proxy_post_json(self, target_base):
        try:
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length) if length > 0 else b"{}"
            req  = Request(
                target_base,
                data=body,
                headers={
                    "User-Agent": "UAVDash/1.0",
                    "Content-Type": self.headers.get("Content-Type", "application/json"),
                    "Accept": "application/json",
                },
                method="POST",
            )
            with urlopen(req, timeout=60) as r:
                resp_body = r.read()
                ctype = r.info().get("Content-Type", "application/json")
                self._send_ok(ctype)
                self.wfile.write(resp_body)
        except URLError as e:
            msg = json.dumps({"error": str(e), "target": target_base}).encode()
            self.send_response(502); self.send_header("Content-Type","application/json"); self.end_headers(); self.wfile.write(msg)

    def do_POST(self):
        # Start mission
        if self.path == "/mission/start":
            return self._proxy_post_json(f"{AGENT_BASE}/mission/start")
        self.send_response(404); self.end_headers(); self.wfile.write(b"Not found.")

# -----------------------------
# Boot
# -----------------------------
if __name__ == "__main__":
    IMAGES_DIR.mkdir(parents=True, exist_ok=True)
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    url = f"http://127.0.0.1:{PORT}/"
    print(f"✨ UAV Dashboard running at {url}")
    print(f"↪ Proxies: /sensors -> {SENSORS_URL} | /scene -> {SCENE_URL} | /video.mjpg -> {VIDEO_URL}")
    print(f"↪ Images:  /images/* -> local {IMAGES_DIR}")
    print(f"↪ Agent:   POST /mission/start, GET /mission/<id>/events, /mission/<id>/summary -> {AGENT_BASE}")
    try: threading.Thread(target=lambda: webbrowser.open(url), daemon=True).start()
    except Exception: pass
    try: server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down…"); server.server_close(); sys.exit(0)

