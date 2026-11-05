#!/usr/bin/env python3
"""UM982 monitor with a live sky plot at http://127.0.0.1:8080

Run from ~/um982 with the venv active:

    python3 um982_monitor.py --port /dev/cu.usbserial-210
"""

import argparse
import base64
import calendar
import glob
import json
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

BAUDS = [115200, 230400, 460800, 921600, 9600, 38400]
FIX_NAMES = {0: "No fix", 1: "GPS", 2: "DGPS", 4: "RTK FIXED", 5: "RTK float", 6: "Dead reckoning"}
SYS_NAMES = {"GP": "GPS", "GL": "GLONASS", "GA": "Galileo", "GB": "BeiDou", "BD": "BeiDou", "GQ": "QZSS"}


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.gga = None
        self.fix = 0
        self.used = 0
        self.hdop = 0.0
        self.lat = self.lon = self.alt = 0.0
        self.diff_age = None
        self.utc = ""
        self.year = self.month = self.day = 0
        self.gps_unix = None
        self.recv_unix = None
        self.sats = {}
        self.sats_build = {}
        self.hdg = None
        self.hdg_time = 0.0
        self.hdg_status = ""
        self.lines = 0
        self.ntrip = "disabled"
        self.rtcm_bytes = 0
        self.baud = 0

    def note_time(self):
        if len(self.utc) < 6:
            return
        if not self.year:
            t = time.gmtime()
            self.year, self.month, self.day = t.tm_year, t.tm_mon, t.tm_mday
        try:
            hh, mm = int(self.utc[0:2]), int(self.utc[2:4])
            ss = float(self.utc[4:])
            base = calendar.timegm((self.year, self.month, self.day, hh, mm, 0))
        except ValueError:
            return
        self.gps_unix = base + ss
        self.recv_unix = time.time()

    def snapshot(self):
        with self.lock:
            now = time.time()
            hdg = self.hdg if self.hdg and now - self.hdg_time < 3 else None
            sats = [{"sys": s, "prn": p, "elev": e, "az": a, "snr": n}
                    for (s, p), (e, a, n) in self.sats.items()]
            return {
                "fix": self.fix, "fixName": FIX_NAMES.get(self.fix, f"Other ({self.fix})"),
                "used": self.used, "hdop": round(self.hdop, 2),
                "lat": self.lat, "lon": self.lon, "alt": round(self.alt, 2),
                "diffAge": self.diff_age, "utc": self.utc,
                "gpsUnix": self.gps_unix, "recvUnix": self.recv_unix,
                "hdgStatus": self.hdg_status,
                "heading": None if not hdg else {
                    "deg": hdg["heading"], "pitch": hdg["pitch"], "std": hdg["std"],
                    "baseline": hdg["baseline"], "type": hdg["type"]},
                "ntrip": self.ntrip, "rtcm": self.rtcm_bytes, "lines": self.lines,
                "baud": self.baud, "sats": sats,
            }


def nmea_deg(v, hemi):
    if len(v) < 4:
        return 0.0
    raw = float(v)
    deg = int(raw / 100)
    d = deg + (raw - deg * 100) / 60
    return -d if hemi in ("S", "W") else d


def parse_line(st, line):
    st.lines += 1
    body = line.split("*")[0]
    kind = body[3:6] if line.startswith("$") and len(body) >= 6 else ""
    if kind == "GGA":
        f = body.split(",")
        if len(f) < 10:
            return
        st.utc = f[1]
        st.lat = nmea_deg(f[2], f[3]) if f[2] else 0.0
        st.lon = nmea_deg(f[4], f[5]) if f[4] else 0.0
        st.fix = int(f[6] or 0)
        st.used = int(f[7] or 0)
        st.hdop = float(f[8] or 0)
        st.alt = float(f[9] or 0)
        st.diff_age = float(f[13]) if len(f) > 13 and f[13] else None
        st.gga = line
        st.note_time()
        if st.sats_build:
            st.sats, st.sats_build = st.sats_build, {}
    elif kind == "GSV":
        f = body.split(",")
        system = SYS_NAMES.get(body[1:3], body[1:3])
        for i in range(4, len(f) - 3, 4):
            if not f[i]:
                continue
            key = (system, int(f[i]))
            snr = int(f[i + 3]) if f[i + 3] else 0
            elev = int(f[i + 1]) if f[i + 1] else 0
            az = int(f[i + 2]) if f[i + 2] else 0
            prev = st.sats_build.get(key)
            if not prev or snr > prev[2]:
                st.sats_build[key] = (elev, az, snr)
    elif kind == "ZDA":
        f = body.split(",")
        if len(f) >= 5 and f[2] and f[4]:
            st.day, st.month, st.year = int(f[2]), int(f[3]), int(f[4])
            st.note_time()
    elif line.startswith("#UNIHEADINGA") and ";" in body:
        f = body.split(";", 1)[1].split(",")
        if len(f) < 7:
            return
        try:
            baseline = float(f[2])
            heading = float(f[3])
            pitch = float(f[4])
            std = float(f[6])
        except ValueError:
            return
        st.hdg_status = f"{f[0]} / {f[1]}"
        # NONE with a 0 m baseline is "no slave lock", not a real north heading.
        if f[1] in ("NONE", "") or baseline < 0.05:
            st.hdg = None
            return
        st.hdg = {"type": f[1], "baseline": baseline, "heading": heading, "pitch": pitch, "std": std}
        st.hdg_time = time.time()


def find_port():
    candidates = sorted(glob.glob("/dev/cu.usb*") + glob.glob("/dev/cu.wch*") +
                        glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if not candidates:
        sys.exit("No serial port found. Plug in the WitMotion board and check `ls /dev/cu.*`.")
    return candidates[0]


def open_receiver(port, baud):
    print(f"Using port {port}")
    for b in ([baud] if baud else BAUDS):
        ser = serial.Serial(port, b, timeout=0.2)
        try:
            ser.dtr = True
            ser.rts = True
        except OSError:
            pass
        ser.write(b"\r\nVERSIONA\r\nGPGGA 1\r\n")
        start, data = time.time(), b""
        while time.time() - start < 2:
            data += ser.read(512)
            if data.count(b"$") + data.count(b"#") >= 2:
                print(f"UM982 found on {port} at {b} baud")
                return ser, b
        ser.close()
        print(f"No data at {b} baud ({len(data)} bytes: {data[:60]!r})")
    sys.exit("No NMEA from the receiver. Try a different cable, or pass --baud.")


def configure(ser):
    for cmd in ("GPGGA 1", "GPGSV 1", "GPZDA 1", "UNIHEADINGA 1", "CONFIG HEADING FIXLENGTH"):
        ser.write((cmd + "\r\n").encode())
        time.sleep(0.05)


def reader(ser, st, raw_log):
    buf = b""
    while True:
        buf += ser.read(4096)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("ascii", "replace").strip()
            if not line:
                continue
            if raw_log:
                raw_log.write(line + "\n")
            with st.lock:
                parse_line(st, line)


def ntrip_worker(ser, st, host, port, mount, user, password):
    while True:
        st.ntrip = "connecting"
        try:
            s = socket.create_connection((host, port), timeout=10)
            auth = base64.b64encode(f"{user}:{password}".encode()).decode()
            s.sendall(f"GET /{mount} HTTP/1.0\r\nUser-Agent: NTRIP UM982Monitor/1.0\r\n"
                      f"Authorization: Basic {auth}\r\n\r\n".encode())
            header = s.recv(1024)
            first = header.split(b"\r\n")[0].decode(errors="replace")
            if "200" not in first:
                st.ntrip = f"rejected: {first}"
                s.close()
                time.sleep(10)
                continue
            st.ntrip = "streaming"
            leftover = header.split(b"\r\n\r\n", 1)[1] if b"\r\n\r\n" in header else b""
            if leftover:
                ser.write(leftover)
                st.rtcm_bytes += len(leftover)
            s.settimeout(1)
            last_gga = 0.0
            while True:
                if st.gga and time.time() - last_gga > 10:
                    s.sendall((st.gga + "\r\n").encode())
                    last_gga = time.time()
                try:
                    data = s.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    raise ConnectionError("caster closed connection")
                ser.write(data)
                st.rtcm_bytes += len(data)
        except Exception as e:
            st.ntrip = f"error: {e}"
            time.sleep(10)


PAGE = r"""<!doctype html><html lang=en><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>UM982</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
:root{--bg:#070b14;--card:#101826ee;--text:#e7eef8;--dim:#8ea0b8;
 --gps:#3dde8a;--glo:#ff5d6c;--gal:#5aa8ff;--bds:#ffc14d;--qz:#c084fc}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(1200px 700px at 70% -10%,#16324f 0%,var(--bg) 55%);
 color:var(--text);font:15px/1.4 ui-sans-serif,system-ui,sans-serif;min-height:100vh}
header{display:flex;justify-content:space-between;align-items:flex-end;gap:16px;padding:18px 22px 6px}
h1{font-size:13px;letter-spacing:.28em;text-transform:uppercase;color:var(--dim);font-weight:600;margin:0}
.clock{font-variant-numeric:tabular-nums;font-size:34px;font-weight:650;letter-spacing:.04em}
.sub{color:var(--dim);font-size:13px;margin-top:2px}
.wrap{display:flex;flex-direction:column;gap:14px;padding:10px 22px 22px}
.card{background:var(--card);border:1px solid #1e2c40;border-radius:18px;padding:12px;
 backdrop-filter:blur(8px);box-shadow:0 20px 50px #0006}
#view{height:min(52vh,560px);min-height:320px;border-radius:14px;overflow:hidden;background:#070b14;position:relative}
#view canvas{display:block}
.hint{position:absolute;left:12px;bottom:10px;color:#8ea0b8;font-size:12px;pointer-events:none}
.lower{display:grid;grid-template-columns:minmax(280px,1.35fr) minmax(260px,.75fr);gap:14px}
#map{height:340px;border-radius:14px;background:#0b121c}
.leaflet-container{font:inherit;background:#0b121c}
.leaflet-control-attribution{background:#101826cc !important;color:#8ea0b8 !important;font-size:10px !important}
.leaflet-bar a{background:#182436;color:#e7eef8;border-color:#243044}
.me{background:transparent;border:0}
.pulse{width:16px;height:16px;margin:-1px;border-radius:50%;background:#3dde8a;box-shadow:0 0 12px #3dde8a}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:flex-end}
.pill{border-radius:999px;padding:6px 12px;background:#182436;font-weight:650}
.fix0{color:#ff8b8b}.fix1,.fix2{color:#ffd36a}.fix4{color:var(--gps);box-shadow:0 0 18px #3dde8a55}.fix5{color:#8ec5ff}
.stat b{display:block;font-size:18px;font-variant-numeric:tabular-nums}.stat span{color:var(--dim);font-size:11px;letter-spacing:.08em;text-transform:uppercase}
.legend{display:flex;gap:14px;flex-wrap:wrap;color:var(--dim);font-size:12px;margin-top:8px;padding:0 4px}
.dot{width:9px;height:9px;border-radius:50%;display:inline-block;margin-right:6px}
.bars{max-height:220px;overflow:auto;margin-top:8px}
.barline{display:grid;grid-template-columns:78px 36px 1fr 32px;gap:8px;align-items:center;font-size:12px;margin:3px 0}
.track{height:7px;background:#1c2938;border-radius:99px;overflow:hidden}.fill{height:100%;border-radius:99px}
.note{color:var(--dim);font-size:12px;margin-top:8px}
a{color:#8ec5ff}
@media(max-width:860px){.lower{grid-template-columns:1fr}.clock{font-size:26px}#view{height:46vh}}
</style>
<script type="importmap">
{"imports":{
  "three":"https://unpkg.com/three@0.170.0/build/three.module.js",
  "three/addons/":"https://unpkg.com/three@0.170.0/examples/jsm/"
}}
</script>
</head><body>
<header><div><h1>UM982</h1><div class=sub id=where>waiting for a fix…</div></div>
<div style="text-align:right"><div class=clock id=clock>--:--:--</div><div class=sub id=skew>GPS time</div></div></header>
<div class=wrap>
<section class=card>
 <div id=view><div class=hint>drag to look around · scroll to zoom</div></div>
 <div class=legend id=legend></div>
</section>
<div class=lower>
<section class=card>
 <div id=map></div>
 <div class=sub id=mapmeta style="margin-top:8px">map loads with the first fix</div>
</section>
<section>
<div class=card>
 <div class=row><div class="pill fix0" id=fix>No fix</div>
  <div class=stat><b id=used>-</b><span>satellites used</span></div>
  <div class=stat><b id=hdop>-</b><span>HDOP</span></div>
  <div class=stat><b id=alt>-</b><span>altitude m</span></div>
  <div class=stat><b id=hdg>—</b><span>heading</span></div></div>
 <div class=sub id=hdgmeta style="margin-top:8px"></div>
 <div class=sub id=corr style="margin-top:4px"></div>
</div>
<div class=card style="margin-top:14px"><div class=bars id=bars></div><div class=note id=note></div></div>
</section>
</div></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const COL = {GPS:'#3dde8a', GLONASS:'#ff5d6c', Galileo:'#5aa8ff', BeiDou:'#ffc14d', QZSS:'#c084fc'};
const R = 10;
const view = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setClearColor(0x070b14);
view.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x070b14, 28, 70);
const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 200);
camera.position.set(14, 9, 14);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 2, 0);
controls.maxDistance = 28;
controls.minDistance = 4;
controls.maxPolarAngle = Math.PI * 0.49;

function ring(elevDeg, color) {
  const el = elevDeg * Math.PI / 180;
  const y = Math.sin(el) * R, rad = Math.cos(el) * R;
  const pts = [];
  for (let i = 0; i <= 96; i++) {
    const a = i / 96 * Math.PI * 2;
    pts.push(new THREE.Vector3(Math.sin(a) * rad, y, Math.cos(a) * rad));
  }
  return new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({color}));
}
scene.add(ring(0, 0x3a5168), ring(30, 0x2a3c52), ring(60, 0x243044));
for (let i = 0; i < 8; i++) {
  const a = i / 8 * Math.PI * 2;
  const geo = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0.02, 0),
    new THREE.Vector3(Math.sin(a) * R, 0.02, Math.cos(a) * R)]);
  scene.add(new THREE.Line(geo, new THREE.LineBasicMaterial({color: 0x243044})));
}
const ground = new THREE.Mesh(new THREE.CircleGeometry(R, 80), new THREE.MeshBasicMaterial({color: 0x101a28}));
ground.rotation.x = -Math.PI / 2;
scene.add(ground);
const you = new THREE.Mesh(new THREE.ConeGeometry(0.22, 0.7, 18), new THREE.MeshBasicMaterial({color: 0x3dde8a}));
you.position.y = 0.35;
scene.add(you);

function makeLabel(text) {
  const c = document.createElement('canvas');
  c.width = 128; c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = '#9fb0c6';
  g.font = '600 40px sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(text, 64, 32);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({map: new THREE.CanvasTexture(c), transparent: true}));
  s.scale.set(1.8, 0.9, 1);
  return s;
}
const north = makeLabel('N'), east = makeLabel('E'), south = makeLabel('S'), west = makeLabel('W');
north.position.set(0, 0.4, R + 1.1);
south.position.set(0, 0.4, -(R + 1.1));
east.position.set(R + 1.1, 0.4, 0);
west.position.set(-(R + 1.1), 0.4, 0);
scene.add(north, east, south, west);

const starPos = new Float32Array(500 * 3);
for (let i = 0; i < 500; i++) {
  const v = new THREE.Vector3(Math.random() - 0.5, Math.random(), Math.random() - 0.5).normalize();
  v.multiplyScalar(36 + Math.random() * 24);
  starPos.set([v.x, v.y, v.z], i * 3);
}
const stars = new THREE.BufferGeometry();
stars.setAttribute('position', new THREE.BufferAttribute(starPos, 3));
scene.add(new THREE.Points(stars, new THREE.PointsMaterial({color: 0xb7c6d8, size: 0.08})));

const heading = new THREE.Group();
const shaft = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.04, 2.2), new THREE.MeshBasicMaterial({color: 0xe7eef8}));
shaft.position.z = 1.3;
const head = new THREE.Mesh(new THREE.ConeGeometry(0.18, 0.45, 12), new THREE.MeshBasicMaterial({color: 0xe7eef8}));
head.rotation.x = Math.PI / 2;
head.position.z = 2.5;
heading.add(shaft, head);
heading.position.y = 0.06;
heading.visible = false;
scene.add(heading);

const meshes = new Map();
function satVec(az, elev) {
  const a = az * Math.PI / 180, e = elev * Math.PI / 180;
  const h = Math.cos(e) * R;
  return new THREE.Vector3(Math.sin(a) * h, Math.sin(e) * R, Math.cos(a) * h);
}
function syncSats(sats) {
  const seen = new Set();
  for (const s of sats) {
    const key = s.sys + ':' + s.prn;
    seen.add(key);
    let obj = meshes.get(key);
    if (!obj) {
      const color = new THREE.Color(COL[s.sys] || '#fff');
      const group = new THREE.Group();
      group.add(new THREE.Mesh(new THREE.SphereGeometry(0.28, 18, 18), new THREE.MeshBasicMaterial({color})));
      group.add(new THREE.Mesh(new THREE.SphereGeometry(0.55, 14, 14), new THREE.MeshBasicMaterial({color, transparent: true, opacity: 0.28})));
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
        new THREE.LineBasicMaterial({color, transparent: true, opacity: 0.35}));
      scene.add(group, line);
      obj = {group, line};
      meshes.set(key, obj);
    }
    const p = satVec(s.az, s.elev);
    const scale = 0.55 + Math.min(s.snr, 50) / 40;
    obj.group.position.copy(p);
    obj.group.scale.setScalar(scale);
    const attr = obj.line.geometry.attributes.position;
    attr.setXYZ(0, 0, 0.4, 0);
    attr.setXYZ(1, p.x, p.y, p.z);
    attr.needsUpdate = true;
  }
  for (const [k, obj] of meshes) {
    if (!seen.has(k)) { scene.remove(obj.group, obj.line); meshes.delete(k); }
  }
}
function setHeading(h) {
  heading.visible = !!h;
  if (h) heading.rotation.y = -h.deg * Math.PI / 180;
}
function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view);
resize();
let drawing = false;
function draw() {
  if (drawing) return;
  drawing = true;
  controls.update();
  renderer.render(scene, camera);
  drawing = false;
}
controls.addEventListener('change', draw);
draw();

const map = L.map('map', {zoomControl: true});
L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
  attribution: '&copy; OpenStreetMap &copy; CARTO', maxZoom: 20
}).addTo(map);
map.setView([49.13, -122.52], 12);
const marker = L.marker([49.13, -122.52], {
  icon: L.divIcon({className: 'me', html: '<div class="pulse"></div>', iconSize: [16, 16], iconAnchor: [8, 8]})
}).addTo(map);
const trail = L.polyline([], {color: '#3dde8a', weight: 3, opacity: 0.85}).addTo(map);
const trailPts = [];
let placed = false;
function moved(lat, lon) {
  const last = trailPts[trailPts.length - 1];
  if (!last) return true;
  const dy = (lat - last[0]) * 111320;
  const dx = (lon - last[1]) * 111320 * Math.cos(lat * Math.PI / 180);
  return Math.hypot(dx, dy) > 0.4;
}
setTimeout(() => map.invalidateSize(), 250);

let data = null, anchor = null;
function pad(n, w = 2) { return String(Math.floor(Math.abs(n))).padStart(w, '0'); }
function showClock() {
  if (!(anchor && anchor.gps)) return;
  const t = new Date((anchor.gps + (performance.now() - anchor.perf) / 1000) * 1000);
  document.getElementById('clock').textContent =
    pad(t.getUTCHours()) + ':' + pad(t.getUTCMinutes()) + ':' + pad(t.getUTCSeconds()) + '.' + pad(t.getUTCMilliseconds(), 3);
  const skew = (anchor.recv - anchor.gps) * 1000;
  document.getElementById('skew').textContent = 'GPS UTC over USB  ·  this Mac ' + (skew > 0 ? '+' : '') + skew.toFixed(0) + ' ms';
}
function tickClock() {
  showClock();
  draw();
  requestAnimationFrame(tickClock);
}
async function poll() {
  try {
    data = await (await fetch('/api')).json();
    if (data.gpsUnix && (!anchor || data.gpsUnix !== anchor.gps))
      anchor = {gps: data.gpsUnix, recv: data.recvUnix, perf: performance.now()};
    document.getElementById('fix').textContent = data.fixName;
    document.getElementById('fix').className = 'pill fix' + data.fix;
    document.getElementById('used').textContent = data.used + ' / ' + data.sats.length;
    document.getElementById('hdop').textContent = data.hdop.toFixed(2);
    document.getElementById('alt').textContent = data.alt.toFixed(1);
    document.getElementById('where').textContent = data.lat ? data.lat.toFixed(7) + ',  ' + data.lon.toFixed(7) : 'no position yet';
    document.getElementById('corr').textContent = 'corrections ' + (data.diffAge == null ? 'none' : data.diffAge.toFixed(1) + ' s old') + '   ·   ' + data.ntrip;
    if (data.heading) {
      document.getElementById('hdg').textContent = data.heading.deg.toFixed(1) + '°';
      document.getElementById('hdgmeta').textContent = '±' + data.heading.std.toFixed(2) + '°   pitch ' + data.heading.pitch.toFixed(1) + '°   baseline ' + data.heading.baseline.toFixed(3) + ' m   ' + data.heading.type;
    } else {
      document.getElementById('hdg').textContent = '—';
      document.getElementById('hdgmeta').textContent = data.hdgStatus || 'slave antenna not locked';
    }
    setHeading(data.heading);
    syncSats(data.sats);
    showClock();
    draw();
    const counts = {};
    data.sats.forEach(s => counts[s.sys] = (counts[s.sys] || 0) + 1);
    document.getElementById('legend').innerHTML = Object.entries(counts).map(([k, v]) =>
      '<span><i class=dot style="background:' + COL[k] + '"></i>' + k + ' ' + v + '</span>').join('');
    document.getElementById('bars').innerHTML = data.sats.slice().sort((a, b) => b.snr - a.snr).map(s =>
      '<div class=barline><span>' + s.sys + '</span><span>' + s.prn + '</span><div class=track><div class=fill style="width:' +
      Math.min(100, s.snr * 2) + '%;background:' + (COL[s.sys] || '#fff') + '"></div></div><span>' + s.snr + '</span></div>').join('');
    document.getElementById('note').textContent = data.baud + ' baud · ' + data.lines + ' sentences · north is the far side of the dome';
    if (data.lat || data.lon) {
      const ll = [data.lat, data.lon];
      marker.setLatLng(ll);
      if (!placed) { map.setView(ll, 17); placed = true; map.invalidateSize(); }
      if (moved(data.lat, data.lon)) { trailPts.push(ll); trail.setLatLngs(trailPts); }
      const q = data.lat.toFixed(6) + '/' + data.lon.toFixed(6);
      document.getElementById('mapmeta').innerHTML = data.alt.toFixed(1) + ' m  ·  <a target=_blank href="https://www.openstreetmap.org/#map=18/' + q + '">open in OpenStreetMap</a>';
    }
  } catch (e) {}
}
setInterval(poll, 400); poll(); tickClock();
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    state = None

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path.startswith("/api"):
            body = json.dumps(self.state.snapshot()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(PAGE.encode())
        else:
            self.send_error(404)

    def log_message(self, *args):
        pass


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port")
    p.add_argument("--baud", type=int)
    p.add_argument("--web", type=int, default=8080)
    p.add_argument("--ntrip", help="caster as host:port/mountpoint")
    p.add_argument("--user", default="")
    p.add_argument("--password", default="")
    p.add_argument("--log")
    a = p.parse_args()

    ser, baud = open_receiver(a.port or find_port(), a.baud)
    configure(ser)
    st = State()
    st.baud = baud
    raw_log = open(a.log, "a", buffering=1) if a.log else None
    threading.Thread(target=reader, args=(ser, st, raw_log), daemon=True).start()
    if a.ntrip:
        hostport, mount = a.ntrip.split("/", 1)
        host, nport = (hostport.split(":") + ["2101"])[:2]
        threading.Thread(target=ntrip_worker,
                         args=(ser, st, host, int(nport), mount, a.user, a.password), daemon=True).start()

    Handler.state = st
    try:
        httpd = ThreadingHTTPServer(("127.0.0.1", a.web), Handler)
    except OSError:
        sys.exit(f"Port {a.web} is in use. Stop the other monitor, or pass --web 8081.")
    print(f"Sky plot:  http://127.0.0.1:{a.web}")
    print("Leave this running and open that address in a browser. Ctrl-C to quit.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
