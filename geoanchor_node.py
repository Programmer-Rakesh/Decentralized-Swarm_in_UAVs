#!/usr/bin/env python3
"""geoanchor_node.py v2 - DIRECT GeoAnchor for the simulator AND the real drone.

Every --interval seconds (default 2) the newest camera frame is matched, on its own, against the satellite
map and the lat/lon of the point directly UNDER THE DRONE is sent to the GCS by UDP.  No particle filter, no
odometry, no memory: a fix cannot lag or drift, and if the drone "hops" the next fix lands where it really is.

MAP   --map <file.png>   a map image + <map>.bounds.txt                      (simulator world)
      --map auto         satellite tiles around --home (lat,lon), --radius-m, --zoom.  The tiles are
                         downloaded ONCE (needs internet), stitched and cached in ~/.geoanchor; every later run
                         is fully OFFLINE.  Georeferencing is exact (Web-Mercator tile maths): no calibration.
CAMERA  default          Gazebo camera topic (simulator)
        --video SRC      real camera: device number (0), /dev/videoN, rtsp://... , or a video file
STATE   real flight only: the GCS forwards the Cube's roll / pitch / heading / relative altitude (telemetry)
        to UDP --state-port; they give the expected scale + rotation and the tilt correction.

REPLAY  --video FILE --replay   (used by the GCS "FLIGHT LOG" card) the video position is not free-running: it follows the
        replay clock the GCS puts into every ga_state datagram ("vt" = seconds into the video), so video, recorded
        telemetry and map fixes stay in sync.  Preview frames go back to the GCS as  b"GAFR" + jpeg  datagrams.

Prepare the map at home WITH internet, then fly offline:
    python3 geoanchor_node.py --prepare --map auto --home 28.3659,77.5448 --radius-m 500

UDP datagrams to 127.0.0.1:--port (same as before, gcs_odo.py understands them):
  {"k":"ga_status", ...}   {"k":"ga_fix","t","lat","lon","inl","mode","ms","n","scale","hdg","first"}
"""
import argparse
import concurrent.futures as cf
import json
import math
import os
import socket
import sys
import threading
import time
import traceback
import urllib.request
from pathlib import Path

import cv2
import numpy as np

M_PER_DEG = 111320.0
GA_HOME = Path(os.environ.get("GEOANCHOR_HOME", Path.home() / ".geoanchor"))
TILE_URL = "https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}"     # same imagery as the GCS map
FEATURE_VERSION = "v2"                                                    # bump when the feature code changes


# ----------------------------------------------------------------------------------------------------
class Out:
    """UDP sender towards the GCS."""

    def __init__(self, port):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = ("127.0.0.1", port)

    def send(self, d):
        try:
            self.s.sendto(json.dumps(d).encode(), self.addr)
        except OSError:
            pass

    def status(self, state, text):
        self.send({"k": "ga_status", "state": state, "text": text, "t": time.time()})

    def send_raw(self, b):
        try:
            self.s.sendto(b, self.addr)
        except OSError:
            pass


# ---------------------------------------------------------------- Web-Mercator tiles ---------------
def deg2num(lat, lon, z):
    n = 2.0 ** z
    return ((lon + 180.0) / 360.0 * n,
            (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)


def num2deg(x, y, z):
    n = 2.0 ** z
    return (math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n)))), x / n * 360.0 - 180.0)


def _get_tile(url_tpl, cache_dir, z, x, y, retries=3):
    p = Path(cache_dir) / str(z) / str(x) / f"{y}.jpg"
    if p.exists() and p.stat().st_size > 200:
        return True
    url = url_tpl.format(x=x, y=y, z=z)
    for k in range(retries):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0 (GeoAnchor tile cache)"})
            with urllib.request.urlopen(req, timeout=10) as r:
                data = r.read()
            if cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR) is None:
                raise ValueError("not an image")
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(data)
            return True
        except Exception:
            time.sleep(0.4 * (k + 1))
    return False


MAX_MOSAIC_PX = 6144
MIN_ZOOM = 17


def plan_tiles(south, west, north, east, zoom):
    """Tile range covering the box.  Lowers the zoom if the mosaic would get too big.  -> (zoom, tx0, ty0, nx, ny)"""
    while True:
        xa, ya = deg2num(north, west, zoom)
        xb, yb = deg2num(south, east, zoom)
        tx0, ty0, tx1, ty1 = int(xa), int(ya), int(xb), int(yb)
        nx, ny = tx1 - tx0 + 1, ty1 - ty0 + 1
        if max(nx, ny) * 256 <= MAX_MOSAIC_PX or zoom <= MIN_ZOOM:
            break
        zoom -= 1
    if max(nx, ny) * 256 > MAX_MOSAIC_PX:
        km = max(nx, ny) * 256 * 156543.03392 * math.cos(math.radians((south + north) / 2)) / 2 ** zoom / 1000.0
        raise RuntimeError(f"area is about {km:.1f} km wide - too large for one map. Mark the points closer together.")
    return zoom, tx0, ty0, nx, ny


def bbox_around(points, margin_m):
    """(south, west, north, east) of all points plus a margin in metres on every side."""
    lats, lons = [p[0] for p in points], [p[1] for p in points]
    lat_mid = 0.5 * (min(lats) + max(lats))
    dlat = margin_m / M_PER_DEG
    dlon = margin_m / (M_PER_DEG * math.cos(math.radians(lat_mid)))
    return min(lats) - dlat, min(lons) - dlon, max(lats) + dlat, max(lons) + dlon


def parse_points(txt):
    pts = []
    for part in txt.split(";"):
        part = part.strip()
        if part:
            la, lo = [float(v) for v in part.split(",")[:2]]
            pts.append((la, lo))
    return pts


def build_mosaic(lat, lon, radius_m, zoom, url_tpl, out):
    """Mosaic of a square of +-radius_m around one point."""
    s_, w_, n_, e_ = bbox_around([(lat, lon)], radius_m)
    return build_mosaic_bbox(s_, w_, n_, e_, zoom, url_tpl, out)


def build_mosaic_bbox(south, west, north, east, zoom, url_tpl, out):
    """Satellite mosaic covering a box.  Returns (bgr image, merc=(z, tx0, ty0, nx, ny), name)."""
    z0 = zoom
    zoom, tx0, ty0, nx, ny = plan_tiles(south, west, north, east, zoom)
    tx1, ty1 = tx0 + nx - 1, ty0 + ny - 1
    if zoom != z0:
        print(f"map: area is big - using zoom {zoom} instead of {z0} (coarser map)", flush=True)
        out.status("loading", f"area is big - using zoom {zoom} (coarser map)")
    name = f"z{zoom}_{tx0}_{ty0}_{nx}x{ny}"
    mosaic_png = GA_HOME / "mosaics" / f"mosaic_{name}.png"
    if mosaic_png.exists():
        img = cv2.imread(str(mosaic_png), cv2.IMREAD_COLOR)
        if img is not None and img.shape[1] == nx * 256 and img.shape[0] == ny * 256:
            print(f"map: cached mosaic {name}  ({nx}x{ny} tiles)", flush=True)
            return img, (zoom, tx0, ty0, nx, ny), name
    tiles = [(x, y) for y in range(ty0, ty1 + 1) for x in range(tx0, tx1 + 1)]
    cache_dir = GA_HOME / "tiles"
    todo = [t for t in tiles if not (cache_dir / str(zoom) / str(t[0]) / f"{t[1]}.jpg").exists()]
    if todo:
        print(f"map: downloading {len(todo)} of {len(tiles)} tiles (needs internet) ...", flush=True)
    done = 0
    with cf.ThreadPoolExecutor(8) as ex:
        futs = {ex.submit(_get_tile, url_tpl, cache_dir, zoom, x, y): (x, y) for (x, y) in todo}
        for f in cf.as_completed(futs):
            done += 1
            if done % 10 == 0 or done == len(todo):
                out.status("loading", f"downloading map tiles {done}/{len(todo)}")
    img = np.full((ny * 256, nx * 256, 3), 127, np.uint8)
    missing = 0
    for (x, y) in tiles:
        p = cache_dir / str(zoom) / str(x) / f"{y}.jpg"
        t = cv2.imread(str(p), cv2.IMREAD_COLOR) if p.exists() else None
        if t is None:
            missing += 1
            continue
        if t.shape[:2] != (256, 256):
            t = cv2.resize(t, (256, 256), interpolation=cv2.INTER_AREA)
        img[(y - ty0) * 256:(y - ty0 + 1) * 256, (x - tx0) * 256:(x - tx0 + 1) * 256] = t
    if missing > max(2, 0.02 * len(tiles)):
        raise RuntimeError(f"{missing}/{len(tiles)} map tiles missing - no internet and not cached yet. "
                           f"Prepare the map at home WITH internet (switch GeoAnchor on once, or run --prepare).")
    mosaic_png.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(mosaic_png), img)
    print(f"map: mosaic {name} built ({img.shape[1]}x{img.shape[0]} px, {missing} tiles missing)", flush=True)
    return img, (zoom, tx0, ty0, nx, ny), name


# ----------------------------------------------------------------------------------------------------
def _prep_gray(img):
    g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(g)      # evens out light / season / haze


def _rootsift(d):
    if d is None:
        return None
    d = d.astype(np.float32)
    d /= (np.abs(d).sum(axis=1, keepdims=True) + 1e-7)
    return np.sqrt(d)


class MapMatcher:
    """Locates a camera frame on the satellite map.  Pure image matching, no state."""

    MAX_MAP_PX = MAX_MOSAIC_PX

    def __init__(self, gray, bounds, cache_file=None, out=None, merc=None, color=None):
        self.merc = merc
        if color is None:
            color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if max(gray.shape) > self.MAX_MAP_PX:
            f = self.MAX_MAP_PX / max(gray.shape)
            gray = cv2.resize(gray, None, fx=f, fy=f, interpolation=cv2.INTER_AREA)
            color = cv2.resize(color, (gray.shape[1], gray.shape[0]), interpolation=cv2.INTER_AREA)
        self.color = color
        self.H, self.W = gray.shape
        self.west, self.south, self.east, self.north = bounds
        lat_mid = 0.5 * (self.south + self.north)
        if merc:
            z, _, _, nx, _ = merc
            self.gsd = 156543.03392 * math.cos(math.radians(lat_mid)) / 2 ** z * (nx * 256 / self.W)
        else:
            mx = (self.east - self.west) * M_PER_DEG * math.cos(math.radians(lat_mid)) / self.W
            my = (self.north - self.south) * M_PER_DEG / self.H
            self.gsd = 0.5 * (mx + my)
        self.sift = cv2.SIFT_create(nfeatures=4000, contrastThreshold=0.02)
        cache = Path(cache_file) if cache_file else None
        if cache is not None and cache.exists():
            z_ = np.load(cache)
            self.pts, self.desc = z_["pts"], z_["desc"]
        else:
            if out:
                out.status("loading", "computing map features (one-off, cached afterwards) ...")
            big = cv2.SIFT_create(nfeatures=int(min(200000, max(60000, gray.size / 150))), contrastThreshold=0.02)
            kp, desc = big.detectAndCompute(_prep_gray(gray), None)
            self.pts = np.array([k.pt for k in kp], np.float32)
            self.desc = _rootsift(desc)
            if cache is not None:
                cache.parent.mkdir(parents=True, exist_ok=True)
                np.savez(cache, pts=self.pts, desc=self.desc)
        self.flann = cv2.FlannBasedMatcher(dict(algorithm=1, trees=4), dict(checks=64))
        self.flann.add([self.desc])
        self.flann.train()

    # -- pixel -> lat/lon --------------------------------------------------------------------------
    def px_to_latlon(self, px, py):
        if self.merc:
            z, tx0, ty0, nx, ny = self.merc
            return num2deg(tx0 + px / self.W * nx, ty0 + py / self.H * ny, z)
        return (self.north - (py / self.H) * (self.north - self.south),
                self.west + (px / self.W) * (self.east - self.west))

    # -- one frame ---------------------------------------------------------------------------------
    def locate(self, gray, expect_scale, min_inl, scale_tol=0.30, rot_expect=None, rot_tol=25.0):
        """gray: camera frame (uint8).  expect_scale: expected map-px per frame-px (None = accept any).
        rot_expect: expected compass bearing of the frame's up direction (None = accept any)."""
        kp, desc = self.sift.detectAndCompute(_prep_gray(gray), None)
        if desc is None or len(kp) < 60:
            return None, "too little texture / blurry"
        src_pts = np.float32([k.pt for k in kp])
        desc = _rootsift(desc)
        matches = self.flann.knnMatch(desc, k=2)
        src, dst = [], []
        for mm in matches:
            if len(mm) == 2 and mm[0].distance < 0.85 * mm[1].distance:
                src.append(src_pts[mm[0].queryIdx])
                dst.append(self.pts[mm[0].trainIdx])
        if len(src) < 10:
            return None, f"only {len(src)} feature matches"
        src, dst = np.float32(src), np.float32(dst)
        M, inl = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC, ransacReprojThreshold=4.0,
                                             maxIters=5000, confidence=0.995)
        if M is None:
            return None, "no consistent match"
        n_in = int(inl.sum())
        scale = math.hypot(M[0, 0], M[1, 0])                                    # map px per frame px
        vx, vy = M[:, :2] @ np.array([0.0, -1.0])                               # where frame-up points on the map
        hdg = math.degrees(math.atan2(vx, -vy)) % 360.0                         # compass bearing of frame-up
        info = f"{n_in} inliers"
        if expect_scale:
            info += f", scale x{scale / expect_scale:.2f} of expected"
        if rot_expect is not None:
            info += f", heading {hdg:.0f} vs expected {rot_expect:.0f}"
        if n_in < min_inl:
            return None, f"weak match ({info})"
        if expect_scale and abs(scale / expect_scale - 1.0) > scale_tol:
            return None, f"scale mismatch ({info})"
        if rot_expect is not None and abs(((hdg - rot_expect + 180.0) % 360.0) - 180.0) > rot_tol:
            return None, f"rotation mismatch ({info})"
        h, w = gray.shape
        cx, cy = M @ np.array([w / 2.0, h / 2.0, 1.0])
        if not (0 <= cx < self.W and 0 <= cy < self.H):
            return None, "match falls outside the map"
        return {"px": float(cx), "py": float(cy), "inl": n_in, "scale": scale, "hdg": hdg,
                "n_match": len(src), "mode": "direct match", "M": M}, ""


def save_proof(run_dir, nfix, rgb, mm, r, lat, lon, ms, alt):
    """One small jpg per fix: LEFT = live camera frame, RIGHT = the satellite map seen through the same
    window (the match transform applied in reverse).  If the fix is right the two halves show the same scene."""
    h, w = rgb.shape[:2]
    left = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    right = cv2.warpAffine(mm.color, r["M"], (w, h), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    for im in (left, right):
        cv2.drawMarker(im, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 22, 3)
        cv2.drawMarker(im, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 22, 1)
    tw = 560
    th = int(h * tw / w)
    both = np.hstack([cv2.resize(left, (tw, th), interpolation=cv2.INTER_AREA),
                      cv2.resize(right, (tw, th), interpolation=cv2.INTER_AREA)])
    bar = np.full((30, both.shape[1], 3), 25, np.uint8)
    cv2.putText(bar, f"fix {nfix}   {time.strftime('%H:%M:%S')}   {lat:.6f}, {lon:.6f}   {r['inl']} inliers   alt {alt:.0f} m   {ms:.0f} ms",
                (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(bar, "", (0, 0), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    both = np.vstack([bar, both])
    cv2.putText(both, "LIVE CAMERA", (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(both, "MATCHED MAP", (tw + 8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
    name = f"fix_{nfix:04d}_{time.strftime('%H%M%S')}_inl{r['inl']}.jpg"
    cv2.imwrite(str(Path(run_dir) / name), both, [cv2.IMWRITE_JPEG_QUALITY, 72])


def nadir_correction(alt, roll_deg, pitch_deg, heading_deg):
    """The optical axis does not point straight down when the drone is tilted: the ground point at the frame
    centre is off the drone's nadir by alt*tan(tilt).  Returns (north_m, east_m) of that offset; subtract it
    from the matched centre.  Nose up (+pitch) -> centre is AHEAD of the drone; right wing down (+roll) ->
    centre is to the LEFT (aerospace convention, camera fixed to the airframe pointing down)."""
    f = alt * math.tan(math.radians(pitch_deg))
    r = -alt * math.tan(math.radians(roll_deg))
    psi = math.radians(heading_deg)
    return f * math.cos(psi) - r * math.sin(psi), f * math.sin(psi) + r * math.cos(psi)


# ---------------------------------------------------------------- inputs ----------------------------
class Cam:
    """Gazebo camera (simulator)."""

    def __init__(self, node, topic, Image):
        self.lock, self.n, self.rgb, self.t, self.err = threading.Lock(), 0, None, 0.0, None
        if not node.subscribe(Image, topic, self._cb):
            sys.exit(f"cannot subscribe to {topic}")

    def _cb(self, msg):
        if len(msg.data) != msg.width * msg.height * 3:
            return
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
        with self.lock:
            self.rgb, self.n, self.t = a, self.n + 1, time.time()

    def get(self):
        with self.lock:
            return self.n, self.rgb, self.t


class VideoCam(threading.Thread):
    """Real camera: USB / RTSP / file.  Keeps only the newest frame, stamped at grab time."""

    def __init__(self, src, latency_s=0.15, undistort=None):
        super().__init__(daemon=True)
        self.src = int(src) if str(src).isdigit() else src
        self.latency, self.und = latency_s, undistort
        self.lock, self.n, self.rgb, self.t, self.err = threading.Lock(), 0, None, 0.0, None
        self.is_file = isinstance(self.src, str) and os.path.isfile(self.src)
        self.cap = cv2.VideoCapture(self.src)
        if not self.cap.isOpened():
            sys.exit(f"cannot open camera source {src}")
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 25.0
        self.start()

    def run(self):
        while True:
            t0 = time.time()
            ok, bgr = self.cap.read()
            if not ok:
                if self.is_file:
                    self.err = "end of video file"
                    return
                time.sleep(0.05)
                continue
            if self.und is not None:
                bgr = cv2.undistort(bgr, self.und[0], self.und[1])
            with self.lock:
                self.rgb, self.n, self.t = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), self.n + 1, time.time() - self.latency
            if self.is_file:
                time.sleep(max(0.0, 1.0 / self.fps - (time.time() - t0)))

    def get(self):
        with self.lock:
            return self.n, self.rgb, self.t


class ReplayCam(threading.Thread):
    """Recorded flight video, slaved to the GCS replay clock (see the REPLAY note at the top).

    Same get() interface as VideoCam.  The frame shown is always the one at  vt * fps  where vt is the newest
    'vt' received in a ga_state datagram; frames are skipped (never queued) when the decoder is slower than the
    clock.  Every new frame is also sent to the GCS as a small JPEG so the GCS panel can display it."""

    def __init__(self, src, state, out, max_w=1280, undistort=None):
        super().__init__(daemon=True)
        self.state, self.out, self.max_w, self.und = state, out, max_w, undistort
        self.lock, self.n, self.rgb, self.t, self.err = threading.Lock(), 0, None, 0.0, None
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            sys.exit(f"cannot open video {src}")
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.fps = fps if 1.0 <= fps <= 240.0 else 30.0
        nf = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        self.nframes = nf if nf > 0 else 0
        self.pos, self.idx, self._pv_t = 0, -1, 0.0                # next frame cap.read() gives / frame shown now
        self.w = self.h = 0
        if not self._show(0):
            sys.exit("cannot read the first frame of the video")
        self.start()

    def _show(self, idx):
        """Make frame number idx the current one.  True if a new frame was produced."""
        if self.nframes:
            idx = min(idx, self.nframes - 1)
        idx = max(0, idx)
        if idx == self.idx:
            return False
        if self.pos <= idx <= self.pos + 8:                       # normal playback: step forward
            while self.pos < idx:
                if not self.cap.grab():
                    break
                self.pos += 1
        elif idx != self.pos:                                      # jump (start, offset, rewind)
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            self.pos = idx
        ok, bgr = self.cap.read()
        if not ok:                                                 # the file is shorter than its frame count says
            self.nframes = max(1, idx)
            return False
        self.pos, self.idx = idx + 1, idx
        if self.und is not None:
            bgr = cv2.undistort(bgr, self.und[0], self.und[1])
        self.h, self.w = bgr.shape[:2]
        if self.w > self.max_w:
            bgr = cv2.resize(bgr, (self.max_w, int(self.h * self.max_w / self.w)), interpolation=cv2.INTER_AREA)
        with self.lock:
            self.rgb, self.n, self.t = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), self.n + 1, time.time()
        self._preview(bgr)
        return True

    def _preview(self, bgr):
        if time.time() - self._pv_t < 0.045:                       # <= ~20 preview frames per second
            return
        self._pv_t = time.time()
        h, w = bgr.shape[:2]
        small = cv2.resize(bgr, (480, max(2, int(h * 480 / w))), interpolation=cv2.INTER_AREA) if w > 480 else bgr
        for q in (62, 40, 25):
            ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, q])
            if ok and len(buf) < 60000:
                self.out.send_raw(b"GAFR" + buf.tobytes())
                return

    def _ready(self):
        dur = (self.nframes / self.fps) if self.nframes else None
        self.out.send({"k": "ga_replay", "state": "ready", "fps": self.fps, "frames": self.nframes, "dur": dur,
                       "w": self.w, "h": self.h})

    def run(self):
        last_ready, got_vt = 0.0, False
        while True:
            with self.state.lock:
                d = self.state.d
            vt = d.get("vt") if isinstance(d, dict) else None
            if vt is None:
                if time.time() - last_ready > 1.0:                 # tell the GCS we are ready (until the clock starts)
                    last_ready = time.time()
                    self._ready()
                time.sleep(0.05)
                continue
            got_vt = True
            try:
                self._show(int(round(float(vt) * self.fps)))
            except Exception:
                traceback.print_exc()
                time.sleep(0.2)
            time.sleep(0.02)

    def get(self):
        with self.lock:
            return self.n, self.rgb, self.t


class StateFeed(threading.Thread):
    """roll / pitch / heading / relative altitude forwarded by the GCS from the Cube (UDP json 'ga_state')."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(("127.0.0.1", port))
        self.lock, self.d, self.t = threading.Lock(), None, 0.0
        self.start()

    def run(self):
        while True:
            try:
                m = json.loads(self.s.recvfrom(4096)[0].decode())
            except Exception:
                continue
            if isinstance(m, dict) and m.get("k") == "ga_state":
                with self.lock:
                    self.d, self.t = m, time.time()

    def get(self, max_age=2.0):
        with self.lock:
            d, t = self.d, self.t
        if d is None or time.time() - t > max_age or d.get("alt") is None:
            return None
        return d


class PoseFeed:
    """Simulator altitude (barometer stand-in) from Gazebo.  Only z is used - never x/y."""

    def __init__(self, node, world, model, Pose_V):
        self.lock, self.p, self.model = threading.Lock(), None, model
        try:
            ok = node.subscribe(Pose_V, f"/world/{world}/pose/info", self._cb)
        except Exception:
            ok = False
        if not ok:
            print("warning: no pose feed - using --alt as altitude", flush=True)

    def _cb(self, msg):
        for p in msg.pose:
            if p.name == self.model:
                with self.lock:
                    self.p = (p.position.x, p.position.y, p.position.z)

    def get(self):
        with self.lock:
            return self.p


def pick_camera_topic(node, wanted):
    try:
        topics = list(node.topic_list())
    except Exception:
        return wanted
    if wanted in topics:
        return wanted
    cands = [t for t in topics if "camera" in t.lower() and "info" not in t.lower() and "depth" not in t.lower()]
    cands.sort(key=lambda t: (("image" not in t.lower()), len(t)))
    if cands:
        print(f"topic {wanted} not found - using {cands[0]}  (all camera topics: {cands})", flush=True)
        return cands[0]
    print(f"topic {wanted} not found and no camera topic seen: {topics}", flush=True)
    return wanted


def parse_bounds(txt):
    try:
        v = [float(x) for x in txt.replace(" ", "").split(",")]
        if len(v) != 4:
            raise ValueError
    except ValueError:
        raise ValueError(f"bad bounds '{txt[:60]}' - need 4 numbers: west,south,east,north")
    w, s, e, n = v
    if not (-180 <= w < e <= 180 and -90 <= s < n <= 90):
        raise ValueError(f"bounds out of range / wrong order: {v}")
    return v


def load_gray(path):
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        from PIL import Image as PILImage
        PILImage.MAX_IMAGE_PIXELS = None
        img = np.array(PILImage.open(path).convert("L"))
    return img


def get_home(args, src_map):
    if args.home:
        la, lo = [float(v) for v in args.home.replace(" ", "").split(",")]
        return la, lo
    if src_map is not None:
        hp = src_map.with_suffix(".home.json")
        if hp.exists():
            h = json.loads(hp.read_text())
            return float(h["lat"]), float(h["lon"])
    return None


# ----------------------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True, help="map image (png/jpg + .bounds.txt) or 'auto' (tiles around --home)")
    ap.add_argument("--bounds", default="", help="west,south,east,north (or <map>.bounds.txt)")
    ap.add_argument("--home", default="", help="lat,lon of the home position (needed for --map auto)")
    ap.add_argument("--radius-m", type=float, default=500.0, help="auto map without --points: half-size of the area around home")
    ap.add_argument("--points", default="", help="auto map: 'lat,lon;lat,lon;...' = home + every target; the map covers all of them")
    ap.add_argument("--margin-m", type=float, default=150.0, help="auto map with --points: extra area around the points, m")
    ap.add_argument("--zoom", type=int, default=19, help="auto map: tile zoom (19 = 0.26 m/px at 28N)")
    ap.add_argument("--tiles", default=TILE_URL, help="tile URL template with {x} {y} {z}")
    ap.add_argument("--prepare", action="store_true", help="only download + cache the map, then exit")
    ap.add_argument("--video", default="", help="REAL camera: 0 / /dev/video0 / rtsp://... / file.mp4 (empty = Gazebo)")
    ap.add_argument("--replay", action="store_true",
                    help="--video is a RECORDED flight: its position follows the GCS replay clock ('vt' in ga_state)")
    ap.add_argument("--replay-max-w", type=int, default=1280, help="replay: frames wider than this are scaled down for matching")
    ap.add_argument("--state-port", type=int, default=15901, help="UDP port the GCS forwards attitude/altitude to")
    ap.add_argument("--calib", default="", help="camera calibration .npz with K and dist (undistorts the real video)")
    ap.add_argument("--cam-latency-s", type=float, default=0.15, help="real camera: delay between exposure and grab")
    ap.add_argument("--cam-yaw", type=float, default=0.0, help="real camera: bearing of image-up relative to the nose, deg")
    ap.add_argument("--no-tilt-comp", action="store_true", help="real: do not correct the nadir point for roll/pitch")
    ap.add_argument("--rot-tol", type=float, default=25.0, help="real: allowed heading disagreement, deg")
    ap.add_argument("--alt", type=float, default=150.0, help="cruise altitude, m")
    ap.add_argument("--heading", type=float, default=90.0, help="simulator: bearing of the camera's up direction")
    ap.add_argument("--hfov", type=float, default=1.047, help="camera horizontal field of view, rad")
    ap.add_argument("--interval", type=float, default=2.0, help="seconds between fixes (start to start)")
    ap.add_argument("--topic", default="/drone/camera/image")
    ap.add_argument("--world", default="photo_world")
    ap.add_argument("--model", default="iris")
    ap.add_argument("--min-inliers", type=int, default=0, help="0 = 25 (simulator) / 20 (real)")
    ap.add_argument("--min-alt-m", type=float, default=0.0, help="start fixing from this altitude, m (0 = 100 simulator / half of --alt real)")
    ap.add_argument("--min-alt-frac", type=float, default=0.0, help="alternative to --min-alt-m: fraction of --alt")
    ap.add_argument("--log-dir", default="", help="proof images folder (default: 'flight visual logs' next to this file)")
    ap.add_argument("--no-visual-log", action="store_true", help="do not save the live-vs-map proof images")
    ap.add_argument("--log-max", type=int, default=3000, help="max proof images per run")
    ap.add_argument("--port", type=int, default=15900, help="GCS UDP port")
    # accepted for compatibility with older GCS versions, ignored
    for a in ("--search-m", "--particles", "--world-scale", "--window", "--gate-m"):
        ap.add_argument(a, type=float, default=0.0)
    args = ap.parse_args()

    out = Out(args.port)
    out.status("loading", "starting GeoAnchor (direct matcher) ...")
    try:
        run(args, out)
    except SystemExit:
        raise
    except Exception as e:
        traceback.print_exc()
        out.status("error", f"{type(e).__name__}: {e}")
        raise


def run(args, out):
    real = bool(args.video)
    if args.replay and not args.video:
        out.status("error", "--replay needs --video <recorded flight video>")
        sys.exit("--replay needs --video")
    home = None
    if args.map.lower() == "auto":
        pts = []
        try:
            pts = parse_points(args.points) if args.points else []
            home = get_home(args, None) or (pts[0] if pts else None)
        except Exception:
            home = None
        if home is None:
            out.status("error", "map 'auto' needs the HOME position (set it in VISUAL ODO) - none given")
            sys.exit("--map auto needs --home")
        if pts:
            s_, w_, n_, e_ = bbox_around(pts + [home], args.margin_m)
            print(f"map area: {len(pts)} points + {args.margin_m:.0f} m margin", flush=True)
            bgr, merc, name = build_mosaic_bbox(s_, w_, n_, e_, args.zoom, args.tiles, out)
        else:
            bgr, merc, name = build_mosaic(home[0], home[1], args.radius_m, args.zoom, args.tiles, out)
        nw, se = num2deg(merc[1], merc[2], merc[0]), num2deg(merc[1] + merc[3], merc[2] + merc[4], merc[0])
        bounds = [nw[1], se[0], se[1], nw[0]]
        mm = MapMatcher(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), bounds,
                        GA_HOME / "features" / f"sift_{FEATURE_VERSION}_{name}.npz", out, merc=merc, color=bgr)
    else:
        src_map = Path(args.map).expanduser().resolve()
        if not src_map.exists():
            out.status("error", f"map not found: {src_map}")
            sys.exit(f"map not found: {src_map}")
        if src_map.suffix.lower() in (".tif", ".tiff"):
            out.status("error", "give the .png / .jpg map (with <map>.bounds.txt), not a GeoTIFF")
            sys.exit("need a png/jpg map")
        bpath = src_map.with_suffix(".bounds.txt")
        txt = args.bounds or (bpath.read_text().strip() if bpath.exists() else "")
        if not txt:
            out.status("error", "need bounds: west,south,east,north (or <map>.bounds.txt next to the map)")
            sys.exit("need --bounds")
        try:
            bounds = parse_bounds(txt)
            home = get_home(args, src_map)
        except Exception as e:
            out.status("error", str(e))
            sys.exit(str(e))
        g = load_gray(src_map)
        col = cv2.imread(str(src_map), cv2.IMREAD_COLOR)
        if col is None or col.shape[:2] != g.shape:
            col = None
        mm = MapMatcher(g, bounds, src_map.parent / "ga_cache" /
                        f"sift_{FEATURE_VERSION}_{src_map.stem}_{g.shape[1]}x{g.shape[0]}.npz", out, color=col)
    print(f"map {mm.W}x{mm.H} px, {mm.gsd:.3f} m/px, {len(mm.pts)} features", flush=True)
    if args.prepare:
        out.status("waiting", "map ready (prepare mode)")
        print("map prepared - you can fly offline now.", flush=True)
        return

    min_inl = args.min_inliers or (20 if real else 25)
    min_alt = args.min_alt_m or (args.min_alt_frac * args.alt if args.min_alt_frac else (0.5 * args.alt if real else 100.0))

    if real:
        und = None
        if args.calib:
            z = np.load(args.calib)
            und = (z["K"], z["dist"])
        state = StateFeed(args.state_port)
        if args.replay:
            cam = ReplayCam(os.path.expanduser(args.video), state, out, args.replay_max_w, und)
        else:
            cam = VideoCam(args.video, args.cam_latency_s, und)
        pose = None
        t0 = time.monotonic()
        while cam.get()[0] == 0:
            if time.monotonic() - t0 > 60:
                out.status("error", f"no frames from the camera {args.video}")
                sys.exit("no camera frames")
            out.status("waiting", "waiting for camera frames ...")
            time.sleep(0.3)
        print(f"camera {args.video} connected - localising every {args.interval:g} s", flush=True)
    else:
        try:
            from gz.transport13 import Node
        except ImportError:                                   # the venv does not see the system Gazebo modules
            sys.path.append("/usr/lib/python3/dist-packages")
            from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.pose_v_pb2 import Pose_V
        node = Node()
        topic = pick_camera_topic(node, args.topic)
        cam = Cam(node, topic, Image)
        pose = PoseFeed(node, args.world, args.model, Pose_V)
        state = None
        t0 = time.monotonic()
        while cam.get()[0] == 0:
            if time.monotonic() - t0 > 90:
                out.status("error", f"no camera frames on {topic} - is Gazebo running?")
                sys.exit("no camera frames")
            out.status("waiting", "waiting for camera frames ...")
            time.sleep(0.5)
        print(f"camera connected on {topic} - localising every {args.interval:g} s", flush=True)

    run_dir = None
    if not args.no_visual_log:
        base = Path(args.log_dir).expanduser() if args.log_dir else Path(__file__).resolve().parent / "flight visual logs"
        run_dir = base / time.strftime("run_%Y%m%d_%H%M%S")
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / "fixes.csv").write_text("fix,time,lat,lon,inliers,ms,alt_m\n")
        print(f"proof images (live frame | matched map) -> {run_dir}", flush=True)
    tried = 0
    last_n, next_t, nfix, last_status, lock_sent, k_lock = -1, 0.0, 0, 0.0, False, None
    while True:
        if time.monotonic() < next_t:
            time.sleep(0.02)
            continue
        n, rgb, t_cap = cam.get()
        if rgb is None or n == last_n:
            if real and getattr(cam, "err", None):
                out.status("error", cam.err)
                sys.exit(cam.err)
            time.sleep(0.02)
            continue
        roll = pitch = heading = None
        if real:
            st = state.get()
            alt = st["alt"] if st else None
            if st:
                roll, pitch, heading = st.get("roll"), st.get("pitch"), st.get("heading")
        else:
            p = pose.get()
            alt = p[2] if p else args.alt
        if alt is None or alt < min_alt:
            if time.monotonic() - last_status > 1.0:
                last_status = time.monotonic()
                txt = "waiting for telemetry (altitude / attitude from the flight controller)" if alt is None else \
                    f"waiting for altitude ({alt:.0f} / {min_alt:.0f} m)"
                out.status("waiting", txt)
            time.sleep(0.3)
            continue
        if real and roll is not None and pitch is not None and (abs(roll) > 20 or abs(pitch) > 20):
            out.status("searching", f"tilted {max(abs(roll), abs(pitch)):.0f} deg - frame skipped")
            time.sleep(0.2)
            continue
        last_n = n
        next_t = time.monotonic() + args.interval                  # period start-to-start, whatever the match takes
        try:
            t1 = time.time()
            tried += 1
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            cam_gsd = 2.0 * alt * math.tan(args.hfov / 2.0) / gray.shape[1]       # real m per frame pixel
            expect = cam_gsd / mm.gsd                                             # map px per frame px
            if k_lock:
                exp_used, tol = expect * k_lock, 0.30
            elif real:
                exp_used, tol = expect, 0.45
            else:
                exp_used, tol = None, 0.30
            rot = ((heading + args.cam_yaw) % 360.0) if (real and heading is not None) else None
            r, why = mm.locate(gray, exp_used, min_inl, tol, rot, args.rot_tol)
            ms = (time.time() - t1) * 1000.0
            if r is None:
                out.status("searching", f"{why} (frame {n}, {ms:.0f} ms)")
                continue
            k = r["scale"] / expect                                               # real metres per Gazebo metre
            if k_lock is None:
                if not (0.2 < k < 5.0):
                    out.status("searching", f"match with implausible scale x{k:.2f} ignored")
                    continue
                k_lock = k
            lat, lon = mm.px_to_latlon(r["px"], r["py"])
            if real and not args.no_tilt_comp and roll is not None and pitch is not None and heading is not None:
                dn, de = nadir_correction(alt, roll, pitch, heading)
                lat -= dn / M_PER_DEG
                lon -= de / (M_PER_DEG * math.cos(math.radians(lat)))
            nfix += 1
            first = not lock_sent
            k = 1.0 if (real or abs(k - 1.0) < 0.1) else round(k, 3)
            hdg = args.heading if (not real and abs(((r["hdg"] - args.heading + 180) % 360) - 180) < 8.0) else round(r["hdg"], 1)
            if first:
                lock_sent = True
                print(f"LOCKED: world scale x{k:g}, heading {hdg:.0f} ({r['inl']} inliers)", flush=True)
                if home is not None:
                    d = math.hypot((lat - home[0]) * M_PER_DEG,
                                   (lon - home[1]) * M_PER_DEG * math.cos(math.radians(home[0])))
                    print(f"first fix is {d:.1f} m from the home position", flush=True)
            print(f"fix {nfix}: {lat:.6f} {lon:.6f}  inl {r['inl']}/{r['n_match']}  {ms:.0f} ms", flush=True)
            if run_dir is not None and nfix <= args.log_max:
                try:
                    save_proof(run_dir, nfix, rgb, mm, r, lat, lon, ms, alt)
                    with open(run_dir / "fixes.csv", "a") as fcsv:
                        fcsv.write(f"{nfix},{time.strftime('%H:%M:%S')},{lat:.7f},{lon:.7f},{r['inl']},{ms:.0f},{alt:.1f}\n")
                    (run_dir / "summary.txt").write_text(f"frames tried: {tried}\nmatched (images saved): {nfix}\n")
                except Exception as e:
                    print(f"proof image failed: {e}", flush=True)
            out.send({"k": "ga_fix", "t": t_cap, "lat": float(lat), "lon": float(lon), "inl": r["inl"],
                      "mode": r["mode"], "spread": None, "ms": float(ms), "n": nfix,
                      "scale": k, "hdg": hdg, "first": first})
            out.status("tracking", f"{r['mode']}, {r['inl']} inliers, {ms:.0f} ms")
        except Exception:
            traceback.print_exc()
            out.status("error", "exception - see geoanchor_node.log")


if __name__ == "__main__":
    main()
