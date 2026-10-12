#!/usr/bin/env python3
"""gcs_odo.py -- the WHOLE ground station (UI + GCS logic + 3D button + VISUAL ODO panel) in ONE file.  Run:  python3 gcs_odo.py

  swarm_agent.py  all swarm algorithms; the GCS starts one copy per drone (keep it next to this file)

Search for "# SECTION:" to jump around.  Order inside this file:
  1 imports / dependency checks
  2 UI + config   colours, sizes, timings, map styles, glass panel, icons, widgets, button helpers
  3 GCS logic     MAVLink link, connection bar, layout, map, panels, telemetry loop, swarm control
  4 3D map button (needs drone_3d.py in the same folder)
  5 DashboardApp + main()
"""
import time
import queue
import threading
import math
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk
import random
import json
import socket
import os
import subprocess
import sys
import re

try:
    from pymavlink import mavutil
except ImportError:
    raise SystemExit("Missing dependency 'pymavlink'.\nInstall it with:  pip install pymavlink")

try:
    import tkintermapview
except ImportError:
    raise SystemExit("Missing dependency 'tkintermapview'.\nInstall it with:  pip install tkintermapview")

try:
    import serial.tools.list_ports as list_ports
except ImportError:
    raise SystemExit("Missing dependency 'pyserial'.\nInstall it with:  pip install pyserial")


import os
import sys
import tkinter as tk
import math
import tkinter.font as tkfont

try:
    from PIL import Image, ImageDraw, ImageTk, ImageFont
except ImportError:
    raise SystemExit("Missing dependency 'Pillow'.\nInstall it with:  pip install pillow")


##############################################################################
# SECTION: config.py   (was gcs/config.py)
##############################################################################
# Folder that holds gcs.py / swarm_agent.py (the project root). The GCS starts
# swarm_agent.py from here, exactly like before when both files sat together.
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

# ---------------- CONFIG ----------------
SIM_CONNECTION_STRING = "udp:127.0.0.1:14551"
REAL_BAUD_RATE = 57600  # standard default for SiK telemetry radios

# Device name fragments used to spot a telemetry radio among serial ports.
# Linux SiK radios usually show up as ttyUSB*; some show up as ttyACM*.
# Windows shows COM*. This is a best-effort match, not a strict ID check.
TELEMETRY_PORT_HINTS = ("ttyUSB", "ttyACM", "COM")

DEFAULT_MISSION_PATH = "/home/rakesh/Documents/Drone/mission.waypoints"
UPDATE_INTERVAL_MS = 200
SERIAL_POLL_INTERVAL_S = 1.5
LINK_TIMEOUT_S = 3.0     # no MAVLink message from a sysid for this long -> "LOST"
LINK_CHECK_MS = 1000     # how often we re-check link age

DEFAULT_MAP_ZOOM = 18    # zoom level used both on startup and by the
                          # Locate Drone button, so "locate" always
                          # snaps back to the exact same framing you
                          # get when the dashboard first opens.
MAX_LOG_LINES = 300

SATELLITE_TILE_SERVER = "https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}"

# 57: every selectable base map, grouped for the drop-down:
#   (group title, [(id, label, tile url, max zoom), ...])
# The id "satellite" is the startup map (self.current_tile_server). Add your own line to any group to
# get a new entry in the drop-down.
_ESRI = "https://server.arcgisonline.com/ArcGIS/rest/services"
MAP_STYLES = [
    ("SATELLITE", [
        ("satellite",     "Google Satellite", SATELLITE_TILE_SERVER, 22),
        ("google_hybrid", "Google Hybrid",    "https://mt0.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}", 22),
        ("esri_sat",      "Esri World Imagery", _ESRI + "/World_Imagery/MapServer/tile/{z}/{y}/{x}", 19),
    ]),
    ("TERRAIN", [
        ("opentopo",      "OpenTopoMap",      "https://a.tile.opentopomap.org/{z}/{x}/{y}.png", 17),
        ("google_terrain", "Google Terrain",  "https://mt0.google.com/vt/lyrs=p&hl=en&x={x}&y={y}&z={z}", 20),
    ]),
]
MAP_STYLE_BY_ID = {sid: (name, url, mz)
                   for _g, items in MAP_STYLES for sid, name, url, mz in items}

# Monochrome / professional color palette
COL_BG = "#1b1b1d"
COL_BTN = "#2c2c2e"
COL_BTN_HOVER = "#3a3a3d"
COL_BTN_ACCENT = "#33373c"
COL_BTN_ACCENT_HOVER = "#42474e"
COL_BORDER = "#3f3f46"
COL_TEXT = "#e6e6e6"
COL_TEXT_DIM = "#9a9a9a"
COL_VALUE = "#7fd1ae"
COL_ARMED = "#e07a7a"
COL_WARN = "#e0a03d"
COL_DOT_OFF = "#616161"
COL_DOT_ON = "#3ddc84"

# ---- Glass overlay look (52) ----
# The map now fills the whole window and every panel floats on top of it.
# GLASS_OVERLAYS=True  -> each panel is a borderless, semi-transparent
#                         window (real see-through to the map).
# GLASS_OVERLAYS=False -> fallback: panels are plain opaque in-window
#                         frames (use this if your desktop has no
#                         compositor and transparency looks wrong).
GLASS_OVERLAYS = True
GLASS_ALPHA = 0.86
COL_GLASS = "#171a20"
COL_GLASS_BORDER = "#4a505c"
TELEM_PANEL_W = 530          # wide enough that the Base Lon field isn't clipped
COL_CARD = "#12151a"            # inset "instrument card" background (darker than the glass)
COL_PRIMARY = "#1f3d33"         # primary action (muted mint)
COL_PRIMARY_HOVER = "#285243"
COL_PRIMARY_BORDER = "#3f7d66"
COL_DANGER_HOVER = "#4a2b2b"
COL_DANGER_BORDER = "#8a4747"
COL_ACTIVE = "#4a3a1c"          # a toggle that is currently ON (marking) -- muted amber
COL_ACTIVE_HOVER = "#5a4723"
PANEL_RADIUS = 14            # corner radius of every floating panel

# Mission / path colors
COL_MISSION_PATH = "#ffd400"
COL_RTL_PATH = "#ff3b30"
COL_MANUAL_PATH = "#39ff14"
COL_TRIANGLE = "#39ff14"      # formation triangle lines + distance labels (drawn while the swarm flies)
# One shared trajectory colour for every drone (Mission Planner style yellow).
COL_TRAJECTORY = "#ffd400"
COL_RSSI_LINE = "#3b82f6"   # blue -- signal-strength graph line, per request

# Per-drone waypoint marking colors -- Drone A/B/C get fixed, recognizable
# colors; any drone beyond that gets the next unused color from the pool
# below (all chosen to read clearly against satellite imagery), falling
# back to a randomly generated vivid color if the pool ever runs out.
DEFAULT_WP_COLORS = {
    1: "#39ff14",   # Drone A - green
    2: "#ffd400",   # Drone B - yellow
    3: "#ff3b30",   # Drone C - red
}
WP_COLOR_POOL = [
    "#00d4ff",   # cyan
    "#ff8c00",   # orange
    "#c874ff",   # purple
    "#ff4fa3",   # pink
    "#5b8cff",   # blue
    "#00ffab",   # spring green
    "#ffffff",   # white
    "#ff6161",   # coral
]
# -----------------------------------------


##############################################################################
# SECTION: helpers.py   (was gcs/helpers.py)
##############################################################################
def hex_to_rgba(hex_color, alpha=255):
    """Converts a '#rrggbb' string (as used by tkintermapview paths/markers)
    into the (r, g, b, a) tuple Pillow needs to draw a matching dot icon."""
    hex_color = hex_color.lstrip("#")
    r = int(hex_color[0:2], 16)
    g = int(hex_color[2:4], 16)
    b = int(hex_color[4:6], 16)
    return (r, g, b, alpha)


def display_mode(mode):
    """Maps internal mode tokens to what the user should actually see.
    Internal logic still compares against 'REAL' / 'SIM' everywhere else --
    this only affects text shown in the UI and log."""
    return "TELE" if mode == "REAL" else mode


_CARDINALS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")


def cardinal_for(deg):
    try:
        return _CARDINALS[int(((float(deg) % 360) + 22.5) // 45) % 8]
    except (TypeError, ValueError):
        return "--"


##############################################################################
# SECTION: glass_panel.py   (was gcs/ui/glass_panel.py)
##############################################################################
# ---------------- rounded corners for the floating panels ----------------
# Tk can't round a window by itself, so the window *shape* is clipped with the
# platform's own call: XShape on Linux/X11, SetWindowRgn on Windows. If
# anything about that fails the panels simply stay square (nothing breaks).
_SHAPE_STATE = {"x11": None, "disabled": False}


def _window_ids(win):
    """Every X window that makes up this Tk toplevel: the inner window and
    (once the window has been mapped) Tk's outer wrapper window."""
    ids = {int(win.winfo_id())}
    try:
        ids.add(int(win.wm_frame(), 16))
    except Exception:
        pass
    return tuple(sorted(ids))


def _round_window_x11(win, w, h, r):
    import ctypes
    import ctypes.util
    st = _SHAPE_STATE["x11"]
    if st is None:
        x11 = ctypes.CDLL(ctypes.util.find_library("X11"))
        xext = ctypes.CDLL(ctypes.util.find_library("Xext"))
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XCreateBitmapFromData.restype = ctypes.c_ulong
        x11.XCreateBitmapFromData.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_char_p,
                                              ctypes.c_uint, ctypes.c_uint]
        x11.XFreePixmap.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        x11.XFlush.argtypes = [ctypes.c_void_p]
        xext.XShapeCombineMask.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
                                           ctypes.c_int, ctypes.c_int, ctypes.c_ulong, ctypes.c_int]
        x11.XQueryTree.restype = ctypes.c_int
        x11.XQueryTree.argtypes = [ctypes.c_void_p, ctypes.c_ulong,
                                   ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong),
                                   ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
                                   ctypes.POINTER(ctypes.c_uint)]
        x11.XFree.argtypes = [ctypes.c_void_p]
        x11.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
        dpy = x11.XOpenDisplay(None)
        if not dpy:
            raise RuntimeError("cannot open X display")
        st = (x11, xext, dpy)
        _SHAPE_STATE["x11"] = st
    x11, xext, dpy = st

    mask = Image.new("1", (w, h), 0)
    ImageDraw.Draw(mask).rounded_rectangle([0, 0, w - 1, h - 1], radius=r, fill=1)
    data = mask.tobytes("raw", "1;R")          # X bitmaps are least-significant-bit first

    # 56: the white corners came from a parent X window (Tk's wrapper, or a
    # window-manager frame) that was NOT clipped and painted its white
    # background in the corners. wm_frame() doesn't always report it, so walk
    # the real X window tree upward and clip every ancestor below the root.
    ids = list(_window_ids(win))
    try:
        cur = int(win.winfo_id())
        for _ in range(6):
            root_ret, parent_ret = ctypes.c_ulong(0), ctypes.c_ulong(0)
            kids = ctypes.POINTER(ctypes.c_ulong)()
            nkids = ctypes.c_uint(0)
            ok = x11.XQueryTree(dpy, cur, ctypes.byref(root_ret), ctypes.byref(parent_ret),
                                ctypes.byref(kids), ctypes.byref(nkids))
            if kids:
                x11.XFree(ctypes.cast(kids, ctypes.c_void_p))
            if not ok or not parent_ret.value or parent_ret.value == root_ret.value:
                break
            cur = parent_ret.value
            if cur not in ids:
                ids.append(cur)
    except Exception:
        pass            # fall back to the ids we already had

    for wid in ids:
        pm = x11.XCreateBitmapFromData(dpy, wid, data, w, h)
        xext.XShapeCombineMask(dpy, wid, 0, 0, 0, pm, 0)   # ShapeBounding, ShapeSet
        x11.XFreePixmap(dpy, pm)
    x11.XSync(dpy, 0)      # make sure the server has applied it before we reveal
    return True


def _round_window(win, w, h, r):
    if _SHAPE_STATE["disabled"]:
        return False
    try:
        if sys.platform.startswith("win"):
            import ctypes
            hwnd = ctypes.windll.user32.GetParent(win.winfo_id()) or win.winfo_id()
            rgn = ctypes.windll.gdi32.CreateRoundRectRgn(0, 0, w + 1, h + 1, r * 2, r * 2)
            ctypes.windll.user32.SetWindowRgn(hwnd, rgn, True)
            return True
        if sys.platform == "darwin":
            _SHAPE_STATE["disabled"] = True
            return False
        return _round_window_x11(win, w, h, r)
    except Exception:
        _SHAPE_STATE["disabled"] = True
        return False


class GlassPanel:
    """A floating panel that sits on top of the full-screen map.

    Build whatever you like into `.body`. In GLASS mode the panel is its own
    borderless, slightly transparent window kept glued to the main window; in
    fallback mode it is a normal frame placed over the map. Position it with
    set_geometry() using the same idea as Tk's place(): relx/rely + anchor +
    pixel offsets (dx/dy and width/height may be ints or f(W, H) callables).
    closable=True adds an X in the top-right corner that hides the panel.
    """

    ANCHORS = {"nw": (0, 0), "n": (.5, 0), "ne": (1, 0), "w": (0, .5), "c": (.5, .5),
               "e": (1, .5), "sw": (0, 1), "s": (.5, 1), "se": (1, 1)}

    def __init__(self, app, bg=COL_GLASS, border=COL_GLASS_BORDER, alpha=GLASS_ALPHA,
                 closable=False, start_visible=True, popup=False, radius=PANEL_RADIUS, on_close=None,
                 close_style="x", title=None, title_font=("Segoe UI", 16, "bold")):
        self.app = app
        self.root = app.root
        self.desired = start_visible
        self.is_popup = popup
        self._spec = dict(relx=0.0, rely=0.0, anchor="nw", dx=0, dy=0, width=None, height=None)
        self._last_geo = None
        self._toplevel = GLASS_OVERLAYS
        self._radius = radius
        self._shape_key = None
        self._alpha = alpha
        self._hidden_for_shape = False
        self._seq = 0      # order in which popups were last shown (higher = on top)

        if self._toplevel:
            self.win = tk.Toplevel(self.root, bg=border)
            self.win.withdraw()
            self.win.overrideredirect(True)
            try:
                self.win.attributes("-alpha", alpha)
            except tk.TclError:
                pass
            host = self.win
            self.win.bind("<Configure>", self._on_win_configure)
            self.win.bind("<Map>", self._on_win_map)
        else:
            self.win = None
            host = self.root

        self.frame = tk.Frame(host, bg=bg, highlightthickness=1,
                              highlightbackground=border, highlightcolor=border)
        if self._toplevel:
            self.frame.pack(fill="both", expand=True)

        if closable and close_style == "chevron":
            head = tk.Frame(self.frame, bg=bg)
            head.pack(fill="x", side="top")
            chev = tk.Label(head, text="\u00bb", font=("Segoe UI", 24, "bold"), fg=COL_TEXT,
                            bg=bg, cursor="hand2", padx=12)
            chev.pack(side="left", padx=(6, 0), pady=(0, 0))
            self.head = head
            if title:
                self.title_label = tk.Label(head, text=title, font=title_font,
                                            fg=COL_TEXT, bg=bg)
                self.title_label.pack(side="left", padx=(0, 8))
            chev.bind("<Enter>", lambda e: chev.config(fg="#e74c3c"))
            chev.bind("<Leave>", lambda e: chev.config(fg=COL_TEXT))
            chev.bind("<Button-1>", lambda e: (on_close or self.hide)())
        elif closable:
            head = tk.Frame(self.frame, bg=bg)
            head.pack(fill="x", side="top")
            x_btn = tk.Canvas(head, width=24, height=24, bg=bg,
                              highlightthickness=0, cursor="hand2")
            x_btn.pack(side="right", padx=(0, 6), pady=(6, 0))
            x_bg = x_btn.create_oval(2, 2, 22, 22, fill=bg, outline="")
            x_btn.create_line(8, 8, 16, 16, fill=COL_TEXT, width=2)
            x_btn.create_line(16, 8, 8, 16, fill=COL_TEXT, width=2)
            x_btn.bind("<Enter>", lambda e: x_btn.itemconfig(x_bg, fill="#c0392b"))
            x_btn.bind("<Leave>", lambda e: x_btn.itemconfig(x_bg, fill=bg))
            x_btn.bind("<Button-1>", lambda e: (on_close or self.hide)())

        self.body = tk.Frame(self.frame, bg=bg)
        self.body.pack(fill="both", expand=True)

        # content size changes (e.g. the "Marking: A, B" label) -> re-fit
        self.frame.bind("<Configure>", lambda e: self.app._schedule_layout(), add="+")
        app.panels.append(self)

    def _on_win_configure(self, event):
        if event.widget is self.win:
            self._round()

    def _on_win_map(self, event):
        if event.widget is self.win:
            self._round(force=True)
            # 56: re-clip several times while the window settles (Tk's
            # wrapper window / final size appear a moment after <Map>).
            for ms in (30, 120, 300, 700, 1500, 3000):
                self.win.after(ms, lambda: self._round_and_reveal(force=True))

    def _set_alpha(self, a):
        try:
            self.win.attributes("-alpha", a)
        except tk.TclError:
            pass

    def _round_and_reveal(self, force=False):
        """Re-clip (cheap if nothing changed), then make the panel visible.
        Panels are kept fully transparent until their rounded shape is on,
        so no square/white corners are ever drawn."""
        try:
            if not self.win.winfo_exists():
                return
        except tk.TclError:
            return
        self._round(force=force)
        if self._hidden_for_shape:
            self._hidden_for_shape = False
            self._set_alpha(self._alpha)

    def _round(self, force=False):
        """Clip the window to a rounded rectangle (re-done whenever it
        resizes, or when Tk's outer wrapper window appears/changes)."""
        if not self._toplevel or self._radius <= 0:
            return
        try:
            w, h = self.win.winfo_width(), self.win.winfo_height()
            key = (w, h, _window_ids(self.win))
        except tk.TclError:
            return
        if w < 4 or h < 4 or (key == self._shape_key and not force):
            return
        if _round_window(self.win, w, h, self._radius):
            self._shape_key = key

    # -- public API --
    def set_geometry(self, relx=0.0, rely=0.0, anchor="nw", dx=0, dy=0, width=None, height=None):
        self._spec = dict(relx=relx, rely=rely, anchor=anchor, dx=dx, dy=dy,
                          width=width, height=height)

    def show(self):
        self.desired = True
        self.app._popup_seq += 1
        self._seq = self.app._popup_seq
        self.apply()
        self.lift()

    def hide(self):
        self.desired = False
        self.apply()

    def toggle(self):
        if self.desired:
            self.hide()
        else:
            self.show()

    def lift(self):
        if self._toplevel and self.desired:
            try:
                self.win.lift()
            except tk.TclError:
                pass

    @staticmethod
    def _ev(v, W, H):
        return v(W, H) if callable(v) else v

    def apply(self):
        r = self.root
        try:
            ok = (self.desired and self.app.app_active
                  and r.winfo_viewable() and r.state() != "iconic")
        except tk.TclError:
            return
        if not ok:
            if self._toplevel:
                self.win.withdraw()
            else:
                self.frame.place_forget()
            self._last_geo = None
            self._shape_key = None       # re-clip the rounded corners when shown again
            return

        W, H = r.winfo_width(), r.winfo_height()
        s = self._spec
        self.frame.update_idletasks()
        w = self._ev(s["width"], W, H)
        h = self._ev(s["height"], W, H)
        fixed_size = (w is not None) or (h is not None)
        if w is None:
            w = self.frame.winfo_reqwidth()
        if h is None:
            h = self.frame.winfo_reqheight()
        w, h = int(w), int(h)
        ax, ay = self.ANCHORS[s["anchor"]]
        x = int(s["relx"] * W + self._ev(s["dx"], W, H) - ax * w)
        y = int(s["rely"] * H + self._ev(s["dy"], W, H) - ay * h)

        if self._toplevel:
            X, Y = r.winfo_rootx() + x, r.winfo_rooty() + y
            geo = f"{w}x{h}+{X}+{Y}" if fixed_size else f"+{X}+{Y}"
            if geo != self._last_geo:
                self.win.geometry(geo)
                self._last_geo = geo
            if not self.win.winfo_viewable():
                if self._radius > 0 and not _SHAPE_STATE["disabled"]:
                    self._hidden_for_shape = True
                    self._set_alpha(0.0)      # invisible until the corners are clipped
                    self.win.after(1000, self._round_and_reveal)   # safety net
                self.win.deiconify()
                self.win.lift()
            self._round()
        else:
            self.frame.place(x=x, y=y, width=w, height=h, anchor="nw")
            self.frame.lift()


##############################################################################
# SECTION: icons.py   (was gcs/ui/icons.py)
##############################################################################
def _load_bold_font(px):
    """Bold UI font for icon labels (falls back to Pillow's default)."""
    for name in ("DejaVuSans-Bold.ttf", "LiberationSans-Bold.ttf",
                 "Arial Bold.ttf", "arialbd.ttf"):
        try:
            return ImageFont.truetype(name, px)
        except Exception:
            continue
    try:
        return ImageFont.load_default()
    except Exception:
        return None


def _drone_icon_image(heading_deg=0.0, size=56):
    """Drone logo (design 2 - solid discs): two dark-green rotor discs at the
    front, two dark-pink discs at the rear, a dark centre body and a white
    heading arrow pointing out of the nose. Drawn at 4x and downsampled for
    smooth edges, then rotated to the heading (0 = up / north)."""
    S = 4
    B = size * S
    img = Image.new("RGBA", (B, B), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    c = B / 2
    G, GD = (22, 128, 44, 255), (10, 70, 24, 255)       # dark green / edge
    P, PD = (178, 38, 120, 255), (100, 15, 66, 255)     # dark pink / edge
    K = (15, 15, 15, 255)
    a = B * 0.26
    r = B * 0.14

    for dx, dy, front in [(-1, -1, True), (1, -1, True), (-1, 1, False), (1, 1, False)]:
        col, edge = (G, GD) if front else (P, PD)
        x, y = c + dx * a, c + dy * a
        d.line([(c, c), (x, y)], fill=K, width=S * 3)
        d.ellipse([x - r, y - r, x + r, y + r], fill=col, outline=edge, width=S * 2)
        d.ellipse([x - r * 0.3, y - r * 0.3, x + r * 0.3, y + r * 0.3], fill=edge)

    d.rounded_rectangle([c - B * 0.09, c - B * 0.11, c + B * 0.09, c + B * 0.11],
                        radius=S * 5, fill=(40, 40, 40, 255), outline=K, width=S)
    # heading arrow
    tip_y, base_y, hw = c - B * 0.30, c - B * 0.14, B * 0.07
    d.polygon([(c, tip_y), (c - hw, base_y), (c + hw, base_y)],
              fill=(235, 235, 235, 255), outline=K)

    img = img.resize((size, size), Image.LANCZOS)
    if heading_deg:
        img = img.rotate(-heading_deg, resample=Image.BICUBIC)
    return img


def create_drone_icon(size=56, heading_deg=0.0):
    return ImageTk.PhotoImage(_drone_icon_image(heading_deg, size))


def create_wp_pin_icon(label="", is_home=False):
    """Mission Planner style waypoint marker: a green map pin with a
    rounded light label box (number, or 'Home') at its upper right.
    The image is padded symmetrically so the pin's tip sits at the
    bottom-centre -- use icon_anchor='s' and it lands exactly on the
    coordinate."""
    S = 3
    pin_w, pin_h = 18 * S, 30 * S
    font = _load_bold_font(11 * S)
    text = str(label) if label != "" else ""
    if text:
        tmp = ImageDraw.Draw(Image.new("RGBA", (4, 4)))
        try:
            tb = tmp.textbbox((0, 0), text, font=font)
            tw, th = tb[2] - tb[0], tb[3] - tb[1]
        except Exception:
            tw, th = len(text) * 7 * S, 10 * S
        box_w, box_h = tw + 14 * S, th + 12 * S
    else:
        box_w = box_h = 0

    gap = 2 * S
    half = max(pin_w // 2, pin_w // 2 + gap + box_w) + 2 * S
    W = half * 2
    top_pad = max(0, int(box_h - 0.8 * (pin_w / 2))) + 2 * S
    H = pin_h + top_pad + 4 * S
    img = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    cx = W / 2
    tip_y = H - 2 * S
    head_r = pin_w / 2
    head_cy = tip_y - pin_h + head_r

    # soft shadow
    d.ellipse([cx - head_r * 0.7, tip_y - 3 * S, cx + head_r * 0.7, tip_y + S],
              fill=(0, 0, 0, 70))
    # teardrop body
    green = (46, 170, 56, 255)
    edge = (10, 70, 20, 255)
    d.polygon([(cx - head_r * 0.88, head_cy + head_r * 0.45),
               (cx + head_r * 0.88, head_cy + head_r * 0.45),
               (cx, tip_y)], fill=green, outline=edge)
    d.ellipse([cx - head_r, head_cy - head_r, cx + head_r, head_cy + head_r],
              fill=green, outline=edge, width=S)
    # tidy the seam between head and cone
    d.polygon([(cx - head_r * 0.80, head_cy + head_r * 0.40),
               (cx + head_r * 0.80, head_cy + head_r * 0.40),
               (cx, tip_y - S)], fill=green)
    # highlight
    hl = head_r * 0.36
    d.ellipse([cx - head_r * 0.45 - hl / 2, head_cy - head_r * 0.45 - hl / 2,
               cx - head_r * 0.45 + hl / 2, head_cy - head_r * 0.45 + hl / 2],
              fill=(190, 255, 190, 200))
    # tiny dark centre dot
    dr = head_r * 0.28
    d.ellipse([cx - dr, head_cy - dr, cx + dr, head_cy + dr], fill=(15, 80, 25, 255))

    # label box at the upper right
    if text:
        bx0 = cx + head_r * 0.75 + gap
        by1 = head_cy - head_r * 0.2
        by0 = by1 - box_h
        d.rounded_rectangle([bx0, by0, bx0 + box_w, by1], radius=6 * S,
                            fill=(214, 226, 245, 245), outline=(120, 140, 175, 255),
                            width=S)
        try:
            d.text((bx0 + box_w / 2, (by0 + by1) / 2), text, font=font,
                   fill=(14, 22, 100, 255), anchor="mm")
        except Exception:
            d.text((bx0 + 7 * S, by0 + 5 * S), text, fill=(14, 22, 100, 255))

    img = img.resize((W // S, H // S), Image.LANCZOS)
    return ImageTk.PhotoImage(img)


##############################################################################
# SECTION: edge_button.py   (was gcs/ui/widgets/edge_button.py)
##############################################################################
class EdgeButton(tk.Canvas):
    """55: a flat button where ONLY the outer corners are rounded.

    edge="left"  -> only the two LEFT corners are smooth (first button of a bar)
    edge="right" -> only the two RIGHT corners are smooth (last button of a bar)
    edge="none"  -> plain square button

    It mimics the bits of tk.Button this dashboard uses: config(text=...,
    bg=..., fg=..., command=..., state=...), cget(), invoke(), and the
    pack()/grid()/place() geometry managers (it is a Canvas underneath).
    """

    def __init__(self, parent, text, command=None, edge="none", bg=COL_BTN,
                 hover_bg=COL_BTN_HOVER, fg=COL_TEXT, font=("Segoe UI", 9),
                 width=9, height=1, radius=9, border=COL_BORDER, **kw):
        try:
            outer_bg = parent.cget("bg")
        except tk.TclError:
            outer_bg = COL_GLASS
        self._font = tkfont.Font(font=font)
        zero_w = max(1, self._font.measure("0"))
        px_w = width * zero_w + 10
        px_h = self._font.metrics("linespace") * height + 10
        super().__init__(parent, width=px_w, height=px_h, bg=outer_bg,
                         highlightthickness=0, bd=0, cursor="hand2")
        self._edge = edge
        self._radius = radius
        self._text = text
        self._command = command
        self._fill = bg
        self._hover = hover_bg
        self._fg = fg
        self._border = border
        self._state = "normal"
        self._hovering = False
        self._poly = None
        self._label = None
        self._redraw()
        self.bind("<Configure>", lambda e: self._redraw())
        self.bind("<Enter>", self._on_enter)
        self.bind("<Leave>", self._on_leave)
        self.bind("<ButtonRelease-1>", self._on_release)

    # ---- drawing ----
    def _outline_points(self, w, h):
        r = min(self._radius, h // 2, w // 2)
        left = self._edge == "left"
        right = self._edge == "right"
        pts = []

        def arc(cx, cy, a0, a1, steps=8):
            for i in range(steps + 1):
                a = math.radians(a0 + (a1 - a0) * i / steps)
                pts.extend((cx + r * math.cos(a), cy - r * math.sin(a)))

        # top-left
        if left:
            arc(r, r, 90, 180)
        else:
            pts.extend((0, 0))
        # bottom-left
        if left:
            arc(r, h - 1 - r, 180, 270)
        else:
            pts.extend((0, h - 1))
        # bottom-right
        if right:
            arc(w - 1 - r, h - 1 - r, 270, 360)
        else:
            pts.extend((w - 1, h - 1))
        # top-right
        if right:
            arc(w - 1 - r, r, 0, 90)
        else:
            pts.extend((w - 1, 0))
        return pts

    def _current_fill(self):
        return self._hover if (self._hovering and self._state != "disabled") else self._fill

    def _redraw(self):
        w = max(self.winfo_width(), int(self.cget("width")))
        h = max(self.winfo_height(), int(self.cget("height")))
        self.delete("all")
        self._poly = self.create_polygon(self._outline_points(w, h),
                                         fill=self._current_fill(),
                                         outline=self._border, width=1)
        self._label = self.create_text(
            w // 2, h // 2, text=self._text, font=self._font,
            fill=(COL_TEXT_DIM if self._state == "disabled" else self._fg))

    # ---- events ----
    def _on_enter(self, _e):
        self._hovering = True
        if self._poly:
            self.itemconfig(self._poly, fill=self._current_fill())

    def _on_leave(self, _e):
        self._hovering = False
        if self._poly:
            self.itemconfig(self._poly, fill=self._current_fill())

    def _on_release(self, e):
        if self._state == "disabled":
            return
        if 0 <= e.x <= self.winfo_width() and 0 <= e.y <= self.winfo_height():
            self.invoke()

    def invoke(self):
        if self._command and self._state != "disabled":
            return self._command()

    # ---- tk.Button-style config ----
    def configure(self, cnf=None, **kw):
        if cnf:
            kw.update(cnf)
        redraw = False
        if "text" in kw:
            self._text = kw.pop("text"); redraw = True
        if "command" in kw:
            self._command = kw.pop("command")
        for k in ("bg", "background"):
            if k in kw:
                self._fill = kw.pop(k); redraw = True
        for k in ("fg", "foreground"):
            if k in kw:
                self._fg = kw.pop(k); redraw = True
        if "activebackground" in kw:
            self._hover = kw.pop("activebackground")
        if "state" in kw:
            self._state = kw.pop("state"); redraw = True
        for k in ("activeforeground", "relief", "bd", "borderwidth",
                  "highlightbackground", "highlightcolor", "highlightthickness"):
            kw.pop(k, None)
        if kw:
            super().configure(**kw)
            redraw = True
        if redraw and self._poly is not None:
            self._redraw()

    config = configure

    def cget(self, key):
        if key == "text":
            return self._text
        if key in ("bg", "background"):
            return self._fill
        if key in ("fg", "foreground"):
            return self._fg
        if key == "state":
            return self._state
        if key == "command":
            return self._command
        return super().cget(key)


##############################################################################
# SECTION: battery_indicator.py   (was gcs/ui/widgets/battery_indicator.py)
##############################################################################
class BatteryIndicator(tk.Canvas):
    """Small battery icon (body + nub) with a proportional fill bar,
    color-coded by charge level. Kept compact for use as a map overlay."""

    def __init__(self, parent, width=24, height=11, **kwargs):
        super().__init__(parent, width=width, height=height, bg=COL_BTN,
                          highlightthickness=0, **kwargs)
        self.w = width
        self.h = height
        self.percent = None
        self._draw()

    def update_percent(self, percent):
        self.percent = max(0, min(100, percent))
        self._draw()

    def _draw(self):
        self.delete("all")
        w, h = self.w, self.h
        body_w = w - 5
        nub_w = 3

        self.create_rectangle(1, 1, body_w, h - 1, outline=COL_TEXT, width=1)
        self.create_rectangle(body_w, h * 0.28, body_w + nub_w, h * 0.72,
                               fill=COL_TEXT, outline="")

        if self.percent is not None:
            fill_w = max(0, (body_w - 3) * (self.percent / 100.0))
            if self.percent > 50:
                color = "#3ddc84"
            elif self.percent > 20:
                color = COL_WARN
            else:
                color = COL_ARMED
            self.create_rectangle(2, 2, 2 + fill_w, h - 2, fill=color, outline="")


##############################################################################
# SECTION: readout_row.py   (was gcs/ui/widgets/readout_row.py)
##############################################################################
class ReadoutRow(tk.Frame):
    """A row of small captioned values, e.g.  ROLL 1.2deg | PITCH -0.4deg.
    set() with no arguments / None values shows '--'."""

    def __init__(self, parent, captions, bg=COL_CARD):
        super().__init__(parent, bg=bg)
        self._vals = []
        for i, cap in enumerate(captions):
            cell = tk.Frame(self, bg=bg)
            cell.pack(side="left", expand=True, fill="x")
            tk.Label(cell, text=cap, font=("Segoe UI", 7, "bold"), fg=COL_TEXT_DIM,
                     bg=bg).pack()
            v = tk.Label(cell, text="--", font=("Segoe UI", 11, "bold"), fg=COL_VALUE, bg=bg)
            v.pack()
            self._vals.append(v)
            if i < len(captions) - 1:
                tk.Frame(self, bg=COL_BORDER, width=1).pack(side="left", fill="y", pady=2)

    def set(self, *values):
        for lab, val in zip(self._vals, values or ()):
            lab.config(text="--" if val is None else str(val))
        for lab in self._vals[len(values or ()):]:
            lab.config(text="--")


##############################################################################
# SECTION: smooth_check.py   (was gcs/ui/widgets/smooth_check.py)
##############################################################################
class SmoothCheck(tk.Frame):
    """Check box with anti-aliased rounded corners (drawn at 8x and scaled
    down) plus a clickable label. Three looks: off, on (mint + tick) and
    'partial' (mint + dash, used by Select all). Works with a BooleanVar like
    a normal Checkbutton; pass on_click to take over what a click does."""

    _cache = {}

    def __init__(self, parent, text, variable, font, bg, fg=COL_TEXT, size=22,
                 on_click=None):
        super().__init__(parent, bg=bg, cursor="hand2")
        self.var, self._bg, self._size = variable, bg, size
        self._on_click = on_click
        self._hover = False
        self._partial = False
        self.box = tk.Label(self, bd=0, highlightthickness=0, bg=bg, cursor="hand2")
        self.box.pack(side="left")
        self.lbl = tk.Label(self, text=text, font=font, fg=fg, bg=bg, cursor="hand2")
        self.lbl.pack(side="left", padx=(8, 0))
        for w in (self, self.box, self.lbl):
            w.bind("<Button-1>", self._click)
            w.bind("<Enter>", lambda e: self._set_hover(True))
            w.bind("<Leave>", lambda e: self._set_hover(False))
        self._trace = variable.trace_add("write", lambda *a: self._refresh())
        self.bind("<Destroy>", self._on_destroy)
        self._refresh()

    def _on_destroy(self, e):
        if e.widget is self:
            try:
                self.var.trace_remove("write", self._trace)
            except (tk.TclError, ValueError):
                pass

    def _click(self, _e=None):
        if self._on_click:
            self._on_click()
        else:
            self.var.set(not self.var.get())

    def _set_hover(self, h):
        self._hover = h
        self._refresh()

    def set_partial(self, partial):
        if partial != self._partial:
            self._partial = partial
            self._refresh()

    def _state(self):
        if self.var.get():
            return "on"
        return "partial" if self._partial else "off"

    def _refresh(self):
        try:
            self.box.config(image=self._image(self._state(), self._hover))
        except tk.TclError:
            pass

    def _image(self, state, hover):
        key = (state, hover, self._bg, self._size)
        img = SmoothCheck._cache.get(key)
        if img is None:
            S = 8
            W = self._size * S
            im = Image.new("RGB", (W, W), self._bg)
            d = ImageDraw.Draw(im)
            pad, r = S, int(W * 0.30)
            box = (pad, pad, W - pad - 1, W - pad - 1)

            def rrect(b, **kw):
                try:
                    d.rounded_rectangle(b, radius=r, **kw)
                except AttributeError:          # very old Pillow
                    d.rectangle(b, **{k: v for k, v in kw.items() if k != "width"})

            ink = "#10261d"
            if state == "off":
                rrect(box, fill=COL_CARD,
                      outline=COL_VALUE if hover else COL_GLASS_BORDER, width=int(S * 1.7))
            else:
                rrect(box, fill="#95e0c0" if hover else COL_VALUE)
                lw = int(S * 2.5)
                if state == "on":
                    pts = [(W * 0.27, W * 0.52), (W * 0.43, W * 0.68), (W * 0.74, W * 0.33)]
                    d.line(pts, fill=ink, width=lw, joint="curve")
                    for x, y in (pts[0], pts[-1]):
                        d.ellipse((x - lw / 2, y - lw / 2, x + lw / 2, y + lw / 2), fill=ink)
                else:
                    pts = [(W * 0.30, W * 0.50), (W * 0.70, W * 0.50)]
                    d.line(pts, fill=ink, width=lw)
                    for x, y in pts:
                        d.ellipse((x - lw / 2, y - lw / 2, x + lw / 2, y + lw / 2), fill=ink)
            im = im.resize((self._size, self._size), Image.LANCZOS)
            img = ImageTk.PhotoImage(im)
            SmoothCheck._cache[key] = img
        return img


##############################################################################
# SECTION: rssi_graph.py   (was gcs/ui/widgets/rssi_graph.py)
##############################################################################
class RSSIGraph(tk.Canvas):
    """Rolling line graph of radio signal strength (RADIO_STATUS.rssi,
    0-255 scale from the SiK radio, shown here as a 0-100% line) for one
    drone. Keeps the last ROLLING_WINDOW samples and redraws as a simple
    blue polyline over a faint percentage grid -- same drawing style as
    the other gauges (dark canvas, COL_BORDER grid, COL_TEXT_DIM labels),
    just line-chart shaped instead of circular."""

    ROLLING_WINDOW = 60  # ~60 samples of rolling history

    def __init__(self, parent, width=280, height=85, **kwargs):
        super().__init__(parent, width=width, height=height, bg="#111113",
                          highlightthickness=1, highlightbackground=COL_BORDER, **kwargs)
        self.w = width
        self.h = height
        self.samples = []  # list of 0-100 values, oldest first
        self._draw()

    def add_sample(self, percent):
        self.samples.append(max(0, min(100, percent)))
        if len(self.samples) > self.ROLLING_WINDOW:
            self.samples = self.samples[-self.ROLLING_WINDOW:]
        self._draw()

    def clear(self):
        self.samples = []
        self._draw()

    def _draw(self):
        self.delete("all")
        w, h = self.w, self.h
        pad_l, pad_r, pad_t, pad_b = 24, 6, 6, 6
        plot_w = w - pad_l - pad_r
        plot_h = h - pad_t - pad_b

        # Faint 0/50/100% gridlines with labels, so the line is readable
        # at a glance instead of being a shape with no scale.
        for pct in (0, 50, 100):
            y = pad_t + plot_h * (1 - pct / 100.0)
            self.create_line(pad_l, y, w - pad_r, y, fill=COL_BORDER, width=1)
            self.create_text(pad_l - 4, y, text=str(pct), fill=COL_TEXT_DIM,
                              font=("Segoe UI", 7), anchor="e")

        if len(self.samples) < 2:
            self.create_text(w / 2, h / 2, text="No signal data yet",
                              fill=COL_TEXT_DIM, font=("Segoe UI", 8))
            return

        n = len(self.samples)
        step = plot_w / (self.ROLLING_WINDOW - 1)
        # Right-align samples so the newest point always sits at the same
        # spot near the right edge as older points scroll off to the left.
        start_x = w - pad_r - (n - 1) * step
        points = []
        for i, val in enumerate(self.samples):
            x = start_x + i * step
            y = pad_t + plot_h * (1 - val / 100.0)
            points.extend([x, y])
        self.create_line(*points, fill=COL_RSSI_LINE, width=2, smooth=True)

        latest = self.samples[-1]
        self.create_text(w - pad_r, pad_t, text=f"{latest}%", fill=COL_TEXT,
                          font=("Segoe UI", 8, "bold"), anchor="ne")


##############################################################################
# SECTION: gauge_corners  (shared by the compass + attitude gauges)
##############################################################################
class _GaugeCornerMixin:
    """The square dial sits centred in a (possibly wider) canvas; the two small readouts (e.g. ROLL / PITCH)
    are drawn in the two bottom corners of that canvas, outside the round dial."""

    def _corner_init(self, size):
        self.cw = size
        self.corner_caps = ("", "")
        self.corner_vals = ("--", "--")

    def resize(self, size, width=None):
        width = max(int(size), int(width or size))
        if size == self.size and width == self.cw:
            return
        self.size, self.cw = int(size), width
        self.config(width=width, height=int(size))
        self._draw()

    def set_corners(self, a, b):
        vals = tuple("--" if v is None else str(v) for v in (a, b))
        if vals != self.corner_vals:
            self.corner_vals = vals
            self.delete("corner")
            self._draw_corner_text()

    def _finish_draw(self):
        if self.cw > self.size:
            self.move("all", (self.cw - self.size) / 2.0, 0)
        self._draw_corner_text()

    def _draw_corner_text(self):
        s, W = self.size, self.cw
        fs = 9 if s >= 120 else 8
        for i, (cap, val) in enumerate(zip(self.corner_caps, self.corner_vals)):
            x, anc = (4, "sw") if i == 0 else (W - 4, "se")
            self.create_text(x, s - 3, text=val, anchor=anc, fill=COL_VALUE,
                             font=("Segoe UI", fs, "bold"), tags="corner")
            self.create_text(x, s - 3 - fs - 7, text=cap, anchor=anc, fill=COL_TEXT_DIM,
                             font=("Segoe UI", 6, "bold"), tags="corner")


class GaugeCornerReadout:
    """Same set() interface as ReadoutRow, but the two values are drawn inside the gauge (bottom corners)."""

    def __init__(self, gauge, captions):
        self.gauge = gauge
        gauge.corner_caps = tuple(captions)
        gauge.delete("corner")
        gauge._draw_corner_text()

    def set(self, *values):
        v = list(values or ()) + [None, None]
        self.gauge.set_corners(v[0], v[1])


##############################################################################
# SECTION: compass_gauge.py   (was gcs/ui/widgets/compass_gauge.py)
##############################################################################
class CompassGauge(_GaugeCornerMixin, tk.Canvas):
    """Heading indicator styled after a real directional gyro: a
    numbered compass card (N/E/S/W plus the 3/6/12/15/21/24/30/33
    minor markers) that rotates so the current heading always sits at
    the top lubber line, beneath a FIXED white aircraft silhouette
    (matching the reference gauge photo)."""

    def __init__(self, parent, size=84, **kwargs):
        super().__init__(parent, width=size, height=size, bg=COL_BG,
                          highlightthickness=0, **kwargs)
        self.size = size
        self._corner_init(size)
        self.heading = 0.0
        self._draw()

    def update_heading(self, heading_deg):
        self.heading = heading_deg
        self._draw()

    def _draw(self):
        self.delete("all")
        s = self.size
        cx, cy = s / 2, s / 2
        r = s / 2 - 6
        card_r = r - 4
        heading = self.heading

        # Outer bezel: pale ring + heavy black rim.
        self.create_oval(cx - r - 3, cy - r - 3, cx + r + 3, cy + r + 3,
                          fill="#0a0a0a", outline="")
        self.create_oval(cx - r, cy - r, cx + r, cy + r, fill="#cfcfd2", outline="")
        self.create_oval(cx - card_r, cy - card_r, cx + card_r, cy + card_r,
                          fill="#2b2622", outline="#0a0a0a", width=2)

        # Rotating card: minor ticks every 10 deg, longer ticks + labels
        # every 30 deg (where N/E/S/W and the 3/6/12/... numbers sit).
        for deg in range(0, 360, 10):
            screen_ang = math.radians(deg - heading - 90)
            major = (deg % 30 == 0)
            length = 9 if major else 4
            x1 = cx + (card_r - length) * math.cos(screen_ang)
            y1 = cy + (card_r - length) * math.sin(screen_ang)
            x2 = cx + (card_r - 1) * math.cos(screen_ang)
            y2 = cy + (card_r - 1) * math.sin(screen_ang)
            self.create_line(x1, y1, x2, y2, fill="#e8e6e0", width=2 if major else 1)

        cardinals = {0: "N", 90: "E", 180: "S", 270: "W"}
        for deg in range(0, 360, 30):
            screen_ang = math.radians(deg - heading - 90)
            lx = cx + (card_r - 18) * math.cos(screen_ang)
            ly = cy + (card_r - 18) * math.sin(screen_ang)
            if deg in cardinals:
                self.create_text(lx, ly, text=cardinals[deg], fill="white",
                                  font=("Segoe UI", 10, "bold"))
            else:
                self.create_text(lx, ly, text=str(deg // 10), fill="#d8d6d0",
                                  font=("Segoe UI", 7))

        # Two small fixed decorative dots either side of the lubber line,
        # matching the twin marks in the reference photo.
        for off in (-25, 25):
            a = math.radians(off - 90)
            dx = cx + (card_r - 5) * math.cos(a)
            dy = cy + (card_r - 5) * math.sin(a)
            self.create_oval(dx - 1.5, dy - 1.5, dx + 1.5, dy + 1.5, fill="white", outline="")

        # Fixed white aircraft silhouette (does not rotate -- the card behind it does).
        self._draw_plane(cx, cy, r * 0.36)

        # Small decorative reference ball, bottom-left, like the photo.
        bx, by = cx - r * 0.72, cy + r * 0.72
        self.create_oval(bx - 5, by - 5, bx + 5, by + 5, fill="#1a1a1a",
                          outline="#e8e6e0", width=1)
        self._finish_draw()

    def _draw_plane(self, cx, cy, sc):
        """Simple top-down aircraft glyph (nose/wings/tail), sized as a
        fraction (sc) of the dial radius, always upright."""
        pts = [
            cx, cy - sc,
            cx - sc * 0.12, cy - sc * 0.55,
            cx - sc * 0.95, cy - sc * 0.05,
            cx - sc * 0.95, cy + sc * 0.18,
            cx - sc * 0.15, cy - sc * 0.05,
            cx - sc * 0.15, cy + sc * 0.55,
            cx - sc * 0.40, cy + sc * 0.78,
            cx - sc * 0.40, cy + sc * 0.92,
            cx, cy + sc * 0.72,
            cx + sc * 0.40, cy + sc * 0.92,
            cx + sc * 0.40, cy + sc * 0.78,
            cx + sc * 0.15, cy + sc * 0.55,
            cx + sc * 0.15, cy - sc * 0.05,
            cx + sc * 0.95, cy + sc * 0.18,
            cx + sc * 0.95, cy - sc * 0.05,
            cx + sc * 0.12, cy - sc * 0.55,
        ]
        self.create_polygon(*pts, fill="white", outline="#111113", width=1)


##############################################################################
# SECTION: attitude_gauge.py   (was gcs/ui/widgets/attitude_gauge.py)
##############################################################################
class AttitudeGauge(_GaugeCornerMixin, tk.Canvas):
    """Artificial-horizon gauge styled after a real aircraft attitude
    indicator: blue sky / brown ground split by a horizon line that
    rotates with roll and shifts with pitch, a 5/10-degree pitch ladder
    riding along with that same disk, a fixed yellow aircraft symbol at
    the center, and a fixed white roll-index pointer + tick scale on
    the bezel (matching the reference gauge photo)."""

    SKY = "#2f7dc4"
    GROUND = "#8a5a2e"

    def __init__(self, parent, size=84, **kwargs):
        super().__init__(parent, width=size, height=size, bg=COL_BG,
                          highlightthickness=0, **kwargs)
        self.size = size
        self._corner_init(size)
        self.roll = 0.0
        self.pitch = 0.0
        self._draw()

    def update_attitude(self, roll_deg, pitch_deg):
        self.roll = roll_deg
        self.pitch = pitch_deg
        self._draw()

    @staticmethod
    def _flatten(pts):
        flat = []
        for x, y in pts:
            flat.extend([x, y])
        return flat

    def _horizon_geometry(self, cx, cy, r, pitch_value):
        """u = direction along the horizon line (rotates with roll).
        n = direction perpendicular to it (which way pitch shifts the
        line). p0 = the point on that line closest to the center.
        Generalised over 'pitch_value' so the same math draws both the
        real horizon (pitch_value=self.pitch) and each pitch-ladder
        rung (pitch_value=self.pitch-5, -10, etc.)."""
        roll = math.radians(self.roll)
        px_per_deg = r / 45.0
        d = max(-r, min(r, pitch_value * px_per_deg))
        u = (math.cos(roll), math.sin(roll))
        n = (-math.sin(roll), math.cos(roll))
        p0 = (cx + n[0] * d, cy + n[1] * d)
        return u, n, p0, d

    def _draw(self):
        self.delete("all")
        s = self.size
        cx, cy = s / 2, s / 2
        r = s / 2 - 9    # leaves room outside the disk for the fixed bezel scale (same overall size as the compass)

        u, n, p0, d = self._horizon_geometry(cx, cy, r, self.pitch)

        half_sq = r * r - d * d
        if half_sq <= 0:
            color = self.GROUND if d > 0 else self.SKY
            self.create_oval(cx - r, cy - r, cx + r, cy + r, fill=color, outline="")
        else:
            hc = math.sqrt(half_sq)
            i1 = (p0[0] - u[0] * hc, p0[1] - u[1] * hc)
            i2 = (p0[0] + u[0] * hc, p0[1] + u[1] * hc)

            a1 = math.atan2(i1[1] - cy, i1[0] - cx)
            a2 = math.atan2(i2[1] - cy, i2[0] - cx)
            steps = 28

            def arc_points(start, end):
                diff = (end - start) % (2 * math.pi)
                pts = []
                for k in range(steps + 1):
                    a = start + diff * k / steps
                    pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
                return pts

            arc_a = arc_points(a1, a2)
            arc_b = arc_points(a2, a1)

            mid = arc_a[len(arc_a) // 2]
            side = (mid[0] - p0[0]) * n[0] + (mid[1] - p0[1]) * n[1]
            poly_a = [i1] + arc_a + [i2]
            poly_b = [i2] + arc_b + [i1]

            ground_poly = poly_a if side > 0 else poly_b
            sky_poly = poly_b if side > 0 else poly_a

            self.create_polygon(*self._flatten(sky_poly), fill=self.SKY, outline="")
            self.create_polygon(*self._flatten(ground_poly), fill=self.GROUND, outline="")

        # Horizon line itself.
        h1 = (p0[0] - u[0] * r, p0[1] - u[1] * r)
        h2 = (p0[0] + u[0] * r, p0[1] + u[1] * r)
        self.create_line(h1[0], h1[1], h2[0], h2[1], fill="white", width=2)

        # ---- Pitch ladder: +/-5 and +/-10 degree rungs riding with the disk ----
        for value, half_len_frac in ((5, 0.28), (10, 0.42), (-5, 0.28), (-10, 0.42)):
            _, _, p_val, d_val = self._horizon_geometry(cx, cy, r, self.pitch - value)
            if abs(d_val) > r * 0.94:
                continue  # scrolled off the visible disk
            max_half = math.sqrt(max(0.0, r * r - d_val * d_val))
            half_len = min(r * half_len_frac, max_half)
            gap = r * 0.14
            if half_len <= gap:
                continue
            for sign in (-1, 1):
                lx1 = (p_val[0] + u[0] * sign * gap, p_val[1] + u[1] * sign * gap)
                lx2 = (p_val[0] + u[0] * sign * half_len, p_val[1] + u[1] * sign * half_len)
                self.create_line(lx1[0], lx1[1], lx2[0], lx2[1], fill="white", width=2)
            label = str(abs(value))
            for sign in (-1, 1):
                tx = p_val[0] + u[0] * sign * (half_len + 7)
                ty = p_val[1] + u[1] * sign * (half_len + 7)
                self.create_text(tx, ty, text=label, fill="white", font=("Segoe UI", 7, "bold"))

        # Thin ring right at the edge of the moving disk.
        self.create_oval(cx - r, cy - r, cx + r, cy + r, outline=COL_BORDER, width=1)

        # ---- Fixed bezel: black rim, roll-index ticks, top pointer ----
        # None of this rotates or shifts -- it's the static case around
        # the moving ball, same as a real attitude indicator.
        self.create_oval(cx - r - 4, cy - r - 4, cx + r + 4, cy + r + 4,
                          outline="#0a0a0a", width=4)
        for tick_deg in (0, 10, 20, 30, 45, -10, -20, -30, -45):
            a = math.radians(tick_deg - 90)
            x1 = cx + (r + 1) * math.cos(a)
            y1 = cy + (r + 1) * math.sin(a)
            x2 = cx + (r + 4) * math.cos(a)
            y2 = cy + (r + 4) * math.sin(a)
            self.create_line(x1, y1, x2, y2, fill="white", width=2 if tick_deg == 0 else 1)
        for dot_deg in (-25, 25):
            a = math.radians(dot_deg - 90)
            dx = cx + (r + 2.5) * math.cos(a)
            dy = cy + (r + 2.5) * math.sin(a)
            self.create_oval(dx - 1.5, dy - 1.5, dx + 1.5, dy + 1.5, fill="white", outline="")

        tip_x = cx
        apex_y = cy - r - 3
        base_y = cy - r - 9
        self.create_polygon(tip_x, apex_y, tip_x - 5, base_y, tip_x + 5, base_y,
                             fill="white", outline="#0a0a0a")

        # ---- Fixed yellow aircraft symbol at the center ----
        self.create_line(cx - r * 0.32, cy, cx - r * 0.10, cy, fill="#ffd400", width=3)
        self.create_line(cx + r * 0.10, cy, cx + r * 0.32, cy, fill="#ffd400", width=3)
        self.create_line(cx, cy - r * 0.10, cx, cy - r * 0.34, fill="#ffd400", width=2)
        self.create_polygon(cx, cy - r * 0.42, cx - 4, cy - r * 0.32, cx + 4, cy - r * 0.32,
                             fill="", outline="#ffd400", width=1)
        self.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#111113", outline="#ffd400", width=2)
        self.create_oval(cx - 2, cy - 2, cx + 2, cy + 2, fill="#ffd400", outline="")
        self._finish_draw()


##############################################################################
# SECTION: ui_helpers.py   (was gcs/app/layout/ui_helpers.py)
##############################################################################
class UiHelpersMixin:
    """Shared UI building helpers: buttons, section headers, instrument cards."""

    def _section_header(self, parent, text, first=False):
        """Small dim caption with a hairline running to the right edge."""
        row = tk.Frame(parent, bg=COL_GLASS)
        row.pack(fill="x", pady=((0, 4) if first else (10, 4)))
        tk.Label(row, text=text, font=("Segoe UI", 8, "bold"), fg=COL_TEXT_DIM,
                 bg=COL_GLASS).pack(side="left")
        tk.Frame(row, bg=COL_BORDER, height=1).pack(side="left", fill="x", expand=True,
                                                    padx=(8, 0), pady=(2, 0))
        return row

    _BTN_STYLES = {
        "default": dict(bg=COL_BTN, hover=COL_BTN_HOVER, border=COL_BORDER,
                        hborder=COL_GLASS_BORDER, fg=COL_TEXT),
        "primary": dict(bg=COL_PRIMARY, hover=COL_PRIMARY_HOVER, border=COL_PRIMARY_BORDER,
                        hborder=COL_VALUE, fg="#d6f5e8"),
        "danger": dict(bg=COL_BTN, hover=COL_DANGER_HOVER, border=COL_BORDER,
                       hborder=COL_DANGER_BORDER, fg=COL_TEXT),
        "active": dict(bg=COL_ACTIVE, hover=COL_ACTIVE_HOVER, border=COL_WARN,
                       hborder=COL_WARN, fg="#ffd98a"),
    }

    def _style_ui_button(self, btn, kind):
        st = self._BTN_STYLES[kind]
        btn._style = st
        btn.config(bg=st["bg"], fg=st["fg"], activebackground=st["hover"],
                   activeforeground=st["fg"], highlightbackground=st["border"],
                   highlightcolor=st["border"])

    def _ui_button(self, parent, text, command, kind="default"):
        """Flat panel button with a consistent hover/border treatment."""
        btn = tk.Button(parent, text=text, command=command, font=("Segoe UI", 9, "bold"),
                        relief="flat", bd=0, height=1, highlightthickness=1, cursor="hand2")
        self._style_ui_button(btn, kind)
        btn.bind("<Enter>", lambda e: btn.config(bg=btn._style["hover"],
                                                 highlightbackground=btn._style["hborder"]))
        btn.bind("<Leave>", lambda e: btn.config(bg=btn._style["bg"],
                                                 highlightbackground=btn._style["border"]))
        return btn

    def _make_button(self, parent, text, command, bold=False, small=False, height=None, width=None, edge=None):
        if height is None:
            height = 1 if small else 2
        if width is None:
            width = 14
        if edge in ("left", "right"):
            # 55: end-of-bar buttons -- only the outer corners are rounded.
            return EdgeButton(
                parent, text, command, edge=edge,
                bg=COL_BTN_ACCENT if bold else COL_BTN,
                hover_bg=COL_BTN_ACCENT_HOVER if bold else COL_BTN_HOVER,
                font=("Segoe UI", 9 if small else 11, "bold" if bold else "normal"),
                width=width, height=height)
        btn = tk.Button(
            parent, text=text, command=command,
            bg=COL_BTN_ACCENT if bold else COL_BTN,
            fg=COL_TEXT,
            activebackground=COL_BTN_ACCENT_HOVER if bold else COL_BTN_HOVER,
            activeforeground=COL_TEXT,
            font=("Segoe UI", 9 if small else 11, "bold" if bold else "normal"),
            relief="flat", bd=0,
            width=width,
            height=height,
            highlightthickness=1,
            highlightbackground=COL_BORDER,
            highlightcolor=COL_BORDER,
            cursor="hand2",
        )
        hover_color = COL_BTN_ACCENT_HOVER if bold else COL_BTN_HOVER
        base_color = COL_BTN_ACCENT if bold else COL_BTN
        btn.bind("<Enter>", lambda e: btn.config(bg=hover_color))
        btn.bind("<Leave>", lambda e: btn.config(bg=base_color))
        return btn

    def _draw_rounded_rect(self, canvas, x1, y1, x2, y2, radius=6, **kwargs):
        """Draws a rounded-corner rectangle on a Canvas using a single
        smoothed polygon (tkinter's Canvas has no native rounded-rect
        primitive). Returns the item id, which supports the same
        itemconfig(..., fill=...) calls a plain create_rectangle id
        would -- so callers that later recolor a swatch don't need to
        change anything."""
        points = [
            x1 + radius, y1,
            x2 - radius, y1,
            x2, y1,
            x2, y1 + radius,
            x2, y2 - radius,
            x2, y2,
            x2 - radius, y2,
            x1 + radius, y2,
            x1, y2,
            x1, y2 - radius,
            x1, y1 + radius,
            x1, y1,
        ]
        return canvas.create_polygon(points, smooth=True, **kwargs)

    def _instrument_card(self, parent, title):
        """Inset dark card with a small caption + hairline; returns the card."""
        card = tk.Frame(parent, bg=COL_CARD, highlightthickness=1,
                        highlightbackground=COL_BORDER, highlightcolor=COL_BORDER)
        card.pack(fill="x", pady=(0, 8))
        tk.Label(card, text=title, font=("Segoe UI", 8, "bold"), fg=COL_TEXT_DIM,
                 bg=COL_CARD).pack(pady=(7, 4))
        tk.Frame(card, bg=COL_BORDER, height=1).pack(fill="x", padx=10)
        return card



from swarm_agent import (
    FORMATION_SHAPES,
    formation_slots,
)




##############################################################################
# SECTION: mav_commands.py   (was gcs/link/mav_commands.py)
##############################################################################
class MavCommandsMixin:
    """MAVLink commands: arm / disarm / mode / takeoff / speed / params."""

    # ---------------- Commands ----------------
    # Every command below accepts an optional target_system override, used
    # by the swarm/DRONES panel to address a specific vehicle by its
    # MAVLink system ID when multiple drones share this one connection
    # (e.g. --count 3 --auto-sysid). When not given, falls back to
    # self.master.target_system (pymavlink's auto-detected default target
    # -- normally the first vehicle seen), which is exactly the old,
    # single-drone behavior, unchanged.

    def arm(self, target_system=None):
        if not self.master:
            self.data_queue.put(("LOG", "!!! Not connected"))
            return
        ts = target_system if target_system is not None else self.master.target_system
        self.data_queue.put(("LOG", f">>> arm throttle (sysid {ts})"))
        self.master.mav.command_long_send(
            ts, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )

    def disarm(self, target_system=None):
        if not self.master:
            self.data_queue.put(("LOG", "!!! Not connected"))
            return
        ts = target_system if target_system is not None else self.master.target_system
        self.data_queue.put(("LOG", f">>> disarm (sysid {ts})"))
        self.master.mav.command_long_send(
            ts, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def set_mode(self, mode_name, target_system=None):
        if not self.master:
            self.data_queue.put(("LOG", "!!! Not connected"))
            return
        mode_name = mode_name.upper()
        mode_mapping = self.master.mode_mapping()
        if mode_mapping is None or mode_name not in mode_mapping:
            self.data_queue.put(("LOG", f"!!! Unknown mode: {mode_name}"))
            return
        ts = target_system if target_system is not None else self.master.target_system
        self.data_queue.put(("LOG", f">>> mode {mode_name.lower()} (sysid {ts})"))
        mode_id = mode_mapping[mode_name]
        # Bypass master.set_mode() here since that convenience wrapper
        # always targets self.master.target_system with no override --
        # send the underlying SET_MODE message directly instead, so we
        # can address a specific vehicle.
        self.master.mav.set_mode_send(
            ts, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
        )

    def takeoff(self, altitude, target_system=None):
        if not self.master:
            self.data_queue.put(("LOG", "!!! Not connected"))
            return
        ts = target_system if target_system is not None else self.master.target_system
        self.data_queue.put(("LOG", f">>> takeoff {altitude} (sysid {ts})"))
        self.master.mav.command_long_send(
            ts, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, float(altitude)
        )

    def send_position_target(self, target_system, lat, lon, alt, vn=None, ve=None, yaw=None):
        """GUIDED-mode setpoint (SET_POSITION_TARGET_GLOBAL_INT, alt relative to
        home). Position always; velocity feed-forward (m/s, north/east) and
        yaw (rad, 0 = north) are optional. Not logged -- it is a stream."""
        if not self.master:
            return
        mask = 448 | 512 | 2048            # ignore accel, force, yaw-rate
        if vn is None or ve is None:
            mask |= 56                     # ignore velocity
            vn = ve = 0.0
        if yaw is None:
            mask |= 1024                   # ignore yaw
            yaw = 0.0
        try:
            self.master.mav.set_position_target_global_int_send(
                int(time.time() * 1000) & 0xFFFFFFFF, target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask,
                int(lat * 1e7), int(lon * 1e7), float(alt),
                float(vn), float(ve), 0.0, 0.0, 0.0, 0.0, float(yaw), 0.0)
        except Exception as e:
            self.data_queue.put(("LOG", f"!!! setpoint send failed (sysid {target_system}): {e}"))

    def abort_mission(self, target_system=None):
        """Stops the drone exactly where it currently is, using BRAKE mode
        (an active hold, not a return-to-launch or landing)."""
        if not self.master:
            self.data_queue.put(("LOG", "!!! Not connected"))
            return
        self.data_queue.put(("LOG", ">>> ABORT MISSION -- switching to BRAKE (hold position)"))
        self.set_mode("BRAKE", target_system=target_system)

    def set_speed(self, speed_kmh, target_system=None):
        if not self.master:
            self.data_queue.put(("LOG", "!!! Not connected"))
            return
        ts = target_system if target_system is not None else self.master.target_system
        speed_ms = float(speed_kmh) / 3.6
        self.data_queue.put(("LOG", f">>> set speed {speed_kmh} km/h ({speed_ms:.2f} m/s) (sysid {ts})"))
        self.master.mav.command_long_send(
            ts, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1,          # speed type: 1 = ground speed
            speed_ms,   # target speed in m/s
            -1,         # throttle: -1 = no change
            0, 0, 0, 0
        )

    def set_param(self, name, value, target_system=None):
        if not self.master:
            self.data_queue.put(("LOG", "!!! Not connected"))
            return
        ts = target_system if target_system is not None else self.master.target_system
        self.data_queue.put(("LOG", f">>> param set {name} {value} (sysid {ts})"))
        self.master.mav.param_set_send(
            ts, self.master.target_component,
            name.encode("utf-8"), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    def read_param(self, name):
        if not self.master:
            self.data_queue.put(("LOG", "!!! Not connected"))
            return
        self.data_queue.put(("LOG", f">>> param request {name}"))
        self.master.mav.param_request_read_send(
            self.master.target_system, self.master.target_component,
            name.encode("utf-8"), -1
        )


##############################################################################
# SECTION: mav_mission.py   (was gcs/link/mav_mission.py)
##############################################################################
class MavMissionMixin:
    """Mission file load + upload to the autopilot."""

    def _drain_mission_queue(self):
        """Discards any stale mission messages left over from a previous
        (e.g. timed-out) upload attempt, so they can't be mistaken for
        replies to a new one."""
        while True:
            try:
                self.mission_queue.get_nowait()
            except queue.Empty:
                break

    def _transfer_mission(self, ts, count, send_item):
        """Runs one mission upload to sysid `ts` and returns the
        MISSION_ACK result (0 = accepted) or None if it never completed.
        Only replies that really come from `ts` are used, requests are
        answered by whatever seq the drone asks for (repeats included), and
        the transfer only ends on that drone's MISSION_ACK -- not after a
        fixed number of requests, which let one stray/duplicate request
        eat the final ACK slot and leave the drone with an incomplete
        mission. Retried up to 3 times. Caller holds _mission_transfer_lock."""
        master = self.master
        for attempt in range(3):
            self._drain_mission_queue()
            master.mav.mission_count_send(ts, master.target_component, count)
            while True:
                try:
                    msg = self.mission_queue.get(timeout=5)
                except queue.Empty:
                    break                       # no reply -> retry whole transfer
                if msg.get_srcSystem() != ts:
                    continue                    # another drone's traffic
                kind = msg.get_type()
                if kind == "MISSION_ACK":
                    return msg.type
                if 0 <= msg.seq < count:
                    send_item(msg.seq)
            self.data_queue.put(("LOG", f"!!! Mission upload to sysid {ts} stalled - retrying ({attempt + 1}/3)"))
        return None

    # ---------------- Mission from file ----------------

    def load_mission(self, filepath, target_system=None):
        """target_system lets the DRONES panel upload the same mission
        file to several ticked drones -- one thread per drone, each
        addressing its own sysid; the transfers themselves still run one
        at a time (see _mission_transfer_lock)."""
        threading.Thread(target=self._load_mission_worker,
                          args=(filepath, target_system), daemon=True).start()

    def _load_mission_worker(self, filepath, target_system=None):
        if not self.master:
            self.data_queue.put(("LOG", "!!! Not connected"))
            return
        try:
            with open(filepath, "r") as f:
                lines = [l.strip() for l in f if l.strip()]
        except Exception as e:
            self.data_queue.put(("LOG", f"!!! Could not open mission file: {e}"))
            return

        if not lines or not lines[0].startswith("QGC WPL"):
            self.data_queue.put(("LOG", "!!! Invalid or missing 'QGC WPL 110' header"))
            return

        waypoints = []
        for line in lines[1:]:
            parts = line.split("\t")
            if len(parts) < 12:
                parts = line.split()
            if len(parts) < 12:
                continue
            waypoints.append(parts)

        if not waypoints:
            self.data_queue.put(("LOG", "!!! No waypoints parsed from file"))
            return

        coords = []
        for p in waypoints:
            try:
                lat, lon = float(p[8]), float(p[9])
                if lat != 0.0 or lon != 0.0:
                    coords.append((lat, lon))
            except (ValueError, IndexError):
                continue
        self.data_queue.put(("MISSION_PATH", coords))
        self.data_queue.put(("MISSION_TOTAL", len(waypoints)))

        master = self.master
        ts = target_system if target_system is not None else master.target_system
        self.data_queue.put(("LOG", f">>> Uploading {len(waypoints)} waypoints to sysid {ts}..."))

        # See the comment on _mission_transfer_lock: without this, loading
        # the same file onto two ticked drones at once would let their
        # replies get crossed on the shared mission_queue.
        def send_item(seq):
            p = waypoints[seq]
            master.mav.mission_item_int_send(
                ts, master.target_component, seq,
                int(p[2]), int(p[3]),
                int(p[1]) if seq == 0 else 0,
                1,
                float(p[4]), float(p[5]), float(p[6]), float(p[7]),
                int(float(p[8]) * 1e7), int(float(p[9]) * 1e7), float(p[10])
            )

        with self._mission_transfer_lock:
            result = self._transfer_mission(ts, len(waypoints), send_item)
        if result is None:
            self.data_queue.put(("LOG", f"!!! Mission upload FAILED (sysid {ts}) - no MISSION_ACK"))
        else:
            self.data_queue.put(("LOG", f"Mission upload result (sysid {ts}): {result}"))

    # ---------------- Mission from manually marked points ----------------

    def upload_manual_mission(self, coords, altitude, target_system=None):
        threading.Thread(target=self._upload_manual_worker,
                          args=(coords, altitude, target_system), daemon=True).start()

    def _upload_manual_worker(self, coords, altitude, target_system=None):
        if not coords or not self.master:
            self.data_queue.put(("LOG", "!!! Not connected"))
            return

        master = self.master
        ts = target_system if target_system is not None else master.target_system

        home_lat, home_lon = coords[0]
        items = [
            (0, 16, 1, home_lat, home_lon, 0),
            (3, 22, 0, home_lat, home_lon, altitude),
        ]
        for lat, lon in coords:
            items.append((3, 16, 0, lat, lon, altitude))
        items.append((0, 20, 0, 0.0, 0.0, 0))

        self.data_queue.put(("MISSION_TOTAL", len(items)))
        self.data_queue.put(("LOG", f">>> Uploading {len(items)} mission items from marked waypoints "
                                     f"(target sysid {ts})..."))

        # See the comment on _mission_transfer_lock: this is what makes it
        # safe to call upload_manual_mission() for several drones back to
        # back (e.g. FLY MARKED WPS with 2 drones ticked) -- each one's
        # transfer waits its turn instead of the two racing on the shared
        # mission_queue and potentially swapping waypoints between drones.
        def send_item(seq):
            frame, cmd, current, lat, lon, alt = items[seq]
            master.mav.mission_item_int_send(
                ts, master.target_component, seq,
                frame, cmd, current, 1,
                0, 0, 0, 0,
                int(lat * 1e7), int(lon * 1e7), float(alt)
            )

        with self._mission_transfer_lock:
            result = self._transfer_mission(ts, len(items), send_item)
        if result is None:
            self.data_queue.put(("LOG", f"!!! Mission upload FAILED (sysid {ts}) - no MISSION_ACK"))
        else:
            self.data_queue.put(("LOG", f"Mission upload result (sysid {ts}): {result}"))


##############################################################################
# SECTION: mavlink.py   (was gcs/link/mavlink.py)
##############################################################################
class MAVLink(MavCommandsMixin, MavMissionMixin):
    """Owns the MAVLink connection and can be reconnected on the fly to
    switch between simulation (UDP) and a real telemetry radio (serial).
    Reading happens on a background thread per-connection; sending is
    called directly from the UI thread."""

    def __init__(self, data_queue):
        self.data_queue = data_queue
        self.master = None
        self.connected = False
        self.mode = None  # "SIM" or "REAL"
        self._generation = 0
        self._lock = threading.Lock()
        # MISSION_REQUEST(_INT)/MISSION_ACK messages get routed here by the
        # single listener thread, so mission upload code (running on its
        # own thread) never calls master.recv_match() itself -- see the
        # comment in _listen_worker for why that matters.
        self.mission_queue = queue.Queue()
        # sysid -> time.time() of the last MAVLink message received from
        # that vehicle, of ANY type. This is what link-loss detection
        # checks against -- it's updated on every message below, so a
        # drone stops looking "LIVE" the moment its radio actually goes
        # quiet, not just when a specific message type is missing.
        self.link_last_seen = {}
        # The messages routed into mission_queue above carry no reliable
        # indication of which drone's transfer they belong to, so if two
        # mission uploads (to two different sysids) are ever in flight at
        # the same time, one upload's worker thread can easily consume a
        # reply meant for the other -- delivering the wrong waypoints to
        # the wrong drone, or leaving one transfer hanging until it times
        # out. This lock forces every mission transfer (from a file or
        # from marked waypoints, to any sysid) to happen one at a time;
        # concurrent requests just queue up and run in order instead of
        # racing on the shared reply queue.
        self._mission_transfer_lock = threading.Lock()

    def _request_data_streams(self, master):
        """This was the actual cause of the ATTITUDE gauge sitting
        frozen near 0 deg while Mission Planner's HUD tracked real
        tilt correctly: ArduPilot only streams a message at whatever
        rate its SRx_* parameters for THIS link happen to already be
        set to, and this dashboard never asked for anything -- it just
        passively took whatever the autopilot felt like sending by
        default on a fresh connection (which can be 0 Hz for ATTITUDE
        on a serial/telemetry port that's never had a GCS request a
        rate on it before). Mission Planner asks for these rates every
        time it connects, which is why its HUD updated live and ours
        didn't, even talking to the same flight controller.
        Called once, right after the heartbeat, for both SIM and REAL.
        Uses the modern per-message MAV_CMD_SET_MESSAGE_INTERVAL for
        the messages this dashboard actually reads, and falls back to
        the older REQUEST_DATA_STREAM (still honored by ArduPilot) as
        a catch-all for anything that command doesn't cover. Silently
        best-effort -- a stream request failing shouldn't take down an
        otherwise-working connection."""
        try:
            master.mav.request_data_stream_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
            )
        except Exception:
            pass
        rates_hz = {
            "ATTITUDE": 10, "VFR_HUD": 10, "GLOBAL_POSITION_INT": 5,
            "SYS_STATUS": 2, "GPS_RAW_INT": 2, "RADIO_STATUS": 2,
            "EKF_STATUS_REPORT": 2, "SCALED_IMU2": 10, "RAW_IMU": 10,
        }
        for msg_name, hz in rates_hz.items():
            try:
                msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{msg_name}")
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msg_id, int(1_000_000 / hz), 0, 0, 0, 0, 0
                )
            except Exception:
                pass

    def start_simulation(self, connection_string=SIM_CONNECTION_STRING):
        self._connect("SIM", connection_string, baud=None)

    def start_real(self, port, baud=REAL_BAUD_RATE):
        self._connect("REAL", port, baud=baud)

    def probe_real(self, port, baud):
        """Tries to open `port` at `baud` and wait for a heartbeat,
        entirely independently of whatever this class is currently
        connected to -- self.master/self.mode/self.connected/
        self._generation are never touched here. That's what lets the
        manual CONNECT button's "Trying to connect..." popup test a
        port without ever disturbing an already-working SIM link: if
        the probe fails, nothing about the live connection changed, so
        SIM just keeps running exactly as it was. Only once this
        reports success (via a MANUAL_PROBE_RESULT message) does the
        dashboard call start_real() to actually switch the live
        connection over."""
        threading.Thread(target=self._probe_worker, args=(port, baud), daemon=True).start()

    def _probe_worker(self, port, baud):
        master = None
        try:
            master = mavutil.mavlink_connection(port, baud=baud)
            hb = master.wait_heartbeat(timeout=6)
        except Exception as e:
            self.data_queue.put(("MANUAL_PROBE_RESULT",
                                  {"ok": False, "port": port, "baud": baud, "error": str(e)}))
            return
        finally:
            if master is not None:
                try:
                    master.close()
                except Exception:
                    pass
        if hb is None:
            self.data_queue.put(("MANUAL_PROBE_RESULT",
                                  {"ok": False, "port": port, "baud": baud, "error": "no heartbeat (timeout)"}))
        else:
            self.data_queue.put(("MANUAL_PROBE_RESULT", {"ok": True, "port": port, "baud": baud}))

    def _connect(self, mode, connection_string, baud):
        with self._lock:
            self._generation += 1
            gen = self._generation
            if self.master:
                try:
                    self.master.close()
                except Exception:
                    pass
                self.master = None
            self.connected = False
            self.mode = mode

        self.data_queue.put(("LINK_STATUS", {"connected": False, "mode": mode}))
        self.data_queue.put(("LOG", f">>> Connecting ({display_mode(mode)}) to {connection_string} ..."))
        threading.Thread(target=self._listen_worker,
                          args=(gen, connection_string, baud, mode), daemon=True).start()

    def _listen_worker(self, gen, connection_string, baud, mode):
        try:
            if baud:
                master = mavutil.mavlink_connection(connection_string, baud=baud)
            else:
                master = mavutil.mavlink_connection(connection_string)
            hb = master.wait_heartbeat(timeout=10)
        except Exception as e:
            self.data_queue.put(("LOG", f"!!! Connection failed ({display_mode(mode)}): {e}"))
            self.data_queue.put(("LINK_STATUS", {"connected": False, "mode": mode}))
            return

        if hb is None:
            self.data_queue.put(("LOG", f"!!! No heartbeat received on {connection_string} (timeout)"))
            self.data_queue.put(("LINK_STATUS", {"connected": False, "mode": mode}))
            try:
                master.close()
            except Exception:
                pass
            return

        with self._lock:
            if gen != self._generation:
                # a newer connection attempt has already superseded this one
                try:
                    master.close()
                except Exception:
                    pass
                return
            self.master = master
            self.connected = True

        self.data_queue.put(("LOG", f"Heartbeat received ({display_mode(mode)}). Connected."))
        self.data_queue.put(("LINK_STATUS", {"connected": True, "mode": mode}))
        self._request_data_streams(master)

        threading.Thread(target=self._heartbeat_loop, args=(gen, master), daemon=True).start()

        # SiK telemetry radios inject their own RADIO_STATUS messages
        # using a hardcoded pseudo system id (conventionally 51) that
        # identifies the RADIO MODEM itself, not any vehicle -- it has
        # nothing to do with the drone's real sysid. Blindly trusting
        # msg.get_srcSystem() for RADIO_STATUS (as this used to) creates
        # a phantom "Drone B (sysid 51)" in the Signal Strength panel
        # showing what is actually Drone A's own radio link. Instead we
        # track whichever real vehicle sysid was last seen from an
        # actual per-vehicle message, and attribute RADIO_STATUS to
        # that -- it's this link's radio, so it belongs to whichever
        # drone this link is talking to.
        last_vehicle_sysid = 1

        while True:
            with self._lock:
                if gen != self._generation:
                    return
            msg = master.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            msg_type = msg.get_type()

            # Record link activity for THIS vehicle regardless of what
            # kind of message it sent -- link-loss detection just cares
            # that something arrived recently, not what.
            src_sysid = msg.get_srcSystem()
            if src_sysid:
                self.link_last_seen[src_sysid] = time.time()
            if msg_type != "RADIO_STATUS" and src_sysid:
                last_vehicle_sysid = src_sysid

            if msg_type == "HEARTBEAT":
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                fmode = mavutil.mode_string_v10(msg)
                self.data_queue.put(("HEARTBEAT", {
                    "armed": armed, "mode": fmode, "sysid": msg.get_srcSystem()
                }))

            elif msg_type == "GLOBAL_POSITION_INT":
                self.data_queue.put(("POSITION", {
                    "lat": msg.lat / 1e7,
                    "lon": msg.lon / 1e7,
                    "alt": msg.relative_alt / 1000.0,
                    "sysid": msg.get_srcSystem(),
                }))

            elif msg_type == "VFR_HUD":
                self.data_queue.put(("VFR", {
                    "airspeed": msg.airspeed,
                    "groundspeed": msg.groundspeed,
                    "heading": msg.heading,
                    "climb": msg.climb,
                    "sysid": msg.get_srcSystem(),
                }))

            elif msg_type == "ATTITUDE":
                self.data_queue.put(("ATTITUDE", {
                    "roll": math.degrees(msg.roll),
                    "pitch": math.degrees(msg.pitch),
                    "yaw": math.degrees(msg.yaw),
                    "sysid": msg.get_srcSystem(),
                }))

            elif msg_type in ("SCALED_IMU2", "SCALED_IMU", "RAW_IMU"):
                # VISUAL ODO: IMU of the real flight controller (mg -> m/s^2, mrad/s -> deg/s)
                g_mg = 9.80665 / 1000.0
                r_mr = 180.0 / math.pi / 1000.0
                self.data_queue.put(("IMU", {
                    "accel": (msg.xacc * g_mg, msg.yacc * g_mg, msg.zacc * g_mg),
                    "gyro": (msg.xgyro * r_mr, msg.ygyro * r_mr, msg.zgyro * r_mr),
                    "sysid": msg.get_srcSystem(),
                }))

            elif msg_type == "PARAM_VALUE":
                name = msg.param_id
                if isinstance(name, bytes):
                    name = name.decode("utf-8", errors="ignore")
                name = name.rstrip("\x00")
                self.data_queue.put(("PARAM_VALUE", {
                    "name": name,
                    "value": msg.param_value,
                    "index": msg.param_index,
                    "count": msg.param_count,
                }))

            elif msg_type == "SYS_STATUS":
                self.data_queue.put(("BATTERY", {
                    "voltage": msg.voltage_battery / 1000.0,
                    "current": msg.current_battery / 100.0,
                    "remaining": msg.battery_remaining,
                    "sysid": msg.get_srcSystem(),
                }))

            elif msg_type == "STATUSTEXT":
                self.data_queue.put(("LOG", f"AP: {msg.text}"))
                # Forwarded separately (with sysid) so the UI can pick out
                # "PreArm: ..." failures per-drone without re-parsing the
                # scrolling log.
                self.data_queue.put(("STATUSTEXT", {
                    "text": msg.text, "sysid": msg.get_srcSystem()
                }))

            elif msg_type == "COMMAND_ACK":
                cmd_enum = mavutil.mavlink.enums["MAV_CMD"].get(msg.command, None)
                cmd_name = cmd_enum.name if cmd_enum else str(msg.command)
                res_enum = mavutil.mavlink.enums["MAV_RESULT"].get(msg.result, None)
                res_name = res_enum.name if res_enum else str(msg.result)
                self.data_queue.put(("LOG", f"Got COMMAND_ACK: {cmd_name}: {res_name}"))

            elif msg_type == "MISSION_CURRENT":
                self.data_queue.put(("MISSION_CURRENT", {"seq": msg.seq}))

            elif msg_type in ("MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"):
                # These belong to a mission upload in progress. They must
                # NOT be read by whichever upload thread called
                # master.recv_match() directly -- this loop is the ONLY
                # thing allowed to read from the shared socket (pymavlink
                # connections aren't safe for two threads to recv_match()
                # on at once), so forward them to a queue the upload
                # threads actually wait on instead.
                self.mission_queue.put(msg)

            elif msg_type == "GPS_RAW_INT":
                self.data_queue.put(("GPS_RAW", {
                    "fix_type": msg.fix_type,
                    "satellites_visible": msg.satellites_visible,
                }))

            elif msg_type == "EKF_STATUS_REPORT":
                # This is what actually tells you whether the autopilot
                # TRUSTS its own position/velocity estimate -- separate
                # from GPS just having a fix. ArduPilot's own EKF health
                # check flags variance > ~0.8 as degraded, so we use the
                # same threshold here rather than inventing our own.
                flags = msg.flags
                bad = []
                if msg.velocity_variance > 0.8:
                    bad.append("velocity")
                if msg.pos_horiz_variance > 0.8:
                    bad.append("pos horiz")
                if msg.pos_vert_variance > 0.8:
                    bad.append("pos vert")
                if msg.compass_variance > 0.8:
                    bad.append("compass")
                if msg.terrain_alt_variance > 0.8:
                    bad.append("terrain")
                # Also flag it if ArduPilot itself says it doesn't trust
                # horizontal position/velocity, even if variance looks ok
                # (flags can drop out a beat before variance climbs).
                ekf_flags_ok = bool(
                    flags & mavutil.mavlink.EKF_POS_HORIZ_ABS and
                    flags & mavutil.mavlink.EKF_VELOCITY_HORIZ
                )
                self.data_queue.put(("EKF_STATUS", {
                    "sysid": src_sysid or last_vehicle_sysid,
                    "bad": bad,
                    "flags_ok": ekf_flags_ok,
                }))

            elif msg_type == "RADIO_STATUS":
                # Only ever arrives over a real SiK telemetry radio (REAL/
                # TELE mode) -- SITL has no physical radio, so this stays
                # silent during simulation. rssi/remrssi are raw 0-255;
                # converted to 0-100% on the UI side. Deliberately NOT
                # msg.get_srcSystem() here -- see the comment above the
                # while-loop for why that's the radio modem's own id
                # (51), not a real vehicle.
                self.data_queue.put(("RADIO_STATUS", {
                    "sysid": last_vehicle_sysid,
                    "rssi": msg.rssi,
                    "remrssi": msg.remrssi,
                }))

    def _heartbeat_loop(self, gen, master):
        """Sends a GCS heartbeat once per second, exactly like MAVProxy
        does. Without this, ArduPilot's GCS failsafe (FS_GCS_ENABLE) can
        block arming or trigger a failsafe, since the autopilot sees no
        ground station actively present on the link."""
        while True:
            with self._lock:
                if gen != self._generation or not self.connected:
                    return
            try:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0,
                    mavutil.mavlink.MAV_STATE_ACTIVE
                )
            except Exception:
                return
            time.sleep(1)

    def stop(self):
        with self._lock:
            self._generation += 1
            self.connected = False
            if self.master:
                try:
                    self.master.close()
                except Exception:
                    pass
                self.master = None


##############################################################################
# SECTION: serial_watcher.py   (was gcs/link/serial_watcher.py)
##############################################################################
class SerialWatcher(threading.Thread):
    """Polls USB serial ports and reports when a likely telemetry radio
    appears or disappears."""

    def __init__(self, data_queue, poll_interval=SERIAL_POLL_INTERVAL_S):
        super().__init__(daemon=True)
        self.data_queue = data_queue
        self.poll_interval = poll_interval
        self._stop = False
        self._last_port = None

    def run(self):
        while not self._stop:
            port = self._find_telemetry_port()
            if port != self._last_port:
                self._last_port = port
                self.data_queue.put(("SERIAL_STATUS", {"port": port}))
            time.sleep(self.poll_interval)

    def _find_telemetry_port(self):
        for p in list_ports.comports():
            if any(hint in p.device for hint in TELEMETRY_PORT_HINTS):
                return p.device
        return None

    def stop(self):
        self._stop = True


##############################################################################
# SECTION: connection.py   (was gcs/app/connection/connection.py)
##############################################################################
class ConnectionMixin:
    """Serial port menu, connect/disconnect, SIM<->TELE mode logic, popups."""

    def _refresh_port_menu(self):
        """Rebuilds the port dropdown's contents right before it opens, so
        it always shows whatever's currently plugged in rather than a
        stale snapshot from when the dashboard started. "Auto" is always
        the first entry -- picking it means "use whatever telemetry
        radio SerialWatcher already auto-detected" instead of a specific
        manually-chosen port (see _on_connect_toggle).

        Only real USB serial adapters are listed -- filtered with the
        same TELEMETRY_PORT_HINTS ("ttyUSB", "ttyACM", "COM") already
        used elsewhere to auto-detect a plugged-in radio. Without this
        filter, Linux also reports its 32 built-in-but-nonexistent
        legacy motherboard ports (/dev/ttyS0 .. /dev/ttyS31) as if they
        were real devices -- they always show up whether or not
        anything is plugged in, and trying to open one fails immediately
        ("Could not configure port: Input/output error"). Filtering to
        the hints means Linux only shows genuine ttyUSB*/ttyACM* USB-
        serial adapters, and on Windows -- where every listed port is
        already a real one, and every device name already contains
        "COM" -- this filter changes nothing: plugging something new in
        still shows up as a fresh COM7/COM8/COM13/etc. the next time
        this menu is opened, exactly as reported by the OS."""
        self.port_menu.delete(0, "end")
        self.port_menu.add_command(label="Auto", command=lambda: self._set_port_selection("Auto", "Auto"))
        ports = [p for p in list_ports.comports()
                 if any(hint in p.device for hint in TELEMETRY_PORT_HINTS)]
        if not ports:
            return
        for p in ports:
            label = self._clean_port_label(p.device)
            self.port_menu.add_command(label=label, command=lambda d=p.device, lbl=label: self._set_port_selection(d, lbl))

    def _clean_port_label(self, device):
        """Shortens a device path for display only -- strips Linux's
        '/dev/' prefix so the dropdown reads 'ttyUSB0'/'ttyS14' instead
        of '/dev/ttyUSB0'/'/dev/ttyS14'. On Windows, pyserial already
        reports plain names like 'COM8'/'COM15' with nothing to strip,
        so this is a no-op there -- there's no way to make a Linux tty
        device show up as "COM8", since that naming is decided by the
        OS itself, not by this app. The full original device path is
        still what actually gets connected to; only the on-screen label
        is shortened (see _set_port_selection)."""
        return device[len("/dev/"):] if device.startswith("/dev/") else device

    def _set_port_selection(self, device, label):
        self.port_var.set(device)
        self.port_display_var.set(label)

    def _on_connect_toggle(self):
        """Mission-Planner-style manual Connect/Disconnect.

        Disconnect: only reachable once we actually have a confirmed
        manual connection (self.manual_conn_active), so this always
        means tearing down a real, live link -- never a link that's
        merely "trying".

        Connect:
          - Refuses outright while Mode is SIM ("SIM mode never opens a
            real serial port" -- per request, SIM should never even
            attempt a telemetry connection).
          - Otherwise probes the selected port/baud in the background
            (see MAVLink.probe_real) and shows a small "Trying to
            connect..." popup with a loading bar while that happens.
            The live SIM/REAL connection is NOT touched during the
            probe, so the SIM indicator stays exactly "SIM" the whole
            time instead of flipping to "TELE (connecting...)" -- the
            bug where that text could get stuck forever, since a failed
            attempt never reset it. Only once the probe reports success
            (see the MANUAL_PROBE_RESULT handler in _poll_queue) does
            this actually call start_real() and flip the button to
            DISCONNECT -- so the button only ever says DISCONNECT when
            there's a confirmed telemetry connection behind it."""
        if self.manual_conn_active:
            self.mav.stop()
            self.manual_conn_active = False
            self.connect_btn.config(text="CONNECT")
            self._append_log(">>> Disconnected")
            # Force _apply_mode_logic to actually reissue a connection
            # (e.g. back to SIM) instead of seeing "desired == last
            # target we told it about" and doing nothing -- it never
            # knew about this manual connection in the first place.
            self._current_target = None
            self._apply_mode_logic()
            return

        if self.mode_override == "SIM":
            messagebox.showinfo(
                "Currently in SIM mode",
                "SIM mode never opens a real serial port. Switch the "
                "Mode button (top right) to AUTO or TELE first, then "
                "try Connect again."
            )
            return

        port = self.port_var.get()
        if port == "Auto":
            port = self.telemetry_port
            if not port:
                messagebox.showerror(
                    "No port auto-detected",
                    "No telemetry radio detected yet -- plug one in, or pick a port manually from the dropdown."
                )
                return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            baud = REAL_BAUD_RATE

        self._append_log(f">>> Trying to connect to {port} @ {baud} baud ...")
        self._open_connecting_popup(port, baud)
        self.mav.probe_real(port, baud)

    def _open_connecting_popup(self, port, baud):
        """Small non-modal popup shown while a manual CONNECT attempt is
        probing a port in the background -- a label plus an
        indeterminate loading bar. Closed automatically (see
        _close_connecting_popup) once MANUAL_PROBE_RESULT comes back,
        whichever way it goes."""
        dialog = tk.Toplevel(self.root)
        self.connecting_dialog = dialog
        dialog.title("Connecting")
        dialog.configure(bg=COL_BTN)
        dialog.resizable(False, False)
        dialog.transient(self.root)

        wrapper = tk.Frame(dialog, bg=COL_BTN, highlightthickness=1,
                            highlightbackground=COL_BORDER)
        wrapper.pack(fill="both", expand=True)

        tk.Label(wrapper, text=f"Trying to connect to {port}",
                 font=("Segoe UI", 12, "bold"), fg=COL_TEXT, bg=COL_BTN).pack(padx=28, pady=(22, 2))
        tk.Label(wrapper, text=f"@ {baud} baud ...", font=("Segoe UI", 9),
                 fg=COL_TEXT_DIM, bg=COL_BTN).pack(padx=28, pady=(0, 14))

        style = ttk.Style()
        style.configure("Connecting.Horizontal.TProgressbar",
                         troughcolor=COL_BG, background=COL_BTN_ACCENT, bordercolor=COL_BORDER)
        bar = ttk.Progressbar(wrapper, mode="indeterminate", length=220,
                               style="Connecting.Horizontal.TProgressbar")
        bar.pack(padx=28, pady=(0, 22))
        bar.start(12)
        self._connecting_bar = bar

        dialog.update_idletasks()
        w, h = dialog.winfo_reqwidth(), dialog.winfo_reqheight()
        rx, ry = self.root.winfo_rootx(), self.root.winfo_rooty()
        rw, rh = self.root.winfo_width(), self.root.winfo_height()
        x = rx + (rw - w) // 2
        y = ry + (rh - h) // 2
        dialog.geometry(f"{w}x{h}+{x}+{y}")
        # No close button / Escape binding on purpose -- this dialog only
        # ever closes itself, once the probe result comes back.
        dialog.protocol("WM_DELETE_WINDOW", lambda: None)

    def _close_connecting_popup(self):
        bar = getattr(self, "_connecting_bar", None)
        if bar is not None:
            try:
                bar.stop()
            except Exception:
                pass
            self._connecting_bar = None
        dialog = getattr(self, "connecting_dialog", None)
        if dialog is not None and dialog.winfo_exists():
            dialog.destroy()
        self.connecting_dialog = None

    def _show_device_not_found_popup(self, port):
        """Small dialog shown after a manual CONNECT probe times out or
        errors -- confirms nothing was found on `port`, and that the
        existing SIM/AUTO connection (if any) was left untouched."""
        dialog = tk.Toplevel(self.root)
        dialog.title("Device not found")
        dialog.configure(bg=COL_BTN)
        dialog.resizable(False, False)
        dialog.transient(self.root)

        wrapper = tk.Frame(dialog, bg=COL_BTN, highlightthickness=1,
                            highlightbackground=COL_BORDER)
        wrapper.pack(fill="both", expand=True)

        tk.Label(wrapper, text="Device not found", font=("Segoe UI", 14, "bold"),
                 fg=COL_TEXT, bg=COL_BTN).pack(padx=30, pady=(22, 4))
        tk.Label(wrapper, text=f"No telemetry heartbeat on {port}.",
                 font=("Segoe UI", 10), fg=COL_TEXT_DIM, bg=COL_BTN).pack(padx=30, pady=(0, 18))

        self._make_button(wrapper, "OK", dialog.destroy, height=1, small=True, width=11).pack(pady=(0, 20))

        dialog.update_idletasks()
        w, h = dialog.winfo_reqwidth(), dialog.winfo_reqheight()
        rx, ry = self.root.winfo_rootx(), self.root.winfo_rooty()
        rw, rh = self.root.winfo_width(), self.root.winfo_height()
        x = rx + (rw - w) // 2
        y = ry + (rh - h) // 2
        dialog.geometry(f"{w}x{h}+{x}+{y}")

        dialog.grab_set()
        dialog.focus_set()
        dialog.bind("<Escape>", lambda e: dialog.destroy())

    def _cycle_mode(self):
        order = ["AUTO", "SIM", "REAL"]
        idx = order.index(self.mode_override)
        self.mode_override = order[(idx + 1) % len(order)]
        self.mode_btn.config(text=f"Mode: {display_mode(self.mode_override)}")
        self._append_log(f">>> Mode override set to {display_mode(self.mode_override)}")
        self._apply_mode_logic()

    def _apply_mode_logic(self):
        """Decides whether we should be connected to SIM or REAL, based on
        the mode override and whether a telemetry radio is currently
        plugged in, and reconnects only if the target actually changed."""
        if self.manual_conn_active:
            # The Connect/Disconnect button (top-right) is in charge of
            # the link right now -- stay out of its way until the user
            # hits Disconnect.
            return
        if self.mode_override == "SIM":
            desired = ("SIM", SIM_CONNECTION_STRING)
        elif self.mode_override == "REAL":
            if self.telemetry_port:
                desired = ("REAL", self.telemetry_port)
            else:
                self._append_log("!!! Forced TELE mode but no telemetry module detected")
                return
        else:  # AUTO
            if self.telemetry_port:
                desired = ("REAL", self.telemetry_port)
            else:
                desired = ("SIM", SIM_CONNECTION_STRING)

        if desired != self._current_target:
            self._current_target = desired
            mode, conn = desired
            if mode == "SIM":
                self.mav.start_simulation(conn)
            else:
                self.mav.start_real(conn, REAL_BAUD_RATE)

    def _update_source_indicators(self, active_mode, connected):
        """Drives the two FIXED-label 'SIM' / 'TELE' dots from a single
        place. active_mode is whichever one is actually live right now
        ("SIM" or "TELE", from the LINK_STATUS message) -- the OTHER
        label always just dims to COL_TEXT_DIM with its dot off. Only
        color changes here; text is set once in _build_status_bar and
        never touched again, which is the actual fix for "SIM changes
        to TELE" -- that was the label's *text* being overwritten on
        every reconnect, not a rendering glitch."""
        is_sim = active_mode == "SIM"
        is_tele = active_mode == "TELE"

        if is_sim:
            sim_color = COL_DOT_ON if connected else COL_WARN
            sim_fg = COL_VALUE if connected else COL_WARN
        else:
            sim_color, sim_fg = COL_DOT_OFF, COL_TEXT_DIM
        self.link_dot.itemconfig(self.link_dot_id, fill=sim_color)
        self.mode_value_label.config(fg=sim_fg)

        if is_tele:
            tele_color = COL_DOT_ON if connected else COL_WARN
            tele_fg = COL_VALUE if connected else COL_WARN
        else:
            tele_color, tele_fg = COL_DOT_OFF, COL_TEXT_DIM
        self.port_dot.itemconfig(self.port_dot_id, fill=tele_color)
        self.port_value_label.config(fg=tele_fg)


##############################################################################
# SECTION: status_bar.py   (was gcs/app/connection/status_bar.py)
##############################################################################
class StatusBarMixin:
    """Top status bar (port menu, connect button, mode, source indicators)."""

    def _build_status_bar(self):
        self.status_panel = GlassPanel(self)
        self.status_panel.set_geometry(relx=1.0, rely=0.0, anchor="ne", dx=-8, dy=8)
        bar = self.status_panel.body

        right = tk.Frame(bar, bg=COL_GLASS)
        right.pack(side="right", padx=8, pady=6)

        # ---- Manual Connect/Disconnect cluster (top-right corner) ----
        # Packed first (side="right") so it claims the far-right edge --
        # everything else in this status bar is packed after this, which
        # is what pushes it all further left, per request.
        self.connect_btn = self._make_button(
            right, "CONNECT", self._on_connect_toggle, bold=True, small=True, width=11,
            edge="right"
        )
        self.connect_btn.pack(side="right", padx=(0, 12))

        # Baud rate dropdown -- simple fixed list of common SiK radio
        # rates, immediately left of the Connect button.
        self.baud_var = tk.StringVar(value=str(REAL_BAUD_RATE))
        baud_dropdown = tk.Menubutton(
            right, textvariable=self.baud_var, font=("Segoe UI", 9),
            bg=COL_BTN, fg=COL_TEXT, activebackground=COL_BTN_HOVER,
            activeforeground=COL_TEXT, relief="flat", bd=0, width=7, anchor="w",
            padx=6, pady=4, highlightthickness=1, highlightbackground=COL_BORDER,
            cursor="hand2"
        )
        baud_menu = tk.Menu(baud_dropdown, tearoff=0, bg=COL_BTN, fg=COL_TEXT,
                             activebackground=COL_BTN_HOVER, activeforeground=COL_TEXT)
        for b in ("4800", "9600", "19200", "38400", "57600", "115200"):
            baud_menu.add_command(label=b, command=lambda v=b: self.baud_var.set(v))
        baud_dropdown.config(menu=baud_menu)
        baud_dropdown.pack(side="right", padx=(0, 14))

        # Port dropdown -- refreshed (via postcommand) every time it's
        # opened, so it always reflects whatever's currently plugged in.
        # Immediately left of the baud dropdown. Two separate variables:
        # port_var holds the REAL device path used to actually connect
        # (e.g. "/dev/ttyUSB0" or "COM8"), while port_display_var holds
        # the shortened text shown on the button (see _clean_port_label)
        # -- so the dropdown can look clean without losing the exact
        # path mavutil needs to open the connection.
        self.port_var = tk.StringVar(value="Auto")
        self.port_display_var = tk.StringVar(value="Auto")
        self.port_dropdown = tk.Menubutton(
            right, textvariable=self.port_display_var, font=("Segoe UI", 9),
            bg=COL_BTN, fg=COL_TEXT, activebackground=COL_BTN_HOVER,
            activeforeground=COL_TEXT, relief="flat", bd=0, width=11, anchor="w",
            padx=6, pady=4, highlightthickness=1, highlightbackground=COL_BORDER,
            cursor="hand2"
        )
        self.port_menu = tk.Menu(self.port_dropdown, tearoff=0, bg=COL_BTN, fg=COL_TEXT,
                                  activebackground=COL_BTN_HOVER, activeforeground=COL_TEXT,
                                  postcommand=self._refresh_port_menu)
        self.port_dropdown.config(menu=self.port_menu)
        # Left padding added (was 0) so this box no longer sits flush
        # against the TELE indicator to its left, per request.
        self.port_dropdown.pack(side="right", padx=(10, 20))

        # Two FIXED-label indicators: "SIM" and "TELE". Previously the
        # left one's text was rewritten on every LINK_STATUS update to
        # whatever mode was currently live (display_mode(data["mode"])),
        # so connecting a real drone actually overwrote the word "SIM"
        # with "TELE" -- both dots ended up reading "TELE" at once,
        # which is what looked like "SIM changes to TELE". Neither
        # label's text is ever touched again after being created here;
        # _update_source_indicators() (below) only ever changes dot
        # color and label brightness, never the word itself.
        # Packed TELE first so it lands rightmost (closer to the port
        # dropdown), then SIM -- since both use side="right", that
        # leaves them reading "SIM  TELE" left-to-right, same order as
        # before this fix.
        tele_block = tk.Frame(right, bg=COL_GLASS)
        tele_block.pack(side="right", padx=(12, 0))
        self.port_dot = tk.Canvas(tele_block, width=10, height=10, bg=COL_GLASS, highlightthickness=0)
        self.port_dot.pack(side="left", padx=(0, 6))
        self.port_dot_id = self.port_dot.create_oval(1, 1, 9, 9, fill=COL_DOT_OFF, outline="")
        self.port_value_label = tk.Label(tele_block, text="TELE",
                                          font=("Segoe UI", 10, "bold"), fg=COL_TEXT_DIM, bg=COL_GLASS)
        self.port_value_label.pack(side="left")

        sim_block = tk.Frame(right, bg=COL_GLASS)
        sim_block.pack(side="right", padx=(12, 0))
        self.link_dot = tk.Canvas(sim_block, width=10, height=10, bg=COL_GLASS, highlightthickness=0)
        self.link_dot.pack(side="left", padx=(0, 6))
        self.link_dot_id = self.link_dot.create_oval(1, 1, 9, 9, fill=COL_DOT_OFF, outline="")
        self.mode_value_label = tk.Label(sim_block, text="SIM",
                                          font=("Segoe UI", 10, "bold"), fg=COL_TEXT_DIM, bg=COL_GLASS)
        self.mode_value_label.pack(side="left")

        # Manual override button
        self.mode_btn = self._make_button(right, "Mode: AUTO", self._cycle_mode, small=True)
        self.mode_btn.pack(side="right", padx=(0, 12))

        # Set Parameters -- opens the popup window that now holds what
        # used to be the inline Parameters section under Flight Controls.
        # Immediately left of Mode: AUTO, per request.
        self.params_btn = self._make_button(right, "Set Parameters", self._open_parameters_popup, small=True, edge="left")
        self.params_btn.pack(side="right", padx=(0, 12))


##############################################################################
# SECTION: layout.py   (was gcs/app/layout/layout.py)
##############################################################################
class LayoutMixin:
    """Window layout: places the floating panels over the map, focus/raise handling."""

    # ---------------- Layout ----------------

    def _build_layout(self):
        """The MAP fills the entire window. Everything else floats on top of
        it as translucent, rounded panels:
          * top right ......... connect / port / mode cluster
          * left .............. map buttons, then ATTITUDE / HEADING gauges
          * right ............. Telemetry / SWARM, with MAVLINK under it
          * bottom ............ one-line task banner (flight controls);
                                SIGNAL STRENGTH and FAILSAFE open as popups
        """
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # ---- full-screen map (+ map toolbar panel + WP/battery/GPS/locate boxes) ----
        map_frame = tk.Frame(self.root, bg=COL_BG)
        map_frame.grid(row=0, column=0, sticky="nsew")
        self._build_map_panel(map_frame)

        # ---- connect / port / mode cluster, top right ----
        self._build_status_bar()
        self._top_y = lambda: self.status_panel.frame.winfo_reqheight() + 16

        # ---- gauges, left (under the map buttons) ----
        self.gauges_panel = GlassPanel(self)
        self.gauges_panel.set_geometry(
            relx=0.0, rely=0.0, anchor="nw", dx=8,
            dy=lambda W, H: 8 + self.toolbar_panel.frame.winfo_reqheight() + 10,
            width=lambda W, H: self.toolbar_panel.frame.winfo_reqwidth())
        self._build_gauges_panel(self.gauges_panel.body)

        # ---- live camera feed: its own rounded box under the whole attitude + heading box ----
        self.camera_panel = GlassPanel(self, alpha=1.0)
        cam_dy = lambda W, H: (8 + self.toolbar_panel.frame.winfo_reqheight() + 10
                               + self.gauges_panel.frame.winfo_reqheight() + 10)
        # Collapsed: a LANDSCAPE box that fills the space from the gauges down to the bottom bar (max 330 px
        # high) and is as wide as a 16:9 picture of that height needs (>= gauges width, max 520 px).
        # Expanded (EXPAND button): a little less than half the screen, to the right of the left column,
        # sitting on the same bottom edge.  _cam_prog (0..1) animates between the two.
        self._cam_prog, self._cam_target, self._cam_anim_job = 0.0, 0, None
        tb_w = lambda: self.toolbar_panel.frame.winfo_reqwidth()
        bottom = lambda: getattr(self, "_bottom_reserved", lambda: 70)()
        cam_h0 = lambda W, H: max(110, min(330, H - cam_dy(W, H) - bottom()))
        cam_w0 = lambda W, H: max(tb_w(), min(520, int((cam_h0(W, H) - 54) * 16 / 9) + 20))

        def cam_exp(W, H):
            x1 = 8 + tb_w() + 10
            w1 = max(cam_w0(W, H), min(int(W * 0.46), int(W * 0.5) - x1 - 8))
            h1 = max(cam_h0(W, H), min(int(H * 0.46), int((w1 - 20) * 9 / 16) + 54))
            return x1, H - bottom() - h1, w1, h1

        lerp = lambda a, b: int(round(a + (b - a) * self._cam_prog))
        self.camera_panel.set_geometry(
            relx=0.0, rely=0.0, anchor="nw",
            dx=lambda W, H: lerp(8, cam_exp(W, H)[0]),
            dy=lambda W, H: lerp(cam_dy(W, H), cam_exp(W, H)[1]),
            width=lambda W, H: lerp(cam_w0(W, H), cam_exp(W, H)[2]),
            height=lambda W, H: lerp(cam_h0(W, H), cam_exp(W, H)[3]))
        self._build_camera_feed(self.camera_panel.body)

        # ---- bottom task banner. Built BEFORE the Telemetry panel because
        # the Signal Strength popup (created inside _build_control_panel)
        # must exist by the time Drone A's box is created. ----
        self.banner_panel = GlassPanel(self, alpha=0.90)
        self.banner_panel.body.config(padx=8)
        self.banner_panel.set_geometry(relx=0.5, rely=1.0, anchor="s", dx=0, dy=-8,
                                       width=lambda W, H: W - 16)
        self._build_control_panel(self.banner_panel.body)

        bottom_reserved = lambda: self.banner_panel.frame.winfo_reqheight() + 16
        self._bottom_reserved = bottom_reserved
        self._telem_h = 460           # replaced by the measured height below

        def telem_height(H):
            avail = H - self._top_y() - bottom_reserved() - 8
            return int(max(260, min(self._telem_h, avail - 150)))

        def log_height(H):
            avail = H - self._top_y() - bottom_reserved() - 8
            return int(max(150, avail - telem_height(H)))

        self._telem_height = telem_height
        # ---- Telemetry / SWARM, right edge (takes what's left above MAVLINK) ----
        self._sliding = set()         # panels currently animating
        self._slide_anim = {}
        self._telem_prog = 0.0        # 0 = out (off-screen right), 1 = fully in
        self._telem_target = 0
        self._telem_job = None
        self._log_prog = 0.0
        self._log_target = 0
        self._log_job = None
        self.telem_panel = GlassPanel(self, start_visible=False, closable=True, close_style="chevron",
                                      title="Telemetry",
                                      on_close=lambda: self._slide_side("telem", 0))
        self.telem_panel.set_geometry(
            relx=1.0, rely=0.0, anchor="ne",
            dx=lambda W, H: -8 + int((1.0 - self._telem_prog) * (TELEM_PANEL_W + 30)),
            dy=lambda W, H: self._top_y(),
            width=TELEM_PANEL_W,
            height=lambda W, H: telem_height(H))
        self._build_telemetry_panel(self.telem_panel.body)
        # Telemetry only needs as tall as its readings -- measure that once
        # (single-drone view); everything below it belongs to MAVLINK.
        self.root.update_idletasks()
        self._telem_h = self.telem_panel.frame.winfo_reqheight() + 6

        # ---- MAVLINK, permanently visible under Telemetry ----
        self.log_panel = GlassPanel(self, alpha=0.94, start_visible=False, closable=True,
                                    close_style="chevron", title="MAVLINK",
                                    title_font=("Segoe UI", 14, "bold"),
                                    on_close=lambda: self._slide_side("log", 0))
        self.log_panel.set_geometry(
            relx=1.0, rely=1.0, anchor="se",
            dx=lambda W, H: -8 + int((1.0 - self._log_prog) * (TELEM_PANEL_W + 30)),
            dy=lambda W, H: -bottom_reserved(),
            width=TELEM_PANEL_W, height=lambda W, H: log_height(H))
        self._build_log_panel(self.log_panel.body)

        # ---- "Telemetry" / "Mavlink" edge buttons, middle of the right side ----
        self._build_side_tabs()

        # Popups were created before the panels above; make sure they always
        # draw on top of everything else.
        self.failsafe_popup.is_popup = True

        # ---- keep floating panels glued to the main window ----
        self.root.bind("<Configure>", self._on_root_event, add="+")
        self.root.bind("<Map>", self._on_root_event, add="+")
        self.root.bind("<Unmap>", self._on_root_event, add="+")
        self.root.bind("<FocusIn>", self._on_root_focus_in, add="+")
        # Borderless windows don't get keyboard focus by themselves on some
        # desktops -- force it when an Entry is clicked.
        self.root.bind_class("Entry", "<Button-1>",
                             lambda e: e.widget.focus_force(), add="+")
        self.root.after(250, self._schedule_layout)
        self.root.after(1500, self._watch_focus)
        # 56: startup safety net -- once the layout has finished settling,
        # force every panel to re-clip its rounded corners (a few times, as
        # the compositor can take a moment), so no square/white corners remain.
        for ms in (400, 1000, 2000, 4000):
            self.root.after(ms, self._heal_panel_shapes)

    def _heal_panel_shapes(self):
        for p in list(self.panels):
            try:
                if p._toplevel and p.desired and p.win.winfo_viewable():
                    p._round(force=True)
            except tk.TclError:
                pass

    def _on_root_event(self, event):
        if event.widget is self.root:
            self._schedule_layout()
            if event.type == tk.EventType.Map:      # restored from minimise
                self._raise_burst()
            if getattr(self, "_map_menu", None):
                self._close_map_menu(immediate=True)
            # window moved/resized/restored -> re-clip shortly after
            if getattr(self, "_heal_job", None):
                try:
                    self.root.after_cancel(self._heal_job)
                except tk.TclError:
                    pass
            self._heal_job = self.root.after(150, self._heal_panel_shapes)

    def _on_root_focus_in(self, event):
        if event.widget is self.root:
            self._unfocused_polls = 0
            if not self.app_active:
                self.app_active = True
                self._schedule_layout()
            self._raise_burst()

    def _watch_focus(self):
        """The floating panels are separate borderless windows, so on their
        own they would stay on top of EVERY program (browser, terminal...).
        This keeps them tied to the dashboard: while none of the dashboard's
        own windows has keyboard focus (you switched to another app, or
        minimized it) the panels are hidden; they come straight back when
        you return to the dashboard."""
        try:
            try:
                focused = self.root.focus_displayof() is not None
            except KeyError:          # focus is on a Tk-internal widget (e.g. an open menu)
                focused = True
        except tk.TclError:
            return
        if focused:
            self._unfocused_polls = 0
            if not self.app_active:
                self.app_active = True
                self._schedule_layout()
                self._raise_burst()
        else:
            self._unfocused_polls += 1
            if self._unfocused_polls >= 2:
                # 60: the map drop-down is its own borderless window, so it
                # must close the moment you switch to another program.
                if getattr(self, "_map_menu", None):
                    self._close_map_menu(immediate=True)
                if self.app_active:
                    self.app_active = False
                    self._schedule_layout()
        self.root.after(120, self._watch_focus)

    def _schedule_layout(self, *_args):
        if getattr(self, "_sliding", None):
            return                      # re-laid out once when the slide ends
        if self._layout_job is None:
            self._layout_job = self.root.after(30, self._run_layout)

    def _run_layout(self):
        self._layout_job = None
        self._position_map_stack()
        # panels that are about to be (re)shown -> stacking must be redone afterwards
        reshown = any(p.desired and p._last_geo is None for p in self.panels)
        # Regular panels first, popups (Swarm Nav / Failsafe) last, so a popup
        # is never mapped underneath Telemetry / MAVLINK.
        for p in self.panels:
            if not p.is_popup:
                p.apply()
        for p in self.panels:
            if p.is_popup:
                p.apply()
        if reshown:
            self._raise_burst()

    def _raise_burst(self):
        """Re-stack the panels several times over ~1.5 s. The window manager
        raises the main window (and maps the panel windows) asynchronously
        when you come back to the dashboard, so one single raise can fire
        too early and lose the race -- that is what sent the Swarm Nav panel
        behind Telemetry / MAVLINK. Repeating it makes the order stick."""
        for j in self._raise_jobs:
            try:
                self.root.after_cancel(j)
            except (tk.TclError, ValueError):
                pass
        self._raise_jobs = []
        for ms in (0, 50, 150, 350, 700, 1200, 2000):
            try:
                self._raise_jobs.append(self.root.after(ms, self._raise_panels))
            except tk.TclError:
                return

    def _raise_panels(self):
        if not self.app_active:
            return
        for p in self.panels:
            if not p.is_popup:
                p.lift()
        # popups: oldest first, most recently opened ends up on top
        for p in sorted((q for q in self.panels if q.is_popup), key=lambda q: q._seq):
            p.lift()
        m = getattr(self, "_map_menu", None)
        if m:
            try:
                m["win"].lift()
            except tk.TclError:
                pass

    def _position_map_stack(self):
        """WP / Battery / GPS / Locate boxes: stacked just left of the
        Telemetry panel, starting level with its top edge."""
        x = -(8 + int(self._telem_prog * (TELEM_PANEL_W + 14)))
        y = self._top_y()
        for p in self._map_stack:
            p.set_geometry(relx=1.0, rely=0.0, anchor="ne", dx=x, dy=y)
            y += p.frame.winfo_reqheight() + 8

    def _make_map_stack_panel(self):
        """A small rounded glass box for the WP / battery / GPS / locate
        readouts. Returns the frame to build the content into."""
        p = GlassPanel(self, bg=COL_BTN, border=COL_BORDER, alpha=0.90, radius=10)
        self._map_stack.append(p)
        return p.body


##############################################################################
# SECTION: side_panels.py   (was gcs/app/layout/side_panels.py)
##############################################################################
class SidePanelsMixin:
    """Side tabs + slide-in/out animation for the telemetry and log panels."""

    # ------------------------------------------------------------------
    #  Telemetry / MAVLINK slide-in panels + the two edge buttons
    # ------------------------------------------------------------------
    def _build_side_tabs(self):
        """Two small buttons in the middle of the right side. 'Telemetry' slides the
        Telemetry/SWARM panel in from the right edge, 'Mavlink' slides the MAVLINK log
        in; clicking again slides it back out. The buttons ride along with the panels
        (like drawer handles) so they never sit on top of an open panel."""
        self._side_tabs = {}
        for key, label, toggle in (("telem", "Telemetry", self._toggle_telem_panel),
                                   ("log", "Mavlink", self._toggle_log_panel)):
            # one single rounded box per button (the panel itself IS the button)
            tp = GlassPanel(self, bg=COL_BTN, border=COL_BORDER, alpha=0.95, radius=12)
            btn = tk.Label(tp.body, text="\u25c0  " + label, font=("Segoe UI", 9), fg=COL_TEXT,
                           bg=COL_BTN, width=11, pady=7, cursor="hand2")
            btn.pack(fill="both", expand=True)
            btn.bind("<Button-1>", lambda e, f=toggle: f())
            btn.bind("<Enter>", lambda e, b=btn: b.config(bg=COL_BTN_HOVER))
            btn.bind("<Leave>", lambda e, b=btn: b.config(bg=COL_BTN))
            self._side_tabs[key] = (tp, btn, label)
        self._place_side_tabs()

    def _place_side_tabs(self):
        """Edge buttons: fixed positions on the right edge, never moved.
        They sit as a pair on the seam between the Telemetry panel (above) and
        the MAVLINK panel (below): the Telemetry button just above the seam,
        the Mavlink button just below it. So an open Telemetry panel never
        covers the Mavlink button and an open MAVLINK panel never covers the
        Telemetry button. A button is only hidden while its OWN panel is open."""
        gap = 14
        t_tp, l_tp = self._side_tabs["telem"][0], self._side_tabs["log"][0]
        seam = lambda W, H: self._top_y() + self._telem_height(H)
        t_tp.set_geometry(relx=1.0, rely=0.0, anchor="ne", dx=-8,
                          dy=lambda W, H: seam(W, H) - gap // 2 - t_tp.frame.winfo_reqheight())
        l_tp.set_geometry(relx=1.0, rely=0.0, anchor="ne", dx=-8,
                          dy=lambda W, H: seam(W, H) + gap // 2)

    def _toggle_telem_panel(self):
        self._slide_side("telem", 0 if self._telem_target else 1)

    def _toggle_log_panel(self):
        self._slide_side("log", 0 if self._log_target else 1)

    SLIDE_SECONDS = 0.22

    def _slide_side(self, which, target):
        if which == "telem":
            self._telem_target = target
            panel, cur = self.telem_panel, self._telem_prog
        else:
            self._log_target = target
            panel, cur = self.log_panel, self._log_prog
        tp, btn, label = self._side_tabs[which]
        if target:
            tp.hide()                 # the ">>" inside the panel closes it
        self._place_side_tabs()
        for key in ("telem", "log"):
            self._side_tabs[key][0].apply()
        if target and not panel.desired:
            panel.show()              # mapped off-screen (prog = 0), still invisible
        # (re)start the time-based animation from wherever the panel is now
        self._slide_anim[which] = {"p0": cur, "t0": None, "wait0": time.monotonic(),
                                   "first": True}
        self._sliding.add(which)
        job = self._telem_job if which == "telem" else self._log_job
        if job is None:
            job = self.root.after(0, lambda: self._slide_side_step(which))
            if which == "telem":
                self._telem_job = job
            else:
                self._log_job = job

    def _slide_side_step(self, which):
        if which == "telem":
            self._telem_job = None
            tgt, panel = self._telem_target, self.telem_panel
        else:
            self._log_job = None
            tgt, panel = self._log_target, self.log_panel
        a = self._slide_anim[which]

        def again(ms):
            job = self.root.after(ms, lambda: self._slide_side_step(which))
            if which == "telem":
                self._telem_job = job
            else:
                self._log_job = job

        now = time.monotonic()
        if a["t0"] is None:
            # Wait (max 0.5 s) until the freshly mapped window has its rounded
            # corners; otherwise the first half of the slide is invisible and
            # the panel appears to jump in.
            if tgt and panel._hidden_for_shape and now - a["wait0"] < 0.5:
                again(10)
                return
            a["t0"] = now
        dur = max(0.08, self.SLIDE_SECONDS * abs(tgt - a["p0"]))
        t = min(1.0, (now - a["t0"]) / dur)
        ease = 1.0 - (1.0 - t) ** 3                     # ease-out cubic
        prog = float(tgt) if t >= 1.0 else a["p0"] + (tgt - a["p0"]) * ease

        if which == "telem":
            self._telem_prog = prog
            self._position_map_stack()          # WP / battery / GPS / locate slide with the panel
            for p in self._map_stack:
                p.apply()
        else:
            self._log_prog = prog
        panel.apply()
        if a["first"]:
            a["first"] = False
            if tgt:
                panel.lift()
            for key in ("telem", "log"):        # handles always above the panels
                self._side_tabs[key][0].lift()

        if t < 1.0:
            again(16)
            return
        # ---- finished ----
        if tgt == 0:
            panel.hide()
            self._side_tabs[which][0].show()      # opening button comes back
        self._sliding.discard(which)
        self._place_side_tabs()
        for key in ("telem", "log"):
            self._side_tabs[key][0].apply()
            self._side_tabs[key][0].lift()
        self._schedule_layout()                 # one tidy re-layout after the motion


##############################################################################
# SECTION: map_panel.py   (was gcs/app/map/map_panel.py)
##############################################################################
class MapPanelMixin:
    """The map widget itself + centre/zoom buttons."""

    def _build_map_panel(self, parent):
        # Map buttons: a compact glass column at the top-left of the map.
        self.toolbar_panel = GlassPanel(self)
        self.toolbar_panel.set_geometry(relx=0.0, rely=0.0, anchor="nw", dx=8, dy=8)
        toolbar = self.toolbar_panel.body
        toolbar.config(padx=10, pady=8)
        self.map_toolbar = toolbar

        self._build_map_style_dropdown(toolbar)

        self._build_3d_map_button(toolbar)

        self._build_marking_selector(toolbar)

        self.mark_btn = self._ui_button(toolbar, "MARK WAYPOINTS", self._toggle_marking)
        self.mark_btn.config(width=20)   # extra room for the "(A)" suffix
        self.mark_btn.pack(fill="x", pady=3, ipady=3)

        self.clear_btn = self._ui_button(toolbar, "CLEAR MARKS",
                                         self._clear_manual_waypoints, kind="danger")
        self.clear_btn.pack(fill="x", pady=3, ipady=3)

        self._ui_button(toolbar, "FLY MARKED WPS", self._on_fly_marked,
                        kind="primary").pack(fill="x", pady=3, ipady=3)

        # The map itself now fills the whole window.
        map_container = tk.Frame(parent, bg=COL_BG)
        map_container.pack(fill="both", expand=True)

        self.map_widget = tkintermapview.TkinterMapView(map_container, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_tile_server(SATELLITE_TILE_SERVER, max_zoom=22)
        self.map_widget.set_position(self.last_lat, self.last_lon)
        self.map_widget.set_zoom(DEFAULT_MAP_ZOOM)

        self.drone_marker = self.map_widget.set_marker(
            self.last_lat, self.last_lon, text="1",
            icon=self.drone_icon, icon_anchor="center"
        )

        self._build_center_zoom_buttons(map_container)
        self._build_wp_overlay(map_container)
        self._build_battery_overlay(map_container)
        self._build_gps_overlay(map_container)
        self._build_locate_overlay(map_container)
        self._bind_scroll_zoom()
        self._setup_click_marking()
        self._update_marking_indicator()

    def _build_center_zoom_buttons(self, map_container):
        """55: the +/- zoom buttons now sit at the TOP CENTER of the map
        (they used to be tkintermapview's built-in pair at the top-left,
        which is why the two left panels sat 100px down). The built-in
        pair is removed and replaced by one small pill: '-' has smooth
        left corners, '+' has smooth right corners."""
        for name in ("button_zoom_in", "button_zoom_out"):
            b = getattr(self.map_widget, name, None)
            try:
                self.map_widget.canvas.delete(b.canvas_rect)
                self.map_widget.canvas.delete(b.canvas_text)
            except Exception:
                pass

        self.zoom_bar = tk.Frame(map_container, bg=COL_GLASS_BORDER, padx=0, pady=0)
        out_btn = EdgeButton(self.zoom_bar, "\u2212", lambda: self.map_widget.set_zoom(self.map_widget.zoom - 1),
                             edge="left", font=("Segoe UI", 13, "bold"), width=3, radius=10)
        in_btn = EdgeButton(self.zoom_bar, "+", lambda: self.map_widget.set_zoom(self.map_widget.zoom + 1),
                            edge="right", font=("Segoe UI", 13, "bold"), width=3, radius=10)
        out_btn.pack(side="left")
        in_btn.pack(side="left", padx=(1, 0))
        self.zoom_bar.place(relx=0.5, y=8, anchor="n")
        self.zoom_bar.lift()


##############################################################################
# SECTION: map_style.py   (was gcs/app/map/map_style.py)
##############################################################################
class MapStyleMixin:
    """Base-map style drop-down (satellite / hybrid / terrain ...)."""

    # ------------------------------------------------------------------
    # 57/58: map style drop-down (satellite + terrain maps), slides out to the right
    # ------------------------------------------------------------------
    def _build_map_style_dropdown(self, toolbar):
        """One button showing the current base map with a small arrow; clicking
        it slides open a rounded list of every map in MAP_STYLES."""
        self._map_menu = None
        btn = tk.Frame(toolbar, bg=COL_BTN, highlightthickness=1,
                       highlightbackground=COL_BORDER, highlightcolor=COL_BORDER,
                       cursor="hand2")
        self.layer_btn = btn
        self.layer_label = tk.Label(btn, text=MAP_STYLE_BY_ID[self.current_tile_server][0],
                                    font=("Segoe UI", 9), fg=COL_TEXT, bg=COL_BTN,
                                    anchor="w", width=17, cursor="hand2")
        self.layer_label.pack(side="left", padx=(10, 2), pady=6, fill="x", expand=True)
        self.layer_arrow = tk.Canvas(btn, width=14, height=14, bg=COL_BTN,
                                     highlightthickness=0, cursor="hand2")
        self.layer_arrow.pack(side="right", padx=(0, 8), pady=6)
        self._draw_map_arrow(False)

        parts = (btn, self.layer_label, self.layer_arrow)

        def _enter(_e):
            for w in parts:
                w.config(bg=COL_BTN_HOVER)

        def _leave(_e):
            for w in parts:
                w.config(bg=COL_BTN)

        for w in parts:
            w.bind("<Enter>", _enter)
            w.bind("<Leave>", _leave)
            w.bind("<Button-1>", lambda e: self._toggle_map_menu())
        btn.pack(fill="x", pady=3)

        # one global handler (cheap no-op while the menu is closed)
        self.root.bind_all("<Button-1>", self._map_menu_outside_click, add="+")
        self.root.bind_all("<Escape>", lambda e: self._close_map_menu(), add="+")

    def _draw_map_arrow(self, opened):
        c = self.layer_arrow
        c.delete("all")
        pts = (9, 3, 9, 11, 4, 7) if opened else (5, 3, 5, 11, 10, 7)   # < open, > closed
        c.create_polygon(*pts, fill=COL_TEXT_DIM, outline="")

    def _toggle_map_menu(self):
        if getattr(self, "_map_menu", None) and self._map_menu["target"] == 1:
            self._close_map_menu()
        else:
            self._open_map_menu()

    def _open_map_menu(self):
        if getattr(self, "_map_menu", None):
            if self._map_menu["target"] == 0:       # was closing -> reopen
                self._start_map_menu_anim(1)
                self._draw_map_arrow(True)
            return
        btn = self.layer_btn
        btn.update_idletasks()
        # 58: slide out from the RIGHT side of the toolbar box (never covers
        # the buttons underneath), top level with the dropdown button.
        panel = self.toolbar_panel.frame
        panel.update_idletasks()
        x = panel.winfo_rootx() + panel.winfo_width() + 6
        y = btn.winfo_rooty() - 4

        win = tk.Toplevel(self.root, bg=COL_GLASS_BORDER)
        win.withdraw()
        win.overrideredirect(True)
        try:
            win.attributes("-alpha", 0.0)      # invisible until the corners are clipped
        except tk.TclError:
            pass
        inner = tk.Frame(win, bg=COL_BTN)

        def make_row(sid, name):
            sel = sid == self.current_tile_server
            row = tk.Frame(inner, bg=COL_BTN, cursor="hand2")
            row.pack(fill="x")
            chk = tk.Label(row, text="\u2713" if sel else "", width=2,
                           font=("Segoe UI", 9, "bold"), fg=COL_VALUE, bg=COL_BTN, cursor="hand2")
            chk.pack(side="left", padx=(6, 0), pady=4)
            lbl = tk.Label(row, text=name, anchor="w",
                           font=("Segoe UI", 9, "bold" if sel else "normal"),
                           fg=COL_VALUE if sel else COL_TEXT, bg=COL_BTN, cursor="hand2")
            lbl.pack(side="left", fill="x", expand=True, padx=(2, 10), pady=4)
            parts = (row, chk, lbl)
            for p in parts:
                p.bind("<Enter>", lambda e, ps=parts: [q.config(bg=COL_BTN_HOVER) for q in ps])
                p.bind("<Leave>", lambda e, ps=parts: [q.config(bg=COL_BTN) for q in ps])
                p.bind("<Button-1>", lambda e, s=sid: self._select_map_style(s))

        for gi, (group, items) in enumerate(MAP_STYLES):
            tk.Label(inner, text=group, anchor="w", font=("Segoe UI", 8, "bold"),
                     fg=COL_TEXT_DIM, bg=COL_BTN, padx=12).pack(
                fill="x", pady=((6 if gi == 0 else 8), 1))
            for sid, name, _url, _mz in items:
                make_row(sid, name)
        tk.Frame(inner, bg=COL_BTN, height=6).pack(fill="x")

        inner.update_idletasks()
        w = max(inner.winfo_reqwidth() + 2, 170)       # final (fully open) width
        full = inner.winfo_reqheight() + 2             # fixed height
        inner.place(x=1, y=1, width=w - 2, height=full - 2)

        self._map_menu = dict(win=win, w=w, x=x, y=y, full=full, frac=0.0,
                              start_frac=0.0, start_t=time.time(), dur=0.18,
                              target=1, job=None, steps=0, shown=False)
        win.geometry(f"8x{full}+{x}+{y}")
        win.deiconify()
        win.lift()
        self._draw_map_arrow(True)
        self._start_map_menu_anim(1)

    def _start_map_menu_anim(self, target):
        m = getattr(self, "_map_menu", None)
        if not m:
            return
        if m["job"]:
            try:
                self.root.after_cancel(m["job"])
            except tk.TclError:
                pass
        m["target"] = target
        m["start_frac"] = m["frac"]
        m["start_t"] = time.time()
        m["dur"] = 0.18 if target else 0.12
        self._map_menu_step()

    def _map_menu_step(self):
        m = getattr(self, "_map_menu", None)
        if not m:
            return
        p = min(1.0, (time.time() - m["start_t"]) / m["dur"])
        e = 1 - (1 - p) ** 3                       # ease-out
        m["frac"] = m["start_frac"] + (m["target"] - m["start_frac"]) * e
        cw = max(8, int(m["w"] * m["frac"]))        # current width (slides out sideways)
        h = m["full"]
        win = m["win"]
        try:
            win.geometry(f"{cw}x{h}+{m['x']}+{m['y']}")
            win.update_idletasks()
            _round_window(win, cw, h, min(8, cw // 2))
            m["steps"] += 1
            if (m["steps"] >= 2 or p >= 1.0) and not m["shown"]:
                win.attributes("-alpha", 0.97)
                m["shown"] = True
        except tk.TclError:
            self._map_menu = None
            return
        if p < 1.0:
            m["job"] = self.root.after(15, self._map_menu_step)
        else:
            m["job"] = None
            if m["target"] == 0:
                self._destroy_map_menu()

    def _destroy_map_menu(self):
        m = getattr(self, "_map_menu", None)
        if not m:
            return
        if m["job"]:
            try:
                self.root.after_cancel(m["job"])
            except tk.TclError:
                pass
        self._map_menu = None
        try:
            m["win"].destroy()
        except tk.TclError:
            pass
        try:
            self._draw_map_arrow(False)
        except tk.TclError:
            pass

    def _close_map_menu(self, immediate=False):
        m = getattr(self, "_map_menu", None)
        if not m:
            return
        if immediate:
            self._destroy_map_menu()
        elif m["target"] != 0:
            self._draw_map_arrow(False)
            self._start_map_menu_anim(0)

    def _map_menu_outside_click(self, event):
        m = getattr(self, "_map_menu", None)
        if not m:
            return
        w = event.widget
        if isinstance(w, str):
            return
        try:
            if w.winfo_toplevel() is m["win"]:
                return
        except tk.TclError:
            return
        if str(w).startswith(str(self.layer_btn)):     # the button toggles itself
            return
        self._close_map_menu()

    def _select_map_style(self, sid):
        self._close_map_menu()
        if sid == self.current_tile_server or sid not in MAP_STYLE_BY_ID:
            return
        name, url, max_zoom = MAP_STYLE_BY_ID[sid]
        try:
            self.map_widget.set_tile_server(url, max_zoom=max_zoom)
            if self.map_widget.zoom > max_zoom:      # this map has no tiles that close
                self.map_widget.set_zoom(max_zoom)
        except Exception as exc:
            self._append_log(f"!!! Could not switch map to {name}: {exc}")
            return
        self.current_tile_server = sid
        self.layer_label.config(text=name)
        self._append_log(f">>> Map: {name}")


##############################################################################
# SECTION: map_overlays.py   (was gcs/app/map/map_overlays.py)
##############################################################################
class MapOverlaysMixin:
    """Overlays on the map (WP counter, battery, GPS, locate drone), scroll zoom, click handling."""

    def _build_wp_overlay(self, map_container):
        overlay = self._make_map_stack_panel()

        self.wp_total_label = tk.Label(overlay, text="Total WP: --",
                                        font=("Segoe UI", 9, "bold"),
                                        fg=COL_TEXT, bg=COL_BTN, anchor="w")
        self.wp_total_label.pack(anchor="w", padx=10, pady=(6, 2))

        self.wp_covered_label = tk.Label(overlay, text="Covered: --",
                                          font=("Segoe UI", 9, "bold"),
                                          fg=COL_VALUE, bg=COL_BTN, anchor="w")
        self.wp_covered_label.pack(anchor="w", padx=10, pady=(0, 6))

    def _build_battery_overlay(self, map_container):
        overlay = self._make_map_stack_panel()

        inner = tk.Frame(overlay, bg=COL_BTN)
        inner.pack(padx=6, pady=4)

        self.battery_indicator = BatteryIndicator(inner)
        self.battery_indicator.pack(side="left", padx=(0, 4))

        self.battery_percent_label = tk.Label(inner, text="--%",
                                               font=("Segoe UI", 8, "bold"),
                                               fg=COL_TEXT, bg=COL_BTN)
        self.battery_percent_label.pack(side="left")

    def _build_gps_overlay(self, map_container):
        overlay = self._make_map_stack_panel()

        self.gps_overlay_label = tk.Label(overlay, text="GPS: --",
                                           font=("Segoe UI", 8, "bold"),
                                           fg=COL_TEXT, bg=COL_BTN)
        self.gps_overlay_label.pack(padx=8, pady=4)

    def _build_locate_overlay(self, map_container):
        """Small square button directly under the GPS box (same
        top-right stack as WP/Battery/GPS) that recenters the map on
        Drone A's current position and resets zoom back to
        DEFAULT_MAP_ZOOM -- the same framing you get when the
        dashboard first opens. Meant to fix having to manually zoom
        back out every time the live drone position jumps (e.g.
        switching SIM -> TELE moves it from wherever SITL spawns to
        the drone's real GPS fix on the other side of the map)."""
        overlay = self._make_map_stack_panel()
        overlay.config(cursor="hand2")

        icon = tk.Canvas(overlay, width=20, height=20, bg=COL_BTN, highlightthickness=0)
        icon.pack(padx=8, pady=6)
        # Simple crosshair/target glyph drawn with primitives instead of
        # a font glyph, so it renders identically everywhere regardless
        # of which emoji/symbol fonts happen to be installed.
        icon.create_oval(3, 3, 17, 17, outline=COL_TEXT, width=1.5)
        icon.create_oval(8, 8, 12, 12, fill=COL_TEXT, outline="")
        icon.create_line(10, 0, 10, 4, fill=COL_TEXT, width=1.5)
        icon.create_line(10, 16, 10, 20, fill=COL_TEXT, width=1.5)
        icon.create_line(0, 10, 4, 10, fill=COL_TEXT, width=1.5)
        icon.create_line(16, 10, 20, 10, fill=COL_TEXT, width=1.5)

        for widget in (overlay, icon):
            widget.bind("<Button-1>", lambda e: self._on_locate_drone())
            widget.bind("<Enter>", lambda e: (overlay.config(bg=COL_BTN_HOVER), icon.config(bg=COL_BTN_HOVER)))
            widget.bind("<Leave>", lambda e: (overlay.config(bg=COL_BTN), icon.config(bg=COL_BTN)))

    def _on_locate_drone(self):
        # Extra safety net alongside the POSITION-handler guard above --
        # if last_lat/last_lon are still literally (0, 0) (no real fix
        # has ever come in), jumping there just lands on a blank ocean
        # tile at Null Island. Tell the user why instead of silently
        # doing that.
        if not self.gps_enabled:
            if self.odo_home:                           # GPS is OFF: go to the manual home location
                self.map_widget.set_position(*self.odo_home)
                self.map_widget.set_zoom(DEFAULT_MAP_ZOOM)
            else:
                self._append_log(">>> Locate Drone: GPS is OFF and no home location is set (VISUAL ODO panel)")
            return
        if self.last_lat == 0.0 and self.last_lon == 0.0:
            self._append_log(">>> Locate Drone: no GPS fix yet for Drone 1 -- nothing to center on")
            return
        self.map_widget.set_position(self.last_lat, self.last_lon)
        self.map_widget.set_zoom(DEFAULT_MAP_ZOOM)

    def _update_wp_overlay(self):
        total_text = str(self.wp_total) if self.wp_total else "--"
        self.wp_total_label.config(text=f"Total WP: {total_text}")
        if self.wp_total:
            self.wp_covered_label.config(text=f"Covered: {self.wp_current}/{self.wp_total}")
        else:
            self.wp_covered_label.config(text="Covered: --")

    def _bind_scroll_zoom(self):
        target = getattr(self.map_widget, "canvas", self.map_widget)

        def zoom_in(_event=None):
            self.map_widget.set_zoom(self.map_widget.zoom + 1)

        def zoom_out(_event=None):
            self.map_widget.set_zoom(self.map_widget.zoom - 1)

        def on_mousewheel(event):
            if event.delta > 0:
                zoom_in()
            else:
                zoom_out()

        target.bind("<Button-4>", zoom_in)
        target.bind("<Button-5>", zoom_out)
        target.bind("<MouseWheel>", on_mousewheel)

    def _setup_click_marking(self):
        try:
            self.map_widget.add_left_click_map_command(self._on_map_click)
        except AttributeError:
            target = getattr(self.map_widget, "canvas", self.map_widget)
            target.bind("<Button-1>", self._on_canvas_click_fallback)

    def _on_map_click(self, coords):
        lat, lon = coords
        self._show_click_coord(lat, lon)            # always: show it + copy it
        if not self.marking_mode:
            return
        self._add_manual_waypoint(lat, lon)

    def _on_canvas_click_fallback(self, event):
        try:
            lat, lon = self.map_widget.convert_canvas_coords_to_decimal_coords(event.x, event.y)
        except Exception:
            return
        self._show_click_coord(lat, lon, event.x, event.y)
        if not self.marking_mode:
            return
        self._add_manual_waypoint(lat, lon)

    def _show_click_coord(self, lat, lon, px=None, py=None):
        """Left click on the map -> a small tag with 'lat, lon' next to the cursor, and the same text is
        copied to the clipboard (paste it into the HOME LOCATION box of VISUAL ODO)."""
        text = f"{lat:.6f}, {lon:.6f}"
        copied = True
        try:
            self.root.clipboard_clear()
            self.root.clipboard_append(text)
            self.root.update_idletasks()
        except tk.TclError:
            copied = False
        mw = self.map_widget
        if px is None or py is None:
            px, py = mw.winfo_pointerx() - mw.winfo_rootx(), mw.winfo_pointery() - mw.winfo_rooty()
        tip = getattr(self, "_coord_tip", None)
        if tip is None:
            tip = tk.Label(mw, font=("DejaVu Sans Mono", 10, "bold"), fg=COL_TEXT, bg="#1b1b1f",
                           padx=8, pady=4, highlightthickness=1, highlightbackground=COL_BORDER)
            tip.bind("<Button-1>", lambda e: tip.place_forget())
            self._coord_tip = tip
        tip.config(text=text + ("   \u2713 copied" if copied else ""))
        tip.update_idletasks()
        x = min(max(4, px + 14), max(4, mw.winfo_width() - tip.winfo_reqwidth() - 4))
        y = min(max(4, py + 14), max(4, mw.winfo_height() - tip.winfo_reqheight() - 4))
        tip.place(x=x, y=y)
        tip.lift()
        job = getattr(self, "_coord_tip_job", None)
        if job is not None:
            try:
                self.root.after_cancel(job)
            except Exception:
                pass
        self._coord_tip_job = self.root.after(3500, tip.place_forget)


##############################################################################
# SECTION: waypoints.py   (was gcs/app/map/waypoints.py)
##############################################################################
class WaypointsMixin:
    """Waypoint marking: per-drone colours/pins, manual waypoints, fly-marked, mission path drawing."""

    def _build_marking_selector(self, toolbar):
        """Small status readout: a color swatch + a "Marking: A, C" label
        that always mirrors whichever drones are currently TICKED in the
        DRONES/Telemetry panel (self.swarm_selected) -- it isn't a
        separate selector anymore, just a live reflection of that one
        selection, so MARK WAYPOINTS/CLEAR MARKS/FLY MARKED WPS can never
        disagree with who you actually ticked. See
        _update_marking_indicator(), called whenever a checkbox changes."""
        wrap = tk.Frame(toolbar, bg=COL_CARD, highlightthickness=1,
                        highlightbackground=COL_BORDER, highlightcolor=COL_BORDER)
        wrap.pack(fill="x", pady=3)

        tk.Label(wrap, text="TARGET", font=("Segoe UI", 7, "bold"), fg=COL_TEXT_DIM,
                 bg=COL_CARD).pack(side="left", padx=(10, 6), pady=6)

        self.marking_swatch = tk.Canvas(wrap, width=14, height=14, bg=COL_CARD,
                                         highlightthickness=0)
        self.marking_swatch.pack(side="left", padx=(0, 4))
        self.marking_swatch_id = self.marking_swatch.create_oval(
            1, 1, 13, 13, fill=COL_TRAJECTORY, outline=COL_BORDER
        )

        self.marking_label = tk.Label(
            wrap, text="Marking: none selected", font=("Segoe UI", 9, "bold"),
            bg=COL_CARD, fg=COL_TEXT, padx=2
        )
        self.marking_label.pack(side="left", padx=(0, 8))

    def _letter_for_sysid(self, sysid):
        if sysid in self.swarm_order:
            return str(self.swarm_order.index(sysid) + 1)
        return "1" if sysid == 1 else "?"

    def _assign_wp_color(self, sysid):
        """Picks a waypoint/path color for a newly discovered drone: the
        fixed A/B/C color if it's free, otherwise the next unused color
        from the pool, otherwise a randomly generated vivid color."""
        used = set(self.drone_colors.values())
        default = DEFAULT_WP_COLORS.get(sysid)
        if default and default not in used:
            return default
        for c in WP_COLOR_POOL:
            if c not in used:
                return c
        while True:
            c = "#{:02x}{:02x}{:02x}".format(
                random.randint(90, 255), random.randint(90, 255), random.randint(90, 255)
            )
            if c not in used:
                return c

    def _get_wp_pin(self, label):
        key = str(label)
        icon = self.wp_pin_icons.get(key)
        if icon is None:
            icon = create_wp_pin_icon(key)
            self.wp_pin_icons[key] = icon
        return icon

    def _apply_drone_heading(self, sysid, heading):
        """Rotates a drone's map icon so its small arrow points along the
        drone's current heading. Heading is bucketed to 5 degrees so icons
        are cached and the map isn't re-skinned on every telemetry tick."""
        try:
            bucket = int(round(float(heading) / 5.0) * 5) % 360
        except (TypeError, ValueError):
            return
        if self.marker_heading.get(sysid) == bucket:
            return
        marker = self.drone_marker if sysid == 1 else self.swarm_markers.get(sysid)
        if marker is None:
            return
        icon = self.drone_icon_cache.get(bucket)
        if icon is None:
            icon = create_drone_icon(heading_deg=bucket)
            self.drone_icon_cache[bucket] = icon
        try:
            marker.change_icon(icon)
            self.marker_heading[sysid] = bucket
        except Exception:
            pass

    def _on_swarm_selection_changed(self):
        """Fires whenever any drone's checkbox in the DRONES/Telemetry
        panel is (un)ticked. Keeps the "Marking: ..." indicator honest,
        and -- if marking is currently in progress -- refreshes the
        formation offsets so a newly ticked/unticked drone is picked up
        right away instead of only at the next MARK WAYPOINTS click."""
        self._update_marking_indicator()
        self._sync_select_all()
        if self.marking_mode:
            self.marking_formation_offsets = self._compute_formation_offsets(
                self._selected_sysids(warn=False)
            )

    def _update_marking_indicator(self):
        sysids = self._selected_sysids(warn=False)
        if not sysids:
            self.marking_label.config(text="Marking: none selected")
            self.marking_swatch.itemconfig(self.marking_swatch_id, fill=COL_DOT_OFF)
        else:
            letters = ", ".join(self._letter_for_sysid(s) for s in sysids)
            self.marking_label.config(text=f"Marking: {letters}")
            self.marking_swatch.itemconfig(self.marking_swatch_id, fill=COL_TRAJECTORY)
        self._update_mark_button_text()

    def _update_mark_button_text(self):
        sysids = self._selected_sysids(warn=False)
        letters = ", ".join(self._letter_for_sysid(s) for s in sysids) if sysids else "none"
        if self.marking_mode:
            self.mark_btn.config(text=f"STOP MARKING ({letters})")
            self._style_ui_button(self.mark_btn, "active")
        else:
            self.mark_btn.config(text=f"MARK WAYPOINTS ({letters})")
            self._style_ui_button(self.mark_btn, "default")

    def _compute_formation_offsets(self, sysids):
        """For the given ticked drones, returns {sysid: (dlat, dlon)}: how
        far each one currently sits from the FIRST drone in the list
        (telemetry-panel order), based on their live GPS positions. Used
        so that marking one route for several drones together sends each
        one to a point offset the same way, preserving the spacing
        they're actually flying in rather than sending every drone to the
        exact same coordinates."""
        offsets = {}
        if not sysids:
            return offsets
        primary = sysids[0]
        primary_pos = self.swarm_positions.get(primary)
        for sid in sysids:
            if sid == primary or not primary_pos:
                offsets[sid] = (0.0, 0.0)
                continue
            pos = self.swarm_positions.get(sid)
            if pos:
                offsets[sid] = (pos[0] - primary_pos[0], pos[1] - primary_pos[1])
            else:
                offsets[sid] = (0.0, 0.0)
        return offsets

    def _toggle_marking(self):
        if not self.marking_mode:
            sysids = self._selected_sysids()
            if not sysids:
                return
            self.marking_mode = True
            self.marking_formation_offsets = self._compute_formation_offsets(sysids)
            letters = ", ".join(self._letter_for_sysid(s) for s in sysids)
            self._append_log(f">>> Waypoint marking ON for Drone(s) {letters} - click the map to add points")
        else:
            self.marking_mode = False
            letters = ", ".join(self._letter_for_sysid(s) for s in self._selected_sysids(warn=False))
            self._append_log(f">>> Waypoint marking OFF for Drone(s) {letters or 'none'}")
        self._update_marking_indicator()

    def _add_manual_waypoint(self, lat, lon):
        sysids = self._selected_sysids(warn=False)
        if not sysids:
            self._append_log("!!! No drone selected -- tick at least one drone in the DRONES panel first")
            return

        # Every ticked drone gets its own copy of this point, shifted by
        # its live offset from the first ticked drone (captured when
        # marking turned on) -- so a formation of 2+ drones stays spaced
        # apart instead of all converging on the same coordinates.
        primary = sysids[0]
        offsets = self.marking_formation_offsets or {}
        for sid in sysids:
            dlat, dlon = offsets.get(sid, (0.0, 0.0))
            self.manual_waypoints.setdefault(sid, []).append((lat + dlat, lon + dlon))

        # Only the first ticked drone's point is actually drawn -- with
        # several drones ticked together this is a single shared route
        # (per-drone offset targets still exist above for when the
        # mission is uploaded). Every drone uses the same trajectory
        # colour and a numbered Mission Planner style pin.
        idx = len(self.manual_waypoints[primary])
        marker = self.map_widget.set_marker(
            lat, lon, icon=self._get_wp_pin(idx), icon_anchor="s"
        )
        self.manual_markers.setdefault(primary, []).append(marker)
        self._redraw_manual_path(primary)

        letters = ", ".join(self._letter_for_sysid(s) for s in sysids)
        self._append_log(f"Marked waypoint {idx} for Drone(s) {letters}: {lat:.6f}, {lon:.6f}")

    def _redraw_manual_path(self, sysid):
        old_path = self.manual_path.get(sysid)
        if old_path:
            old_path.delete()
            self.manual_path[sysid] = None
        wps = self.manual_waypoints.get(sysid, [])
        if len(wps) > 1:
            self.manual_path[sysid] = self.map_widget.set_path(wps, color=COL_TRAJECTORY, width=3)

    def _clear_manual_waypoints(self):
        # NOTE: this used to only delete the visual markers/path stored
        # under sysids[0] of the *current* selection ("primary"), while
        # wiping the waypoint data for every currently-ticked sysid. That
        # mismatch is exactly what made CLEAR MARKS look broken: a route
        # drawn earlier while a *different* drone was primary (e.g. you
        # marked a path with only Drone C ticked, then later ticked
        # A+B and hit Clear) never had its dots/line removed from the
        # map, even though the log said "Cleared". Now every currently
        # ticked sysid has its own markers/path/data cleared, so Clear
        # Marks always removes what you can see for the drones you have
        # selected, regardless of who was "primary" when it was drawn.
        sysids = self._selected_sysids()
        if not sysids:
            return
        for sid in sysids:
            for m in self.manual_markers.get(sid, []):
                m.delete()
            self.manual_markers[sid] = []
            old_path = self.manual_path.get(sid)
            if old_path:
                old_path.delete()
                self.manual_path[sid] = None
            self.manual_waypoints[sid] = []
        if not self._sw:
            self._swarm_clear_trails()
        letters = ", ".join(self._letter_for_sysid(s) for s in sysids)
        self._append_log(f">>> Cleared marked waypoints for Drone(s) {letters}")

    def _on_fly_marked(self):
        if self._swarm_locked("FLY MARKED"):
            return
        sysids = self._selected_sysids()
        if not sysids:
            return
        primary = sysids[0]
        wps = self.manual_waypoints.get(primary, [])
        if len(wps) < 1:
            messagebox.showerror(
                "No waypoints",
                "Click MARK WAYPOINTS, then click the map to add points first."
            )
            return
        try:
            alt = float(self.takeoff_alt_entry.get())
        except ValueError:
            alt = 10.0

        letters = ", ".join(self._letter_for_sysid(s) for s in sysids)
        self._append_log(f">>> Flying {len(wps)} marked waypoints on Drone(s) {letters} "
                          f"(formation spacing preserved, one upload at a time)")

        # Uploaded one drone at a time -- upload_manual_mission() serializes
        # internally too (see _mission_transfer_lock), but looping here
        # keeps the intent obvious: every ticked drone gets its own
        # offset copy of the marked route.
        for sid in sysids:
            coords = self.manual_waypoints.get(sid, [])
            if len(coords) != len(wps):
                bad_letter = self._letter_for_sysid(sid)
                self._append_log(
                    f"!!! Skipping Drone {bad_letter}: waypoint count mismatch "
                    f"({len(coords)} vs {len(wps)}) -- clear marks and re-mark with "
                    f"all drones ticked together"
                )
                continue
            self.mav.upload_manual_mission(list(coords), alt, target_system=sid)

    def _draw_mission(self, coords):
        for m in self.mission_markers:
            m.delete()
        self.mission_markers = []

        if self.mission_path:
            self.mission_path.delete()
            self.mission_path = None
        if self.rtl_path:
            self.rtl_path.delete()
            self.rtl_path = None

        if not coords:
            return

        for i, (lat, lon) in enumerate(coords, start=1):
            marker = self.map_widget.set_marker(lat, lon, icon=self._get_wp_pin(i), icon_anchor="s")
            self.mission_markers.append(marker)

        if len(coords) > 1:
            self.mission_path = self.map_widget.set_path(coords, color=COL_MISSION_PATH, width=3)

        if len(coords) > 1 and coords[0] != coords[-1]:
            self.rtl_path = self.map_widget.set_path([coords[-1], coords[0]],
                                                       color=COL_RTL_PATH, width=3)


##############################################################################
# SECTION: control_panel.py   (was gcs/app/panels/control_panel.py)
##############################################################################
class ControlPanelMixin:
    """Control panel: takeoff, auto, arm/disarm, mode buttons."""

    def _build_control_panel(self, parent):
        """52: every flight control sits in ONE line (this is the bottom task
        banner). Same buttons, same commands, same entry fields as before.
        Signal Strength and Failsafe are not inline any more -- their banner
        buttons open/close floating popups (X in the top-right corner)."""
        bar = parent

        def sep():
            tk.Frame(bar, bg=COL_BORDER, width=1).pack(side="left", fill="y", padx=6, pady=6)

        def btn(text, command, bold=False, width=9, edge=None):
            b = self._make_button(bar, text, command, bold=bold, height=1, small=True, width=width, edge=edge)
            b.pack(side="left", padx=2, pady=7)
            return b

        def entry(width, default):
            e = tk.Entry(bar, width=width, font=("Segoe UI", 10),
                         bg="#111113", fg=COL_TEXT, insertbackground=COL_TEXT,
                         relief="flat", highlightthickness=1, highlightbackground=COL_BORDER)
            e.insert(0, default)
            e.pack(side="left", padx=(0, 4), pady=7)
            return e

        def label(text):
            tk.Label(bar, text=text, font=("Segoe UI", 10), fg=COL_TEXT_DIM,
                     bg=COL_GLASS).pack(side="left", padx=(4, 3), pady=7)

        btn("GUIDED", lambda: self._broadcast("set_mode", "GUIDED"), edge="left")
        btn("ARM", lambda: self._broadcast("arm"), bold=True)
        btn("DISARM", lambda: self._confirm_action("DISARM selected drones", lambda: self._broadcast("disarm")))
        sep()

        # Shared flight parameters -- used by both TAKEOFF and AUTO MODE
        label("Alt (m):")
        self.takeoff_alt_entry = entry(5, "10")
        label("Speed (km/h):")
        self.speed_entry = entry(5, "15")
        sep()

        btn("TAKEOFF", self._on_takeoff, bold=True)
        btn("AUTO MODE", self._on_auto, bold=True)
        sep()

        btn("LAND", lambda: self._confirm_action("Switch selected drones to LAND", lambda: self._broadcast("set_mode", "LAND")))
        btn("RTL", lambda: self._confirm_action("Switch selected drones to RTL", lambda: self._broadcast("set_mode", "RTL")))
        btn("ABORT MISSION", lambda: self._confirm_action("Abort mission on selected drones", lambda: self._broadcast("abort_mission")), width=14)
        sep()

        # ---- popups (built now, hidden until their banner button is clicked) ----
        bottom_gap = lambda W, H: -(self.banner_panel.frame.winfo_reqheight() + 22)

        self.failsafe_popup = GlassPanel(self, bg=COL_BTN, border=COL_BORDER, alpha=0.93,
                                         closable=True, start_visible=False, popup=True)
        self.failsafe_popup.set_geometry(relx=0.30, rely=1.0, anchor="s", dx=0, dy=bottom_gap)
        failsafe_frame = tk.Frame(self.failsafe_popup.body, bg=COL_BTN)
        failsafe_frame.pack(fill="both", padx=18, pady=(0, 14))
        self._build_failsafe_panel(failsafe_frame)

        self._build_swarm_nav_panel()
        self._build_odo_panel()
        btn("SWARM NAV", self._toggle_swarm_nav, bold=True, width=12)
        btn("VISUAL ODO", self._toggle_odo_panel, bold=True, width=12)
        btn("FAILSAFE", self.failsafe_popup.toggle, bold=True, width=10, edge="right")

    def _get_alt_speed(self):
        try:
            alt = float(self.takeoff_alt_entry.get())
        except ValueError:
            messagebox.showerror("Invalid altitude", "Enter a numeric altitude in meters.")
            return None, None
        try:
            speed = float(self.speed_entry.get())
        except ValueError:
            messagebox.showerror("Invalid speed", "Enter a numeric speed in km/h.")
            return None, None
        return alt, speed

    def _on_takeoff(self):
        if self._swarm_locked("TAKEOFF"):
            return
        alt, speed = self._get_alt_speed()
        if alt is None:
            return
        # Bug fix: this used to call self._selected_mavs(), a method that
        # doesn't exist anywhere in this file (a leftover from an older
        # design where each drone had its own separate MAVLink connection
        # object to iterate over). Since every drone now shares the one
        # connection (self.mav) and is addressed by sysid instead, this
        # raised an AttributeError every time TAKEOFF was clicked -- the
        # button silently did nothing because Tkinter sends that error to
        # the terminal, not the on-screen Live Console. Now it targets
        # each ticked drone by sysid, same as every other flight command.
        for sysid in self._selected_sysids():
            self.mav.takeoff(alt, target_system=sysid)
            self.mav.set_speed(speed, target_system=sysid)

    def _on_auto(self):
        # Formation distance SET + 2 or more drones ticked -> swarm auto flight
        # (the per-drone agents take off, form the triangle and fly the marked route
        # by themselves; the GCS only hands them the goal and shows what they do).
        if self._swarm_locked("AUTO MODE"):
            return
        ticked = self._selected_sysids(warn=False)
        if self.swarm_form_d and len(ticked) >= 2:
            self._swarm_auto_start(ticked)
            return
        _, speed = self._get_alt_speed()
        # Same fix as _on_takeoff above -- was calling the nonexistent
        # self._selected_mavs().
        for sysid in self._selected_sysids():
            self.mav.set_param("AUTO_OPTIONS", 3, target_system=sysid)
            self.mav.set_mode("AUTO", target_system=sysid)
            if speed is not None:
                self.mav.set_speed(speed, target_system=sysid)


##############################################################################
# SECTION: gauges_panel.py   (was gcs/app/panels/gauges_panel.py)
##############################################################################
class GaugesPanelMixin:
    """Attitude / compass / RSSI gauges panel and its drone selector."""

    def _build_gauges_panel(self, parent):
        """ONE box: the drone selector on top, then the attitude dial and the heading dial
        (each with its small readout) -- no headings and no inner boxes. Switching the
        dropdown swaps both gauges to that drone's own latest attitude/heading --
        see _on_gauge_drone_change()."""
        box = tk.Frame(parent, bg=COL_GLASS)
        box.pack(padx=10, pady=10, fill="x")

        self._build_gauge_selector(box)

        # The two dials fill the height the old layout (124 px dials + a readout row under each) used, so the
        # box keeps its size; ROLL / PITCH and HDG / DIR now sit in the bottom corners of their dial.
        probe = tk.Frame(box, bg=COL_GLASS)
        tk.Frame(probe, width=124, height=124, bg=COL_GLASS).pack(pady=(4, 4))
        ReadoutRow(probe, ("ROLL", "PITCH"), bg=COL_GLASS).pack(fill="x", padx=6, pady=(0, 14))
        tk.Frame(probe, width=124, height=124, bg=COL_GLASS).pack(pady=(0, 4))
        ReadoutRow(probe, ("HDG", "DIR"), bg=COL_GLASS).pack(fill="x", padx=6, pady=(0, 4))
        probe.pack(fill="x")
        box.update_idletasks()
        self._gauge_h_old = probe.winfo_reqheight()
        probe.destroy()
        self._gauge_g0 = max(124, (self._gauge_h_old - 12) // 2)
        self._gauge_key = None

        self.attitude_gauge = AttitudeGauge(box, size=self._gauge_g0)
        self.attitude_gauge.config(bg=COL_GLASS)
        self.attitude_gauge.pack(pady=(4, 4), fill="x")
        self.attitude_readout = GaugeCornerReadout(self.attitude_gauge, ("ROLL", "PITCH"))

        self.compass_gauge = CompassGauge(box, size=self._gauge_g0)
        self.compass_gauge.config(bg=COL_GLASS)
        self.compass_gauge.pack(pady=(0, 4), fill="x")
        self.compass_readout = GaugeCornerReadout(self.compass_gauge, ("HDG", "DIR"))
        box.bind("<Configure>", self._gauge_fit)

    def _gauge_fit(self, event):
        """Both dials get the SAME size: as big as the box height allows (never wider than the box)."""
        W = event.width
        if W < 60:
            return
        g = max(60, min(self._gauge_g0, W - 4))
        if self._gauge_key == (g, W):
            return
        self._gauge_key = (g, W)
        pad = max(0, self._gauge_h_old - 2 * g)            # keep the total height of the old layout
        a = 4 + max(0, pad - 12) // 4
        self.attitude_gauge.pack_configure(pady=(a, a))
        self.compass_gauge.pack_configure(pady=(0, max(0, pad - 2 * a)))
        self.attitude_gauge.resize(g, W)
        self.compass_gauge.resize(g, W)

    # ---------------- live camera feed (under the attitude box) ----------------
    def _build_camera_feed(self, box):
        """Small live-camera window. It is wired to the VISUAL ODO data source: GAZEBO -> the drone camera of
        the running simulator (GazeboCamera), TELEMETRY (real drone) -> 'NO SIGNAL' for now. There is no
        switch here, only an indicator that shows what is connected."""
        wrap = tk.Frame(box, bg=COL_GLASS)
        wrap.pack(fill="both", expand=True, padx=10, pady=10)
        ind = tk.Frame(wrap, bg=COL_GLASS)
        ind.pack(side="bottom", fill="x", pady=(6, 0))
        self.cam_canvas = tk.Canvas(wrap, height=100, bg="#0b0c0e", highlightthickness=1,
                                    highlightbackground=COL_BORDER, cursor="hand2")
        self.cam_canvas.pack(fill="both", expand=True)
        self._cam_img_id = self.cam_canvas.create_image(0, 0, anchor="center", state="hidden")
        self._cam_fill = False         # False = whole frame (default), True = fill the box (centre crop); click to toggle
        self.cam_canvas.bind("<Button-1>", self._cam_toggle_fill)
        self._cam_ctl_key = None
        self.cam_canvas.tag_bind("camctl", "<Button-1>", self._cam_toggle_expand)
        self.cam_canvas.tag_bind("camctl", "<Enter>", lambda e: self.cam_canvas.itemconfig("camctl_bg", fill="#2e323b"))
        self.cam_canvas.tag_bind("camctl", "<Leave>", lambda e: self.cam_canvas.itemconfig("camctl_bg", fill="#15171c"))
        self._cam_led = tk.Canvas(ind, width=10, height=10, bg=COL_GLASS, highlightthickness=0)
        self._cam_led.pack(side="left", padx=(2, 5))
        self._cam_led_id = self._cam_led.create_oval(1, 1, 9, 9, fill=self._NV_OFF, outline="")
        self._cam_lbl = tk.Label(ind, text="", font=("Segoe UI", 8, "bold"), fg=COL_TEXT_DIM,
                                 bg=COL_GLASS, anchor="w")
        self._cam_lbl.pack(side="left", fill="x")
        self._cam_view = None          # "live" / "nosig" / None  (what the canvas shows right now)
        self._cam_key = None
        self._cam_drawn_t = None
        self._cam_photo = None
        self._cam_err_logged = False
        self.root.after(600, self._cam_tick)

    @staticmethod
    def _cam_frame_to_image(fr, dw, dh, fill=False):
        """Raw Gazebo frame -> PIL image fitted (letterboxed) inside dw x dh, or None if it can't be decoded."""
        spec = {"RGB_INT8": ("RGB", "RGB", 3), "BGR_INT8": ("RGB", "BGR", 3),
                "RGBA_INT8": ("RGBA", "RGBA", 4), "BGRA_INT8": ("RGBA", "BGRA", 4),
                "L_INT8": ("L", "L", 1)}.get(fr["fmt"])
        w, h, data = fr["w"], fr["h"], fr["data"]
        if not spec or w <= 0 or h <= 0 or len(data) < w * h * spec[2]:
            return None
        img = Image.frombytes(spec[0], (w, h), bytes(data[:w * h * spec[2]]), "raw", spec[1])
        resample = getattr(getattr(Image, "Resampling", Image), "BILINEAR")
        if fill:                                   # cover the whole box (centre crop)
            k = max(dw / float(w), dh / float(h))
            size = (max(dw, int(round(w * k))), max(dh, int(round(h * k))))
            img = img.resize(size, resample, reducing_gap=2.0)
            l, t = (size[0] - dw) // 2, (size[1] - dh) // 2
            return img.crop((l, t, l + dw, t + dh))
        k = min(dw / float(w), dh / float(h))      # whole frame, black bars
        size = (max(1, int(w * k)), max(1, int(h * k)))
        return img.resize(size, resample, reducing_gap=2.0)

    def _cam_toggle_fill(self, _e=None):
        if "camctl" in self.cam_canvas.gettags("current"):      # the click was on the expand button
            return
        self._cam_fill = not self._cam_fill
        self._cam_key = None

    # ---- expand / minimize (smooth animation, same idea as the VISUAL ODO slide) ----
    def _cam_toggle_expand(self, _e=None):
        self._cam_slide(0 if self._cam_target else 1)
        return "break"

    def _cam_slide(self, target):
        self._cam_target = target
        self._cam_ctl_key = None
        if target:
            self._popup_seq += 1
            self.camera_panel._seq = self._popup_seq
            self.camera_panel.is_popup = True            # stacked above the other left-side panels
        if self._cam_anim_job is None:
            self._cam_anim_job = self.root.after(0, self._cam_anim_step)

    def _cam_anim_step(self):
        self._cam_anim_job = None
        tgt, cur = self._cam_target, self._cam_prog
        d = tgt - cur
        if abs(d) < 0.012:
            prog = float(tgt)
        else:
            step = d * 0.22
            if abs(step) < 0.012:
                step = 0.012 if d > 0 else -0.012
            prog = cur + step
        self._cam_prog = prog
        self.camera_panel.apply()
        if tgt:
            self.camera_panel.lift()
        if prog != tgt:
            self._cam_anim_job = self.root.after(20, self._cam_anim_step)
        elif tgt == 0:
            self.camera_panel.is_popup = False

    def _cam_ctl_draw(self, W):
        """Small EXPAND / MINIMIZE button in the top-right corner of the camera box."""
        cv, ex = self.cam_canvas, self._cam_target == 1
        key = (W, ex)
        if self._cam_ctl_key != key:
            self._cam_ctl_key = key
            cv.delete("camctl")
            x2, y1 = W - 7, 7
            x1, y2 = x2 - 24, y1 + 24
            cx, cy, c = (x1 + x2) / 2, (y1 + y2) / 2, "#c9ced6"
            cv.create_rectangle(x1, y1, x2, y2, fill="#15171c", outline=COL_BORDER, tags=("camctl", "camctl_bg"))
            if not ex:      # arrows pointing OUT  (expand)
                lines = [(-6, 6, 6, -6), (6, -6, 1, -6), (6, -6, 6, -1), (-6, 6, -1, 6), (-6, 6, -6, 1)]
            else:           # arrows pointing IN   (minimize)
                lines = [(6, -6, 1, -1), (1, -1, 1, -6), (1, -1, 6, -1),
                         (-6, 6, -1, 1), (-1, 1, -1, 6), (-1, 1, -6, 1)]
            for a, b, c2, d in lines:
                cv.create_line(cx + a, cy + b, cx + c2, cy + d, fill=c, width=2, tags="camctl")
        cv.tag_raise("camctl")

    def _cam_cfg(self):
        """(camera topic, model) typed in the VISUAL ODO > GAZEBO BRIDGE card ('' = automatic)."""
        try:
            return self._odo_cfg["cam_topic"].get().strip(), self._odo_cfg["model"].get().strip()
        except Exception:
            return "", ""

    def _cam_apply(self):
        if getattr(self, "_cam", None) is not None:
            self._cam.configure(*self._cam_cfg())

    def _cam_manage(self):
        want = self._gz_wanted()
        if want and self._cam is None:
            self._cam = GazeboCamera(*self._cam_cfg())
            self._cam.start()
        elif not want and self._cam is not None:
            self._cam.stop()
            self._cam = None

    def _cam_nosignal(self, W, H):
        cv = self.cam_canvas
        cv.delete("nosig")
        cv.itemconfig(self._cam_img_id, state="hidden")
        for y in range(0, H, 4):                                   # faint scan-lines
            cv.create_line(0, y, W, y, fill="#14161b", tags="nosig")
        cx, cy = W / 2, H / 2 - 6
        cv.create_rectangle(cx - 14, cy - 9, cx + 8, cy + 9, outline="#4a505b", width=2, tags="nosig")
        cv.create_polygon(cx + 8, cy - 3, cx + 17, cy - 8, cx + 17, cy + 8, cx + 8, cy + 3,
                          outline="#4a505b", fill="", width=2, tags="nosig")
        cv.create_line(cx - 18, cy + 12, cx + 20, cy - 12, fill="#c26d6d", width=2, tags="nosig")
        cv.create_text(W / 2, H - 14, text="NO SIGNAL", fill="#7a808c",
                       font=("Segoe UI", 10, "bold"), tags="nosig")

    def _cam_tick(self):
        try:
            self._cam_manage()
            self._cam_draw()
        except tk.TclError:
            return
        except Exception as e:                                       # never let the loop die
            if not self._cam_err_logged:
                self._cam_err_logged = True
                try:
                    self._append_log(f"!!! camera feed error: {e}")
                except Exception:
                    pass
        self.root.after(80, self._cam_tick)

    def _cam_draw(self):
        cv = self.cam_canvas
        W, H = max(60, cv.winfo_width()), max(40, cv.winfo_height())
        src = self._odo_src()
        color, text, live_fr = self._NV_OFF, "", None
        key = (W, H, self._cam_fill)
        if not self.odo_enabled and self._cam is None:
            color, text = self._NV_OFF, "VISUAL ODO  \u00b7  OFF" if not self.gz_auto else "waiting for Gazebo ..."
        elif src == "gazebo":
            snap = self._cam.snapshot() if self._cam is not None else None
            fr = snap["frame"] if snap else None
            if fr is not None and time.time() - fr["t"] < 2.0:
                live_fr = fr
                color, text = self._NV_OK, f"GAZEBO  \u00b7  LIVE  \u00b7  {snap['fps']:.0f} fps"
            elif snap is None or snap["state"] in ("starting", "waiting"):
                color, text = self._NV_WARN, "GAZEBO  \u00b7  waiting for camera"
            elif snap["state"] == "error":
                color, text = self._NV_BAD, "GAZEBO  \u00b7  camera error"
            else:
                color, text = self._NV_WARN, "GAZEBO  \u00b7  no frames"
        else:
            # REAL DRONE: no live-video function yet -> 'NO SIGNAL' (hook for later: return a frame here)
            color, text = self._NV_BAD, "REAL DRONE  \u00b7  NO SIGNAL"
        try:
            rpf = self._rp_cam_frame()                           # FLIGHT LOG replay video takes over the panel
        except Exception:
            rpf = None
        if rpf is not None:
            live_fr, color, text = rpf
        self._cam_led.itemconfig(self._cam_led_id, fill=color)
        self._cam_lbl.config(text=text, fg=color)
        if live_fr is not None:
            if self._cam_view != "live" or live_fr["t"] != self._cam_drawn_t or self._cam_key != key:
                img = self._cam_frame_to_image(live_fr, W - 2, H - 2, self._cam_fill)
                if img is not None:
                    self._cam_photo = ImageTk.PhotoImage(img)
                    cv.delete("nosig")
                    cv.coords(self._cam_img_id, W / 2, H / 2)
                    cv.itemconfig(self._cam_img_id, image=self._cam_photo, state="normal")
                    self._cam_view, self._cam_drawn_t, self._cam_key = "live", live_fr["t"], key
        elif self._cam_view != "nosig" or self._cam_key != key:
            self._cam_nosignal(W, H)
            self._cam_view, self._cam_key, self._cam_drawn_t = "nosig", key, None
        cv.delete("camdot")
        if live_fr is not None:                                  # red dot = the point GeoAnchor locates on the map
            cx_, cy_ = W / 2, H / 2
            cv.create_oval(cx_ - 5, cy_ - 5, cx_ + 5, cy_ + 5, fill="#ff2020", outline="#ffffff", width=1, tags="camdot")
            cv.tag_raise("camdot")
        self._cam_ctl_draw(W)

    def _build_gauge_selector(self, box):
        """Flat dropdown ('Drone 1 v') -- rebuilt any time a new drone is
        discovered (see _rebuild_gauge_menu(), called from _ensure_drone_ui)."""
        wrap = tk.Frame(box, bg=COL_GLASS)
        wrap.pack(fill="x", pady=(0, 8))

        self.gauge_menu_btn = tk.Menubutton(
            wrap, text="Drone 1 \u25be", font=("Segoe UI", 9, "bold"),
            bg=COL_CARD, fg=COL_TEXT, activebackground=COL_BTN_HOVER,
            activeforeground=COL_TEXT, relief="flat", bd=0, padx=10, pady=6,
            highlightthickness=1, highlightbackground=COL_BORDER,
            highlightcolor=COL_BORDER, cursor="hand2"
        )
        self.gauge_menu = tk.Menu(self.gauge_menu_btn, tearoff=0, bg=COL_BTN,
                                   fg=COL_TEXT, activebackground=COL_BTN_HOVER,
                                   activeforeground=COL_TEXT)
        self.gauge_menu_btn.config(menu=self.gauge_menu)
        self.gauge_menu_btn.pack(fill="x")
        self.gauge_menu_btn.bind("<Enter>", lambda e: self.gauge_menu_btn.config(bg=COL_BTN_HOVER))
        self.gauge_menu_btn.bind("<Leave>", lambda e: self.gauge_menu_btn.config(bg=COL_CARD))
        self._rebuild_gauge_menu()

    def _rebuild_gauge_menu(self):
        self.gauge_menu.delete(0, "end")
        for sysid in self.swarm_order:
            letter = self._letter_for_sysid(sysid)
            color = self.drone_colors.get(sysid, COL_TEXT)
            self.gauge_menu.add_command(
                label=f"   Drone {letter}", foreground=color,
                command=lambda s=sysid: self._on_gauge_drone_change(s)
            )

    def _on_gauge_drone_change(self, sysid):
        self.gauge_sysid = sysid
        letter = self._letter_for_sysid(sysid)
        self.gauge_menu_btn.config(text=f"Drone {letter} \u25be")
        self._refresh_gauges_for_selection()

    def _refresh_gauges_for_selection(self):
        """Repaints both gauges from whatever's already known for
        self.gauge_sysid -- called right when the dropdown changes (so
        switching drones is instant) and also from the ATTITUDE/VFR
        queue handlers whenever new telemetry for the selected drone
        arrives. A drone that's disarmed and just sitting on the ground
        still streams ATTITUDE/VFR_HUD, so this naturally shows its real
        (usually flat/level) reading rather than a stale one left over
        from whichever drone was selected before."""
        roll_pitch = self.swarm_attitude_val.get(self.gauge_sysid)
        if roll_pitch is None:
            self.attitude_gauge.update_attitude(0.0, 0.0)
            self.attitude_readout.set(None, None)
        else:
            roll, pitch = roll_pitch
            self.attitude_gauge.update_attitude(roll, pitch)
            self.attitude_readout.set(f"{roll:.1f}°", f"{pitch:.1f}°")

        heading = self.swarm_heading_val.get(self.gauge_sysid)
        if heading is None:
            self.compass_gauge.update_heading(0.0)
            self.compass_readout.set(None, None)
        else:
            self.compass_gauge.update_heading(heading)
            self.compass_readout.set(f"{heading}°", cardinal_for(heading))


##############################################################################
# SECTION: failsafe_panel.py   (was gcs/app/panels/failsafe_panel.py)
##############################################################################
class FailsafePanelMixin:
    """Battery failsafe panel."""

    # ---------------- Failsafe panel ----------------

    def _build_failsafe_panel(self, parent):
        title = tk.Label(parent, text="Failsafe", font=("Segoe UI", 15, "bold"),
                          fg=COL_TEXT, bg=COL_BTN)
        title.pack(anchor="w", pady=(0, 5))

        tk.Label(parent, text="Battery Type:", font=("Segoe UI", 11),
                 fg=COL_TEXT_DIM, bg=COL_BTN).pack(anchor="w", pady=(2, 3))

        self.battery_cell_var = tk.StringVar(value="3S")
        self.battery_type_display = tk.StringVar()
        cell_options = ["3S", "4S", "6S"]

        cell_dropdown = tk.Menubutton(
            parent, textvariable=self.battery_type_display,
            font=("Segoe UI", 10), bg=COL_BG, fg=COL_TEXT,
            activebackground=COL_BTN_HOVER, activeforeground=COL_TEXT,
            relief="flat", bd=0, width=7, anchor="w", padx=6, pady=4,
            highlightthickness=1, highlightbackground=COL_BORDER, cursor="hand2"
        )
        cell_menu = tk.Menu(cell_dropdown, tearoff=0, bg=COL_BTN, fg=COL_TEXT,
                             activebackground=COL_BTN_HOVER, activeforeground=COL_TEXT,
                             relief="flat", bd=0)
        for opt in cell_options:
            cell_menu.add_command(label=opt, command=lambda o=opt: self._on_cell_change(o))
        cell_dropdown.config(menu=cell_menu)
        cell_dropdown.pack(anchor="w", pady=(0, 6))

        tk.Label(parent, text="Full Charge Voltage:", font=("Segoe UI", 11),
                 fg=COL_TEXT_DIM, bg=COL_BTN).pack(anchor="w", pady=(0, 3))
        self.full_voltage_label = tk.Label(parent, text="12.60 V",
                                            font=("Segoe UI", 10, "bold"), fg=COL_VALUE,
                                            bg="#111113", relief="flat",
                                            highlightthickness=1, highlightbackground=COL_BORDER,
                                            width=9, anchor="center")
        self.full_voltage_label.pack(anchor="w", pady=(0, 6), ipady=3)

        tk.Label(parent, text="Failsafe Voltage (V):", font=("Segoe UI", 11),
                 fg=COL_TEXT_DIM, bg=COL_BTN).pack(anchor="w", pady=(0, 3))
        self.failsafe_voltage_entry = tk.Entry(parent, width=10, font=("Segoe UI", 11),
                                                bg="#111113", fg=COL_TEXT,
                                                insertbackground=COL_TEXT, relief="flat",
                                                highlightthickness=1, highlightbackground=COL_BORDER)
        self.failsafe_voltage_entry.pack(anchor="w", pady=(0, 6))

        self._make_button(parent, "SET", self._on_failsafe_set,
                           bold=True, small=True, width=8).pack(anchor="w")

        self.failsafe_status_label = tk.Label(parent, text="", font=("Segoe UI", 9),
                                               fg=COL_VALUE, bg=COL_BTN, wraplength=170,
                                               justify="left")
        self.failsafe_status_label.pack(anchor="w", pady=(8, 0))

        self._on_cell_change("3S")

    def _on_cell_change(self, selected):
        self.battery_cell_var.set(selected)
        self.battery_type_display.set(f"{selected}  \u25BE")
        cells = int(selected.rstrip("S"))
        full_v = cells * 4.2
        self.full_voltage_label.config(text=f"{full_v:.2f} V")

    def _on_failsafe_set(self):
        """Actually configures the autopilot's own battery failsafe
        (BATT_LOW_VOLT + FS_BATT_ENABLE) so it triggers on its own in
        flight, instead of just forcing RTL right now. Sent to every
        drone currently ticked in the DRONES/Telemetry panel, same as
        ARM/TAKEOFF/AUTO MODE."""
        voltage_str = self.failsafe_voltage_entry.get().strip()
        try:
            voltage = float(voltage_str)
        except ValueError:
            messagebox.showerror("Invalid voltage", "Enter a numeric failsafe voltage, e.g. 10.5")
            return

        sysids = self._selected_sysids()
        if not sysids:
            return

        for sysid in sysids:
            self.mav.set_param("BATT_LOW_VOLT", voltage, target_system=sysid)
            self.mav.set_param("FS_BATT_ENABLE", 1, target_system=sysid)

        who = ", ".join(self._letter_for_sysid(s) for s in sysids)
        self._append_log(
            f">>> Failsafe SET -- BATT_LOW_VOLT={voltage:.2f}V, FS_BATT_ENABLE=1 sent to {who}"
        )
        self.failsafe_status_label.config(text=f"Failsafe armed at {voltage:.2f} V")


##############################################################################
# SECTION: telemetry_panel.py   (was gcs/app/panels/telemetry_panel.py)
##############################################################################
class TelemetryPanelMixin:
    """Telemetry panel (per-drone readouts + swarm view toggle)."""

    def _build_telemetry_panel(self, parent):
        # ">>  Telemetry ........ SWARM" all live in the panel's own header row
        header = self.telem_panel.head
        self.telem_title = self.telem_panel.title_label

        self.swarm_toggle_btn = self._make_button(
            header, "SWARM", self._toggle_swarm_view, small=True
        )
        self.swarm_toggle_btn.pack(side="right", padx=(0, 14))

        self.telem_container = tk.Frame(parent, bg=COL_GLASS)
        self.telem_container.pack(padx=14, pady=(0, 8), fill="both", expand=True)

        # ---- Single-drone view (unchanged behavior, always drone A / sysid 1) ----
        self.single_view = tk.Frame(self.telem_container, bg=COL_GLASS)

        self.telem_fields = [
            ("mode", "Flight Mode"),
            ("armed", "Armed"),
            ("link", "Telemetry Link"),
            ("prearm", "Pre-Arm"),
            ("ekf", "EKF"),
            ("alt", "Altitude (m)"),
            ("lat", "Latitude"),
            ("lon", "Longitude"),
            ("groundspeed", "Ground Speed (m/s)"),
            ("airspeed", "Air Speed (m/s)"),
            ("heading", "Heading (deg)"),
            ("climb", "Climb Rate (m/s)"),
            ("voltage", "Battery Voltage (V)"),
            ("remaining", "Battery Remaining (%)"),
        ]

        for key, label_text in self.telem_fields:
            row = tk.Frame(self.single_view, bg=COL_GLASS)
            row.pack(fill="x", pady=3)
            label = tk.Label(row, text=label_text + ":", font=("Segoe UI", 11),
                              fg=COL_TEXT_DIM, bg=COL_GLASS, width=20, anchor="w")
            label.pack(side="left")
            value_label = tk.Label(row, text="--", font=("Segoe UI", 11),
                                    fg=COL_VALUE, bg=COL_GLASS, anchor="w")
            value_label.pack(side="left")
            self.values[key] = value_label

        # extra rows: IMU + local position (filled by Gazebo, or by the IMU messages of a real flight controller)
        for key, label_text in (("accel", "Accel XYZ (m/s\u00b2)"), ("gyro", "Gyro XYZ (\u00b0/s)"),
                                ("localpos", "Local X / Y (m)"), ("dr_err", "Drift vs truth (m)")):
            row = tk.Frame(self.single_view, bg=COL_GLASS)
            row.pack(fill="x", pady=3)
            tk.Label(row, text=label_text + ":", font=("Segoe UI", 11), fg=COL_TEXT_DIM,
                     bg=COL_GLASS, width=20, anchor="w").pack(side="left")
            vl = tk.Label(row, text="--", font=("Segoe UI", 11), fg=COL_VALUE, bg=COL_GLASS, anchor="w")
            vl.pack(side="left")
            self.values[key] = vl

        # ---- Swarm view (hidden until SWARM is clicked) ----
        self.swarm_view = tk.Frame(self.telem_container, bg=COL_GLASS)
        self._build_swarm_view(self.swarm_view)

        self.single_view.pack(fill="both", expand=True)


##############################################################################
# SECTION: parameters.py   (was gcs/app/panels/parameters.py)
##############################################################################
class ParametersMixin:
    """Parameters pop-up (read / set a flight-controller parameter)."""

    def _open_parameters_popup(self):
        """Opens the parameter READ/SET controls in their own small
        window (Mission-Planner-style) instead of the fixed spot they
        used to occupy at the bottom of Flight Controls -- freeing that
        space for Flight Controls/Failsafe/Signal Strength to expand
        into (see _build_control_panel) and the map above them to grow
        taller (see _build_layout). Not modal (no grab_set) -- unlike
        the DISARM/LAND/RTL confirmation popup, there's no reason
        reading/setting a parameter should block flying the drone from
        the rest of the dashboard while this window is open.

        Re-opening this while it's already open just brings the
        existing window to the front instead of creating a second one:
        self.param_name_entry/param_value_entry/param_readout are
        shared, single-instance attributes also used by
        _on_param_read/_on_param_set and the PARAM_VALUE handler in
        _poll_queue, so a second copy would leave those pointing at
        whichever popup was built most recently, silently breaking the
        other one."""
        if getattr(self, "params_dialog", None) is not None and self.params_dialog.winfo_exists():
            self.params_dialog.lift()
            self.params_dialog.focus_set()
            return

        dialog = tk.Toplevel(self.root)
        self.params_dialog = dialog
        dialog.title("Set Parameters")
        dialog.configure(bg=COL_BTN)
        dialog.resizable(False, False)
        dialog.transient(self.root)

        wrapper = tk.Frame(dialog, bg=COL_BTN, highlightthickness=1,
                            highlightbackground=COL_BORDER)
        wrapper.pack(fill="both", expand=True)

        tk.Label(wrapper, text="Parameters", font=("Segoe UI", 14, "bold"),
                 fg=COL_TEXT, bg=COL_BTN).pack(anchor="w", padx=20, pady=(18, 10))

        param_row1 = tk.Frame(wrapper, bg=COL_BTN)
        param_row1.pack(anchor="w", padx=20, pady=2)
        tk.Label(param_row1, text="Name:", font=("Segoe UI", 9),
                 fg=COL_TEXT_DIM, bg=COL_BTN).grid(row=0, column=0, padx=3)
        self.param_name_entry = tk.Entry(param_row1, width=16, font=("Segoe UI", 9),
                                          bg="#111113", fg=COL_TEXT,
                                          insertbackground=COL_TEXT, relief="flat",
                                          highlightthickness=1, highlightbackground=COL_BORDER)
        self.param_name_entry.insert(0, "BATT_VOLT_MULT")
        self.param_name_entry.grid(row=0, column=1, padx=3)
        self._make_button(param_row1, "READ", self._on_param_read, small=True).grid(row=0, column=2, padx=3)

        param_row2 = tk.Frame(wrapper, bg=COL_BTN)
        param_row2.pack(anchor="w", padx=20, pady=2)
        tk.Label(param_row2, text="Value:", font=("Segoe UI", 9),
                 fg=COL_TEXT_DIM, bg=COL_BTN).grid(row=0, column=0, padx=3)
        self.param_value_entry = tk.Entry(param_row2, width=16, font=("Segoe UI", 9),
                                           bg="#111113", fg=COL_TEXT,
                                           insertbackground=COL_TEXT, relief="flat",
                                           highlightthickness=1, highlightbackground=COL_BORDER)
        # Pre-filled with the BATT_VOLT_MULT value already corrected earlier
        self.param_value_entry.insert(0, "10.841")
        self.param_value_entry.grid(row=0, column=1, padx=3)
        self._make_button(param_row2, "SET", self._on_param_set, small=True).grid(row=0, column=2, padx=3)

        self.param_readout = tk.Label(wrapper, text="Last read: --",
                                       font=("Segoe UI", 9), fg=COL_VALUE, bg=COL_BTN)
        self.param_readout.pack(anchor="w", padx=20, pady=(6, 4))

        close_row = tk.Frame(wrapper, bg=COL_BTN)
        close_row.pack(pady=(4, 18))
        self._make_button(close_row, "Close", dialog.destroy, height=1, small=True, width=11).pack()

        dialog.update_idletasks()
        w, h = dialog.winfo_reqwidth(), dialog.winfo_reqheight()
        rx, ry = self.root.winfo_rootx(), self.root.winfo_rooty()
        rw, rh = self.root.winfo_width(), self.root.winfo_height()
        x = rx + (rw - w) // 2
        y = ry + (rh - h) // 2
        dialog.geometry(f"{w}x{h}+{x}+{y}")

        dialog.bind("<Escape>", lambda e: dialog.destroy())
        dialog.focus_set()

    def _on_param_read(self):
        name = self.param_name_entry.get().strip().upper()
        if not name:
            messagebox.showerror("Missing parameter name", "Enter a parameter name, e.g. BATT_VOLT_MULT.")
            return
        self.pending_param_read = name
        self.param_readout.config(text=f"Reading {name} ...", fg=COL_WARN)
        self.root.after(4000, lambda: self._check_param_timeout(name))
        self.mav.read_param(name)

    def _check_param_timeout(self, name):
        if self.pending_param_read == name and self.param_readout.cget("text") == f"Reading {name} ...":
            self.param_readout.config(
                text=f"!!! No response for {name} (check spelling / connection)",
                fg=COL_ARMED
            )

    def _on_param_set(self):
        name = self.param_name_entry.get().strip().upper()
        if not name:
            messagebox.showerror("Missing parameter name", "Enter a parameter name, e.g. BATT_VOLT_MULT.")
            return
        try:
            value = float(self.param_value_entry.get())
        except ValueError:
            messagebox.showerror("Invalid value", "Enter a numeric parameter value.")
            return
        self.mav.set_param(name, value)


##############################################################################
# SECTION: log_panel.py   (was gcs/app/panels/log_panel.py)
##############################################################################
class LogPanelMixin:
    """Log panel + _append_log."""

    def _build_log_panel(self, parent):
        text_frame = tk.Frame(parent, bg=COL_GLASS)
        text_frame.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        scrollbar = tk.Scrollbar(text_frame)
        scrollbar.pack(side="right", fill="y")

        self.log_text = tk.Text(text_frame, bg="#0f0f10", fg=COL_VALUE,
                                 font=("Consolas", 10), wrap="word",
                                 yscrollcommand=scrollbar.set, state="disabled",
                                 relief="flat", highlightthickness=1,
                                 highlightbackground=COL_BORDER)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.config(command=self.log_text.yview)

    def _append_log(self, text):
        self.log_text.config(state="normal")
        self.log_text.insert("end", text + "\n")
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > MAX_LOG_LINES:
            self.log_text.delete("1.0", f"{line_count - MAX_LOG_LINES}.0")
        self.log_text.see("end")
        self.log_text.config(state="disabled")


##############################################################################
# SECTION: poll_loop.py   (was gcs/app/poll_loop.py)
##############################################################################
class PollLoopMixin:
    """Main UI loop: drains the data queue from MAVLink and updates every widget."""

    # ---------------- Queue polling ----------------

    def _poll_queue(self):
        try:
            while True:
                msg_type, data = self.data_queue.get_nowait()
                try:
                    self._odo_feed(msg_type, data)      # VISUAL ODO: keeps the TELEMETRY source current
                except Exception:
                    pass

                if msg_type == "HEARTBEAT":
                    sysid = data.get("sysid", 1)
                    self._update_swarm_field(sysid, "mode", data["mode"])
                    self._update_swarm_field(
                        sysid, "armed",
                        "ARMED" if data["armed"] else "DISARMED",
                        color=COL_ARMED if data["armed"] else COL_VALUE
                    )
                    if sysid == 1:
                        self.values["mode"].config(text=data["mode"])
                        self.values["armed"].config(
                            text="ARMED" if data["armed"] else "DISARMED",
                            fg=COL_ARMED if data["armed"] else COL_VALUE
                        )
                    # A successful arm is the clearest proof pre-arm checks
                    # passed (ArduPilot never sends an explicit "PreArm: OK"
                    # message) -- so clear any stale FAILED reason here.
                    if data["armed"]:
                        self.prearm_status[sysid] = ("OK", "")
                        self._update_swarm_field(sysid, "prearm", "OK", color=COL_VALUE)
                        if sysid == 1:
                            self.values["prearm"].config(text="OK", fg=COL_VALUE)

                elif msg_type == "STATUSTEXT":
                    sysid = data.get("sysid", 1)
                    text = data["text"]
                    if "PreArm" in text:
                        reason = text.split(":", 1)[1].strip() if ":" in text else text
                        self.prearm_status[sysid] = ("FAILED", reason)
                        display = f"FAILED: {reason}"
                        self._update_swarm_field(sysid, "prearm", display, color=COL_ARMED)
                        if sysid == 1:
                            self.values["prearm"].config(text=display, fg=COL_ARMED)

                elif msg_type == "POSITION":
                    sysid = data.get("sysid", 1)
                    # Kept for every drone (not just sysid 1) so marking a
                    # route with several drones ticked can offset each
                    # one from the first drone's live position -- see
                    # _compute_formation_offsets().
                    #
                    # GLOBAL_POSITION_INT legitimately reports exactly
                    # (0, 0) whenever the EKF has no GPS fix yet (no
                    # absolute origin to report a real position from) --
                    # that's exactly what "GPS: 0 (No Fix)" plus this
                    # (0, 0) combination means. The numeric Lat/Lon
                    # readout below still shows that honestly, but
                    # letting a (0, 0) placeholder also drag the map
                    # marker/last_lat/last_lon out to Null Island (in
                    # the ocean) is what made Locate Drone land on a
                    # blank tile. Position/marker updates below are
                    # skipped for this exact placeholder -- the map just
                    # keeps showing wherever it last had a real fix.
                    # GPS switch (VISUAL ODO panel): OFF -> the position is dropped completely
                    has_fix = (not (data["lat"] == 0.0 and data["lon"] == 0.0)) and self.gps_enabled
                    if has_fix:
                        self.swarm_positions[sysid] = (data["lat"], data["lon"])
                    self._update_swarm_field(sysid, "alt", f"{data['alt']:.2f}")
                    if self.gps_enabled:
                        self._update_swarm_field(sysid, "lat", f"{data['lat']:.6f}")
                        self._update_swarm_field(sysid, "lon", f"{data['lon']:.6f}")
                    if sysid == 1:
                        self.values["alt"].config(text=f"{data['alt']:.2f}")
                        if self.gps_enabled:
                            self.values["lat"].config(text=f"{data['lat']:.6f}")
                            self.values["lon"].config(text=f"{data['lon']:.6f}")
                        if has_fix:
                            self.last_lat, self.last_lon = data["lat"], data["lon"]
                            if self.drone_marker:
                                self.drone_marker.set_position(self.last_lat, self.last_lon)
                    elif has_fix:
                        marker = self.swarm_markers.get(sysid)
                        if marker:
                            marker.set_position(data["lat"], data["lon"])

                elif msg_type == "VFR":
                    sysid = data.get("sysid", 1)
                    self._update_swarm_field(sysid, "airspeed", f"{data['airspeed']:.2f}")
                    self._update_swarm_field(sysid, "groundspeed", f"{data['groundspeed']:.2f}")
                    self._update_swarm_field(sysid, "heading", f"{data['heading']}")
                    self._update_swarm_field(sysid, "climb", f"{data['climb']:.2f}")
                    # Kept for every drone regardless of which one the main
                    # Telemetry panel (sysid 1) shows, so the ATTITUDE/HEADING
                    # box's dropdown can display any drone's own heading.
                    self.swarm_heading_val[sysid] = data["heading"]
                    self._apply_drone_heading(sysid, data["heading"])
                    if sysid == 1:
                        self.values["airspeed"].config(text=f"{data['airspeed']:.2f}")
                        self.values["groundspeed"].config(text=f"{data['groundspeed']:.2f}")
                        self.values["heading"].config(text=f"{data['heading']}")
                        self.values["climb"].config(text=f"{data['climb']:.2f}")
                    if sysid == self.gauge_sysid:
                        self.compass_gauge.update_heading(data["heading"])
                        self.compass_readout.set(f"{data['heading']}°", cardinal_for(data['heading']))

                elif msg_type == "ATTITUDE":
                    sysid = data.get("sysid", 1)
                    # Same idea as heading above -- stored per-drone, only
                    # painted on screen when it's the drone currently picked
                    # in the ATTITUDE/HEADING box's dropdown.
                    self.swarm_attitude_val[sysid] = (data["roll"], data["pitch"])
                    if sysid == self.gauge_sysid:
                        self.attitude_gauge.update_attitude(data["roll"], data["pitch"])
                        self.attitude_readout.set(f"{data['roll']:.1f}°", f"{data['pitch']:.1f}°")

                elif msg_type == "BATTERY":
                    sysid = data.get("sysid", 1)
                    self._update_swarm_field(sysid, "voltage", f"{data['voltage']:.2f}")
                    self._update_swarm_field(sysid, "remaining", f"{data['remaining']}")
                    if sysid == 1:
                        self.values["voltage"].config(text=f"{data['voltage']:.2f}")
                        self.values["remaining"].config(text=f"{data['remaining']}")
                        self.battery_indicator.update_percent(data["remaining"])
                        self.battery_percent_label.config(text=f"{data['remaining']}%")

                elif msg_type == "GPS_RAW":
                    if not self.gps_enabled:
                        continue                        # GPS switch OFF: satellites / fix are not shown
                    fix_short = {
                        0: "No GPS", 1: "No Fix", 2: "2D", 3: "3D",
                        4: "DGPS", 5: "RTK-F", 6: "RTK",
                    }.get(data["fix_type"], "?")
                    self.gps_overlay_label.config(
                        text=f"GPS: {data['satellites_visible']} ({fix_short})",
                        fg=COL_VALUE if data["fix_type"] >= 3 else COL_WARN
                    )

                elif msg_type == "EKF_STATUS":
                    # Whether the autopilot's actual fused position/
                    # velocity estimate is healthy -- a real GPS fix can
                    # coexist with a degraded EKF (bad compass, vibration,
                    # still settling after a GPS glitch), so this is
                    # checked and shown separately from the GPS field.
                    sysid = data.get("sysid", 1)
                    bad = data["bad"]
                    healthy = not bad and data["flags_ok"]
                    if healthy:
                        text, color = "OK", COL_VALUE
                    elif bad:
                        text, color = f"DEGRADED ({', '.join(bad)})", COL_ARMED
                    else:
                        text, color = "DEGRADED (not trusted)", COL_ARMED
                    self._update_swarm_field(sysid, "ekf", text, color=color)
                    if sysid == 1:
                        self.values["ekf"].config(text=text, fg=color)

                elif msg_type == "LOG":
                    self._append_log(data)

                elif msg_type == "MISSION_PATH":
                    self._draw_mission(data)

                elif msg_type == "MISSION_TOTAL":
                    self.wp_total = data
                    self.wp_current = 0
                    self._update_wp_overlay()

                elif msg_type == "MISSION_CURRENT":
                    self.wp_current = data["seq"]
                    self._update_wp_overlay()

                elif msg_type == "PARAM_VALUE":
                    if data["name"] == self.pending_param_read:
                        self.param_readout.config(text=f"Last read: {data['name']} = {data['value']}",
                                                   fg=COL_VALUE)
                        self.param_value_entry.delete(0, "end")
                        self.param_value_entry.insert(0, str(data["value"]))
                        self.pending_param_read = None

                elif msg_type == "SERIAL_STATUS":
                    self.telemetry_port = data["port"]
                    self._apply_mode_logic()

                elif msg_type == "LINK_STATUS":
                    self._update_source_indicators(display_mode(data["mode"]), data["connected"])

                elif msg_type == "MANUAL_PROBE_RESULT":
                    # Result of a manual CONNECT button probe (see
                    # MAVLink.probe_real / _on_connect_toggle). The live
                    # SIM/REAL connection was never touched during the
                    # probe itself, so on failure nothing here needs to
                    # be "put back" -- SIM (or whatever was running) was
                    # never interrupted in the first place.
                    self._close_connecting_popup()
                    if data["ok"]:
                        self._append_log(f">>> Device found on {data['port']} -- connecting ...")
                        self.manual_conn_active = True
                        self.connect_btn.config(text="DISCONNECT")
                        self.mav.start_real(data["port"], data["baud"])
                    else:
                        self._append_log(f"!!! Device not found on {data['port']} ({data.get('error', 'no heartbeat')})")
                        self._show_device_not_found_popup(data["port"])

        except queue.Empty:
            pass

        self.root.after(UPDATE_INTERVAL_MS, self._poll_queue)


##############################################################################
# SECTION: bus.py   (was gcs/app/swarm/bus.py)
##############################################################################
class SwarmBusMixin:
    """UDP swarm bus: open, send, poll agent reports, agent events, home markers."""

    # ================================================================
    #  Swarm AUTO -- the GCS is only a mission hand-over + a window.
    #  The flying (take-off, triangle, route, spacing) is done by one
    #  swarm_agent.py per drone, which talk to each other over UDP.
    # ================================================================
    SWARM_BUS_HOST = "127.0.0.1"
    SWARM_BUS_BASE = 15000        # GCS listens here; the agent of sysid N listens on BASE + N
    SWARM_SIM_TCP_BASE = 5762     # SITL instance i (0-based) serves MAVLink/TCP on 5762 + 10*i
    _SW_PHASES = ["IDLE", "PREP", "WAIT", "FORM", "TRANSIT", "SETTLE", "DONE", "STOPPED"]

    def _swarm_bus_open(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((self.SWARM_BUS_HOST, self.SWARM_BUS_BASE))
            sock.setblocking(False)
            self._bus = sock
        except OSError as e:
            self._bus = None
            self._append_log(f"!!! Swarm bus port {self.SWARM_BUS_BASE} unavailable ({e}) -- swarm AUTO "
                             f"disabled (is another gcs2.py already running?)")
            return
        self._swarm_bus_poll()

    # ---- traffic / command bookkeeping for the SWARM NAV panel ----
    def _names(self, sids):
        out = []
        for sid in sids or []:
            try:
                sid = int(sid)
            except (TypeError, ValueError):
                continue
            out.append("GCS" if sid == 0 else self._letter_for_sysid(sid))
        return ",".join(out) or "-"

    def _feed_push(self, text):
        self._feed_seq += 1
        self._feed.append((self._feed_seq, time.strftime("%H:%M:%S"), text))
        if len(self._feed) > 3000:
            del self._feed[:500]

    def _cmd_push(self, sysid, src, text):
        self._cmd_seq += 1
        lst = self._cmd_log.setdefault(int(sysid), [])
        lst.append((self._cmd_seq, time.strftime("%H:%M:%S"), f"[{src}] {text}"))
        if len(lst) > 1500:
            del lst[:300]

    def _on_agent_evt(self, m):
        """An agent reports one bus message it sent/received or one command it gave its autopilot."""
        sid, ev = m.get("id"), m.get("ev")
        me = self._names([sid])
        if ev == "cmd":
            self._cmd_push(sid, "agent", f"{m.get('what', '')} {m.get('arg', '')}".strip())
        elif ev == "bus":
            if m.get("dir") == "tx":
                self._feed_push(f"{me} \u2192 {self._names(m.get('to'))}   {m.get('what', '')}")
            else:
                self._feed_push(f"{me} \u2190 {self._names([m.get('frm', 0)])}   {m.get('what', '')} received")
        elif ev == "phase":
            self._feed_push(f"{me}   phase {m.get('frm')} \u2192 {m.get('to')}")
        elif ev == "form":
            sm = m.get("sm") or {}
            try:
                self._nav_slots = {int(k): v for k, v in sm.items()}
            except (TypeError, ValueError):
                pass
            txt = "  ".join(f"{self._names([k])}=slot {v}" for k, v in sorted(sm.items(), key=lambda kv: int(kv[0])))
            if m.get("adopt"):
                self._feed_push(f"{me} \u2190 {self._names([m.get('frm', 0)])}   formation adopted: {txt}")
            else:
                self._feed_push(f"{me} \u2192 {self._names(m.get('to'))}   formation agreed: {txt}")

    def _swarm_bus_send(self, msg, sysids):
        if self._bus is None:
            return
        kind = msg.get("k")
        if kind in ("mission", "param", "stop"):
            what = {"mission": f"mission (d={msg.get('d')} m, alt {msg.get('alt')} m, {len(msg.get('route') or [])} wp)",
                    "param": f"param: spacing d = {msg.get('d')} m",
                    "stop": f"STOP ({msg.get('why', '')})"}[kind]
            line = f"GCS \u2192 {self._names(sysids)}   {what}"
            last_txt, last_t = self._feed_last_tx
            if line != last_txt or time.time() - last_t > 3.0:      # mission is re-sent until accepted: don't flood
                self._feed_push(line)
            self._feed_last_tx = (line, time.time())
        data = json.dumps(msg).encode()
        for sid in sysids:
            try:
                self._bus.sendto(data, (self.SWARM_BUS_HOST, self.SWARM_BUS_BASE + int(sid)))
            except OSError:
                pass

    def _swarm_bus_poll(self):
        """Every 100 ms: read what the agents report, update status / map, detect the end."""
        now = time.time()
        if self._bus is not None:
            for _ in range(2000):
                try:
                    data, _addr = self._bus.recvfrom(65535)
                except (BlockingIOError, InterruptedError):
                    break
                except OSError:
                    break
                try:
                    msg = json.loads(data.decode())
                except ValueError:
                    continue
                if not isinstance(msg, dict):
                    continue
                kind = msg.get("k")
                if kind == "state" and msg.get("id") is not None:
                    msg["rx"] = now
                    self._bus_states[msg["id"]] = msg
                    try:
                        self._home_marker_update(msg["id"], msg)
                    except Exception as e:
                        self._append_log(f"!!! Home marker error: {e}")
                    w = self._gcs_rx.setdefault(msg["id"], {"n": 0, "t0": now, "c": 0, "hz": 0.0})
                    w["n"] += 1
                    w["c"] += 1
                    if now - w["t0"] >= 1.0:
                        w["hz"] = w["c"] / (now - w["t0"])
                        w["c"], w["t0"] = 0, now
                elif kind == "log":
                    self._append_log(f">>> [Drone {self._letter_for_sysid(msg.get('id'))}] {msg.get('m', '')}")
                elif kind == "link" and msg.get("id") is not None:
                    msg["rx_t"] = now
                    self._bus_links[msg["id"]] = msg
                elif kind == "evt":
                    self._on_agent_evt(msg)
            try:
                self._swarm_supervise(now)
                self._swarm_triangle_update(now)
            except Exception as e:
                self._append_log(f"!!! Swarm monitor error: {e}")
        self._swarm_poll_job = self.root.after(100, self._swarm_bus_poll)

    # ---------------- home points (H1, H2, ...) ----------------
    _HOME_REACHED = ("WAIT", "FORM", "TRANSIT", "SETTLE", "DONE")

    def _home_marker_update(self, sid, msg):
        """Each agent reports the spot its drone took off from ("home"). The marker H<n> appears on the
        map as soon as that drone has reached its flight altitude, and disappears only when the drone has
        landed on it again (RTL landing, or touched down within 3 m of it)."""
        if not self.gps_enabled:
            return
        home = msg.get("home")
        phase = msg.get("phase")
        mk = self._home_markers.get(sid)
        landed = msg.get("ls") == "LANDED" or (phase == "STOPPED" and str(msg.get("why") or "").startswith("RTL landed"))
        if not landed and mk is not None and home and msg.get("armed") is False \
                and (msg.get("alt") or 0.0) < 0.5 and sid in self._home_airborne:
            try:
                near = math.hypot(*self._ll_to_ne(home, msg["lat"], msg["lon"])) < 3.0
            except Exception:
                near = False
            landed = near
        if (msg.get("alt") or 0.0) > 2.0:
            self._home_airborne.add(sid)
        if landed:
            if mk is not None:
                try:
                    mk.delete()
                except Exception:
                    pass
                self._home_markers.pop(sid, None)
                self._append_log(f">>> Drone {self._letter_for_sysid(sid)} landed on its home point "
                                 f"H{self._letter_for_sysid(sid)}")
            self._home_airborne.discard(sid)
            return
        if not home or phase not in self._HOME_REACHED:
            return
        lat, lon = float(home[0]), float(home[1])
        if mk is None:
            label = f"H{self._letter_for_sysid(sid)}"
            try:
                self._home_markers[sid] = self.map_widget.set_marker(
                    lat, lon, icon=self._get_wp_pin(label), icon_anchor="s")
                self._append_log(f">>> Home point {label} marked for Drone {self._letter_for_sysid(sid)} "
                                 f"({lat:.6f}, {lon:.6f})")
            except Exception as e:
                self._append_log(f"!!! Could not draw home marker: {e}")
        else:
            try:
                if abs(mk.position[0] - lat) > 1e-7 or abs(mk.position[1] - lon) > 1e-7:
                    mk.set_position(lat, lon)
            except Exception:
                pass


##############################################################################
# SECTION: agent_process.py   (was gcs/app/swarm/agent_process.py)
##############################################################################
class AgentProcessMixin:
    """Starts / stops the swarm_agent.py processes (one per SITL drone)."""

    def _swarm_agents_online(self, sysids, now):
        return [sid for sid in sysids
                if sid in self._bus_states and now - self._bus_states[sid]["rx"] < 2.0]

    def _swarm_launch_agent(self, sid):
        """Starts swarm_agent.py for one SITL drone (the same thing you would type by hand)."""
        entry = self._agent_procs.get(sid)
        if entry and entry[0].poll() is None:
            return True                                   # already starting
        here = PROJECT_ROOT          # folder holding swarm_agent.py (was: this file's own folder)
        script = os.path.join(here, "swarm_agent.py")
        if not os.path.isfile(script):
            self._append_log(f"!!! swarm_agent.py not found next to gcs2.py ({script})")
            return False
        port = self.SWARM_SIM_TCP_BASE + 10 * (int(sid) - 1)
        cmd = [sys.executable, "-u", script, "--master", f"tcp:127.0.0.1:{port}", "--sysid", str(sid)]
        try:
            logf = open(os.path.join(here, f"swarm_agent_{sid}.log"), "w")
            proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT, cwd=here)
        except Exception as e:
            self._append_log(f"!!! Could not start the agent for Drone {self._letter_for_sysid(sid)}: {e}")
            return False
        self._agent_procs[sid] = (proc, logf)
        self._append_log(f">>> Starting agent for Drone {self._letter_for_sysid(sid)}: "
                         f"swarm_agent.py --master tcp:127.0.0.1:{port} --sysid {sid}")
        return True

    def shutdown_swarm(self):
        """Called when the window closes: stop the agents this GCS started."""
        for sid, (proc, logf) in list(self._agent_procs.items()):
            try:
                if proc.poll() is None:
                    proc.terminate()
                logf.close()
            except Exception:
                pass
        self._agent_procs = {}
        if self._bus is not None:
            try:
                self._bus.close()
            except OSError:
                pass
            self._bus = None


##############################################################################
# SECTION: swarm_common.py   (was gcs/app/swarm/swarm_common.py)
##############################################################################
class SwarmCommonMixin:
    """Shared swarm helpers: selected drones, lock check, broadcast, confirm dialog, trails."""

    def _selected_sysids(self, warn=True):
        """Every drone ticked in the DRONES/Telemetry panel, in
        telemetry-panel order (i.e. discovery order -- A, B, C, ...).
        This is now the single source of truth for both flight commands
        (ARM/TAKEOFF/AUTO MODE/...) and waypoint marking, so the two can
        never disagree about which drones are involved. Pass warn=False
        for calls made just to read the current selection (e.g. updating
        a label) rather than to actually act on it."""
        selected = [sid for sid in self.swarm_order if self.swarm_selected[sid].get()]
        if not selected and warn:
            self._append_log("!!! No drone selected -- tick at least one drone in the DRONES panel first")
        return selected

    def _swarm_locked(self, what):
        """While the swarm flies in AUTO the drones coordinate by themselves; the
        GCS must not send them anything except ABORT MISSION and RTL."""
        if self._sw:
            self._append_log(f"!!! SWARM AUTO is running -- {what} is blocked (the drones fly the "
                             f"formation by themselves). Only ABORT MISSION and RTL are accepted.")
            return True
        return False

    def _broadcast(self, method_name, *args):
        """Calls method_name(*args, target_system=sysid) on self.mav for
        every currently-ticked drone -- e.g. _broadcast('arm') arms only
        the drones whose checkbox in the DRONES panel is checked."""
        is_abort = method_name == "abort_mission"
        is_rtl = method_name == "set_mode" and bool(args) and str(args[0]).upper() == "RTL"
        if is_rtl:
            if self._on_swarm_rtl():
                return
        sw = self._sw
        if sw:
            if not is_abort:
                self._swarm_locked(f"{method_name} {' '.join(map(str, args))}".strip())
                return
            # ABORT MISSION: tell the agents to stop flying first (so they stop sending
            # velocity set-points), then the operator's command goes to every swarm drone.
            why = "ABORT MISSION"
            self._swarm_bus_send({"k": "stop", "why": f"GCS {why}"}, sw["ids"])
            label = (method_name + (" " + " ".join(map(str, args)) if args else "")).upper()
            for sysid in sw["ids"]:
                getattr(self.mav, method_name)(*args, target_system=sysid)
                self._cmd_push(sysid, "GCS", label)
            self._swarm_auto_stop(f"{why} pressed")
            self._swarm_triangle_clear()
            return
        if is_abort or (method_name == "set_mode" and args and str(args[0]).upper() != "GUIDED"):
            self._swarm_triangle_clear()
        label = (method_name + (" " + " ".join(map(str, args)) if args else "")).upper()
        for sysid in self._selected_sysids():
            getattr(self.mav, method_name)(*args, target_system=sysid)
            self._cmd_push(sysid, "GCS", label)

    # ---------------- Buttons ----------------

    def _confirm_action(self, action_label, command):
        """Pops up a small centered 'Are you sure?' dialog before running
        a destructive command (DISARM / LAND / RTL / ABORT MISSION).
        Blocks input to the rest of the window (grab_set) so a stray
        click elsewhere can't be mistaken for a confirmation. 'Confirm'
        runs the original command; 'Back' just closes the dialog and
        does nothing."""
        dialog = tk.Toplevel(self.root)
        dialog.title("Confirm")
        dialog.configure(bg=COL_BTN)
        dialog.resizable(False, False)
        dialog.transient(self.root)

        # Build content first so winfo_reqwidth/height below are accurate,
        # then center the dialog over the main window before showing it.
        wrapper = tk.Frame(dialog, bg=COL_BTN, highlightthickness=1,
                            highlightbackground=COL_BORDER)
        wrapper.pack(fill="both", expand=True)

        tk.Label(wrapper, text="Are you sure?", font=("Segoe UI", 14, "bold"),
                 fg=COL_TEXT, bg=COL_BTN).pack(padx=30, pady=(22, 4))
        tk.Label(wrapper, text=action_label, font=("Segoe UI", 10),
                 fg=COL_TEXT_DIM, bg=COL_BTN).pack(padx=30, pady=(0, 18))

        btn_row = tk.Frame(wrapper, bg=COL_BTN)
        btn_row.pack(pady=(0, 20))

        def do_confirm():
            dialog.destroy()
            command()

        def do_back():
            dialog.destroy()

        self._make_button(btn_row, "Back", do_back, height=1, small=True,
                           width=11).grid(row=0, column=0, padx=8)
        self._make_button(btn_row, "Confirm", do_confirm, bold=True,
                           height=1, small=True, width=11).grid(row=0, column=1, padx=8)

        dialog.update_idletasks()
        w, h = dialog.winfo_reqwidth(), dialog.winfo_reqheight()
        rx, ry = self.root.winfo_rootx(), self.root.winfo_rooty()
        rw, rh = self.root.winfo_width(), self.root.winfo_height()
        x = rx + (rw - w) // 2
        y = ry + (rh - h) // 2
        dialog.geometry(f"{w}x{h}+{x}+{y}")

        dialog.grab_set()
        dialog.focus_set()
        dialog.bind("<Escape>", lambda e: do_back())

    def _drone_state(self, sid):
        f = self.nav_fields.get(sid, {})
        mode = (f.get("mode") or "").upper()
        armed = f.get("armed") == "ARMED"
        try:
            alt = float(f.get("alt"))
        except (TypeError, ValueError):
            alt = 0.0
        return mode, armed, alt, self.swarm_positions.get(sid)

    def _make_dotted_path(self, positions):
        """Planned route: same yellow as the covered trail, but dotted."""
        path = self.map_widget.set_path(positions, color=COL_TRAJECTORY, width=3)
        try:
            self.map_widget.canvas.itemconfig(path.canvas_line, dash=(1, 7))
        except Exception:
            pass
        return path

    def _swarm_clear_trails(self):
        for p in self.swarm_trail_paths.values():
            try:
                p.delete()
            except Exception:
                pass
        self.swarm_trail_paths = {}
        self.swarm_trail_pts = {}
        try:
            self._gz_clear_trail()
        except Exception:
            pass


##############################################################################
# SECTION: swarm_math.py   (was gcs/app/swarm/swarm_math.py)
##############################################################################
class SwarmMathMixin:
    """Lat/lon <-> north/east metres, angle wrap, formation slots (GCS side)."""

    # ---------------- Swarm formation auto flight ----------------
    _M_PER_DEG = 111320.0

    def _ll_to_ne(self, o, lat, lon):
        return ((lat - o[0]) * self._M_PER_DEG,
                (lon - o[1]) * self._M_PER_DEG * math.cos(math.radians(o[0])))

    def _ne_to_ll(self, o, n, e):
        return (o[0] + n / self._M_PER_DEG,
                o[1] + e / (self._M_PER_DEG * math.cos(math.radians(o[0]))))

    @staticmethod
    def _wrap_pi(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def _formation_slots(count, d):
        """(forward, right) metres relative to the leader. Leader = apex;
        followers fill an arrow/wedge behind it. With 3 drones this is an
        equilateral triangle: every pair is exactly d apart."""
        slots = [(0.0, 0.0)]
        for i in range(1, count):
            k = (i + 1) // 2
            side = -1.0 if i % 2 == 1 else 1.0
            slots.append((-0.8660254 * d * k, side * 0.5 * d * k))
        return slots


##############################################################################
# SECTION: swarm_view.py   (was gcs/app/swarm/swarm_view.py)
##############################################################################
class SwarmViewMixin:
    """Swarm view inside the telemetry panel: per-drone cards, select-all, link health, spawn commands."""

    def _toggle_swarm_view(self):
        self.swarm_view_active = not self.swarm_view_active
        if self.swarm_view_active:
            self.single_view.pack_forget()
            self.swarm_view.pack(fill="both", expand=True)
            self.swarm_toggle_btn.config(text="TELEMETRY")
        else:
            self.swarm_view.pack_forget()
            self.single_view.pack(fill="both", expand=True)
            self.swarm_toggle_btn.config(text="SWARM")

    def _build_swarm_view(self, parent):
        # Whole DRONES section lives inside one outer bordered box. (No
        # big section heading here -- the SWARM/TELEMETRY toggle button
        # right above this panel already says what it is, so the heading
        # was just spending vertical space for no extra information.)
        outer = tk.Frame(parent, bg=COL_GLASS, highlightthickness=1,
                          highlightbackground=COL_BORDER)
        outer.pack(fill="both", expand=True, pady=(4, 0))

        # ---- "Select all" row at the top of the DRONES box ----
        top = tk.Frame(outer, bg=COL_GLASS)
        top.pack(fill="x", padx=12, pady=(10, 6))
        self._select_all_var = tk.BooleanVar(value=False)
        self.select_all_chk = SmoothCheck(
            top, text="Select all", variable=self._select_all_var,
            font=("Segoe UI", 11, "bold"), fg=COL_TEXT, bg=COL_GLASS, size=22,
            on_click=self._on_select_all_click)
        self.select_all_chk.pack(side="left")
        self.select_count_lbl = tk.Label(top, text="", font=("Segoe UI", 9),
                                         fg=COL_TEXT_DIM, bg=COL_GLASS)
        self.select_count_lbl.pack(side="right")
        tk.Frame(outer, bg=COL_BORDER, height=1).pack(fill="x", padx=10, pady=(0, 6))

        # Container the individual per-drone boxes get added into as new
        # sysids are discovered. This lives inside a canvas + scrollbar
        # instead of a plain packed frame, for two reasons: (1) it gives
        # this section mouse-wheel scrolling once there are more drone
        # boxes than fit, so Drone C etc. are always reachable, and (2) a
        # canvas's own requested size does NOT grow with what's drawn
        # inside it (unlike a Frame), so a growing drone list no longer
        # forces the Telemetry column taller -- which is what was
        # shrinking the MAVLINK log panel and enlarging the map whenever
        # SWARM view was opened.
        scroll_frame = tk.Frame(outer, bg=COL_GLASS)
        scroll_frame.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self.swarm_canvas = tk.Canvas(scroll_frame, bg=COL_GLASS, highlightthickness=0)
        swarm_scrollbar = tk.Scrollbar(scroll_frame, orient="vertical",
                                        command=self.swarm_canvas.yview)
        self.swarm_canvas.configure(yscrollcommand=swarm_scrollbar.set)
        self.swarm_canvas.pack(side="left", fill="both", expand=True)
        swarm_scrollbar.pack(side="right", fill="y")

        self.swarm_boxes_container = tk.Frame(self.swarm_canvas, bg=COL_GLASS)
        self.swarm_boxes_window = self.swarm_canvas.create_window(
            (0, 0), window=self.swarm_boxes_container, anchor="nw"
        )

        def _on_swarm_frame_configure(event):
            self.swarm_canvas.configure(scrollregion=self.swarm_canvas.bbox("all"))

        def _on_swarm_canvas_configure(event):
            # Keep the inner frame's width matched to the canvas viewport
            # so the drone boxes still stretch to fill the panel width.
            self.swarm_canvas.itemconfig(self.swarm_boxes_window, width=event.width)

        self.swarm_boxes_container.bind("<Configure>", _on_swarm_frame_configure)
        self.swarm_canvas.bind("<Configure>", _on_swarm_canvas_configure)

        def _on_swarm_mousewheel(event):
            if event.num == 5 or event.delta < 0:
                self.swarm_canvas.yview_scroll(1, "units")
            elif event.num == 4 or event.delta > 0:
                self.swarm_canvas.yview_scroll(-1, "units")

        # Only hijack the scroll wheel while the cursor is actually over
        # the DRONES panel, so it doesn't affect scrolling/zoom anywhere
        # else in the dashboard (e.g. the map).
        def _swarm_scroll_bind(_event):
            self.swarm_canvas.bind_all("<MouseWheel>", _on_swarm_mousewheel)
            self.swarm_canvas.bind_all("<Button-4>", _on_swarm_mousewheel)
            self.swarm_canvas.bind_all("<Button-5>", _on_swarm_mousewheel)

        def _swarm_scroll_unbind(_event):
            self.swarm_canvas.unbind_all("<MouseWheel>")
            self.swarm_canvas.unbind_all("<Button-4>")
            self.swarm_canvas.unbind_all("<Button-5>")

        self.swarm_canvas.bind("<Enter>", _swarm_scroll_bind)
        self.swarm_canvas.bind("<Leave>", _swarm_scroll_unbind)

        # Drone A (sysid 1) always exists by default, even before any
        # telemetry has arrived, so its checkbox/box is there from the start.
        self._ensure_drone_ui(1)

    def _on_select_all_click(self):
        """All ticked -> untick everything; otherwise tick every drone."""
        want = not all(v.get() for v in self.swarm_selected.values())
        for v in self.swarm_selected.values():
            v.set(want)

    def _sync_select_all(self):
        chk = getattr(self, "select_all_chk", None)
        if chk is None:
            return
        vals = [v.get() for v in self.swarm_selected.values()]
        n, total = sum(vals), len(vals)
        self._select_all_var.set(total > 0 and n == total)
        chk.set_partial(0 < n < total)
        self.select_count_lbl.config(text=f"{n}/{total} selected")

    def _ensure_drone_ui(self, sysid):
        """Creates the checkbox + telemetry box for a sysid the first time
        it's seen. Called from HEARTBEAT handling for any new sysid, so a
        4th, 5th, ... vehicle just appears on its own -- no code change
        needed to support more drones."""
        if sysid in self.swarm_widgets:
            return

        self.swarm_order.append(sysid)
        number = str(len(self.swarm_order))   # "1", "2", "3", ...
        self.swarm_data.setdefault(sysid, {})

        if sysid not in self.drone_colors:
            self.drone_colors[sysid] = self._assign_wp_color(sysid)

        default_selected = (sysid == 1)
        self.swarm_selected[sysid] = tk.BooleanVar(value=default_selected)
        # Waypoint marking (MARK WAYPOINTS/CLEAR MARKS/FLY MARKED WPS) now
        # follows this exact same ticked set -- see _on_swarm_selection_changed.
        self.swarm_selected[sysid].trace_add(
            "write", lambda *_args: self._on_swarm_selection_changed()
        )

        box = tk.Frame(self.swarm_boxes_container, bg=COL_BTN,
                        highlightthickness=1, highlightbackground=COL_BORDER)
        box.pack(fill="x", pady=5)

        header = tk.Frame(box, bg=COL_BTN)
        header.pack(fill="x", padx=10, pady=(8, 3))

        chk = SmoothCheck(
            header, text=f"Drone {number}", variable=self.swarm_selected[sysid],
            font=("Segoe UI", 12, "bold"), fg=COL_TEXT, bg=COL_BTN, size=22
        )
        chk.pack(side="left")

        tk.Label(header, text=f"(sysid {sysid})", font=("Segoe UI", 9),
                 fg=COL_TEXT_DIM, bg=COL_BTN).pack(side="left", padx=(8, 0))

        dot = tk.Canvas(header, width=11, height=11, bg=COL_BTN, highlightthickness=0)
        dot.pack(side="right")
        dot_id = dot.create_oval(1, 1, 10, 10, fill=COL_DOT_ON, outline="")

        grid = tk.Frame(box, bg=COL_BTN)
        grid.pack(fill="x", padx=8, pady=(0, 8))

        field_labels = {
            "mode": "Mode", "armed": "Armed", "link": "Link", "prearm": "Pre-Arm",
            "ekf": "EKF",
            "alt": "Alt (m)",
            "lat": "Lat", "lon": "Lon", "heading": "Hdg (deg)",
            "climb": "Climb (m/s)", "voltage": "Batt (V)", "remaining": "Batt (%)",
        }
        widgets = {"dot": dot, "dot_id": dot_id, "box": box}
        for i, (key, short_label) in enumerate(field_labels.items()):
            row_i, col_i = divmod(i, 2)
            cell = tk.Frame(grid, bg=COL_BTN)
            cell.grid(row=row_i, column=col_i, sticky="w", padx=(0, 6), pady=1)
            tk.Label(cell, text=short_label + ":", font=("Segoe UI", 10),
                     fg=COL_TEXT_DIM, bg=COL_BTN, anchor="w", width=10
                     ).pack(side="left")
            val = tk.Label(cell, text="--", font=("Segoe UI", 10, "bold"),
                            fg=COL_VALUE, bg=COL_BTN, anchor="w")
            val.pack(side="left")
            widgets[key] = val

        self.swarm_widgets[sysid] = widgets

        # sysid 1 already has its marker (self.drone_marker, created when
        # the map was built). Any other sysid needs its own marker on the
        # map so all discovered drones actually show up there, not just A.
        if sysid != 1 and sysid not in self.swarm_markers:
            self.swarm_markers[sysid] = self.map_widget.set_marker(
                self.last_lat, self.last_lon, text=number,
                icon=self.drone_icon, icon_anchor="center"
            )

        self._update_marking_indicator()
        self._sync_select_all()
        self._rebuild_gauge_menu()

    def _update_swarm_field(self, sysid, key, text, color=None):
        self._ensure_drone_ui(sysid)
        self.nav_fields.setdefault(sysid, {})[key] = text
        w = self.swarm_widgets[sysid].get(key)
        if w:
            w.config(text=text, fg=color if color else COL_VALUE)

    def _check_link_health(self):
        """Runs once a second, independent of whatever telemetry happens
        to arrive. Compares 'now' against the last time each known drone
        was actually heard from (self.mav.link_last_seen, updated on every
        MAVLink message in the worker thread) -- this is what catches a
        dead radio link that the rest of the UI would otherwise just sit
        there quietly displaying stale numbers for."""
        now = time.time()
        for sysid in list(self.swarm_order):
            last_seen = self.mav.link_last_seen.get(sysid)
            if last_seen is None:
                continue  # nothing received yet -- leave fields at their "--" default
            elapsed = now - last_seen
            lost = elapsed > LINK_TIMEOUT_S
            text = "LIVE" if not lost else f"LOST {elapsed:.0f}s ago"
            color = COL_DOT_ON if not lost else COL_ARMED
            self._update_swarm_field(sysid, "link", text, color=color)

            widgets = self.swarm_widgets.get(sysid)
            if widgets:
                widgets["dot"].itemconfig(widgets["dot_id"], fill=color)
                box = widgets.get("box")
                if box:
                    # Thicker, red-tinted border makes a dead link
                    # impossible to miss even at a glance across the
                    # whole DRONES panel, not just in the text field.
                    box.config(
                        highlightbackground=COL_ARMED if lost else COL_BORDER,
                        highlightthickness=2 if lost else 1,
                    )

            if sysid == 1:
                self.values["link"].config(text=text, fg=color)

        self.root.after(LINK_CHECK_MS, self._check_link_health)

    def _on_generate_spawn_commands(self):
        try:
            distance = float(self.spawn_distance_entry.get())
            base_lat = float(self.spawn_lat_entry.get())
            base_lon = float(self.spawn_lon_entry.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Distance/Base Lat/Base Lon must be numbers.")
            return

        self._append_log(f">>> Spawn command generated ({distance:.1f} m spacing, "
                          f"3 vehicles, single shared connection):")
        self._append_log(
            f"  sim_vehicle.py -v ArduCopter --console --map --count 3 "
            f"--auto-sysid --location CMAC "
            f"--auto-offset-line 90,{distance:.0f} "
            f"--out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14552"
        )
        self._append_log(
            "  (Note: --location CMAC ignores the Base Lat/Lon fields above -- "
            "sim_vehicle.py's --auto-offset-line only works relative to a "
            "named --location, not arbitrary custom coordinates.)"
        )


##############################################################################
# SECTION: formation_bar.py   (was gcs/app/swarm/formation_bar.py)
##############################################################################
class FormationBarMixin:
    """Formation bar: shape picker, distance, shape icons, status."""

    # ---------------- Formation bar (fixed at the bottom of SWARM NAV) ----------------
    def _build_formation_bar(self, parent):
        CARD = "#232326"
        card = tk.Frame(parent, bg=CARD, highlightthickness=1, highlightbackground=COL_BORDER)
        card.pack(side="bottom", fill="x", padx=12, pady=(4, 12))

        head = tk.Frame(card, bg=CARD)
        head.pack(fill="x", padx=12, pady=(9, 2))
        icon = tk.Canvas(head, width=22, height=18, bg=CARD, highlightthickness=0)
        icon.pack(side="left", padx=(0, 8))
        self._form_head_icon = icon
        tk.Label(head, text="FORMATION CONTROL", font=("Segoe UI", 9, "bold"),
                 fg=COL_TEXT, bg=CARD).pack(side="left")
        self.form_status = tk.Label(head, text="", font=("Segoe UI", 9, "bold"),
                                    fg=COL_TEXT_DIM, bg=CARD, anchor="e")
        self.form_status.pack(side="right")

        tk.Frame(card, bg=COL_BORDER, height=1).pack(fill="x", padx=12, pady=(6, 8))

        # ---- shape selector (icons follow the number of ticked drones: 3 = triangle, 4 = square ...) ----
        srow = tk.Frame(card, bg=CARD)
        srow.pack(fill="x", padx=12, pady=(0, 8))
        tk.Label(srow, text="Shape", font=("Segoe UI", 10), fg=COL_TEXT,
                 bg=CARD).pack(side="left", padx=(0, 10))
        self._form_shape_btns = {}
        for key, label in FORMATION_SHAPES:
            cell = tk.Frame(srow, bg=CARD, cursor="hand2", highlightthickness=1,
                            highlightbackground=COL_BORDER)
            cell.pack(side="left", padx=(0, 6))
            cv = tk.Canvas(cell, width=44, height=34, bg=CARD, highlightthickness=0, cursor="hand2")
            cv.pack(padx=4, pady=(3, 0))
            lb = tk.Label(cell, text=label, font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=CARD, cursor="hand2")
            lb.pack(padx=4, pady=(0, 3))
            for w_ in (cell, cv, lb):
                w_.bind("<Button-1>", lambda e, k=key: self._on_pick_formation_shape(k))
            self._form_shape_btns[key] = (cell, cv, lb)
        self._form_icon_key = None
        self._form_icons_tick()

        row = tk.Frame(card, bg=CARD)
        row.pack(fill="x", padx=12, pady=(0, 6))
        tk.Label(row, text="Distance between", font=("Segoe UI", 10), fg=COL_TEXT,
                 bg=CARD).pack(side="left")
        self.form_entry = tk.Entry(row, width=7, font=("Segoe UI", 11), justify="center",
                                   bg="#111113", fg=COL_TEXT, insertbackground=COL_TEXT,
                                   relief="flat", highlightthickness=1,
                                   highlightbackground=COL_BORDER, highlightcolor=COL_VALUE)
        self.form_entry.insert(0, "5")
        self.form_entry.pack(side="left", padx=(10, 4), ipady=3)
        tk.Label(row, text="m", font=("Segoe UI", 10), fg=COL_TEXT_DIM, bg=CARD).pack(side="left")
        self.form_entry.bind("<Return>", lambda e: self._on_set_formation_distance())
        self._make_button(row, "SET", self._on_set_formation_distance, bold=True,
                          small=True, height=1, width=8).pack(side="left", padx=(12, 0))

        tk.Label(card, text="Gap kept between neighbouring drones while flying a route "
                            "(0 = formation off). Pick a shape, then press AUTO MODE.",
                 font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=CARD, anchor="w",
                 justify="left", wraplength=500).pack(fill="x", padx=12, pady=(0, 9))
        self._refresh_form_status()

    def _form_preview_count(self):
        try:
            n = len(self._selected_sysids(warn=False))
        except Exception:
            n = 0
        if self._sw:
            n = len(self._sw["ids"])
        return n if n >= 3 else 4          # nothing/too few ticked -> preview with 4 (square)

    @staticmethod
    def _draw_shape_icon(cv, shape, n, color, w, h):
        cv.delete("all")
        n = max(2, min(n, 8))
        pts = formation_slots(n, 1.0, shape)
        xs = [r for f, r in pts]
        ys = [-f for f, r in pts]
        span_x = (max(xs) - min(xs)) or 1.0
        span_y = (max(ys) - min(ys)) or 1.0
        sc = min((w - 12) / span_x, (h - 12) / span_y, 40.0)
        cx, cy = (max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0
        P = [(w / 2.0 + (x - cx) * sc, h / 2.0 + (y - cy) * sc) for x, y in zip(xs, ys)]
        if shape == "polygon" and n >= 3:
            order = list(range(n))
            order.sort(key=lambda i: math.atan2(P[i][1] - h / 2.0, P[i][0] - w / 2.0))
            cv.create_polygon([c for i in order for c in P[i]], outline=color, fill="", width=2)
        elif shape == "wedge":
            for i in range(1, n):
                cv.create_line(*P[0], *P[i], fill=color, width=1)
            for i in range(1, n - 1):
                cv.create_line(*P[i], *P[i + 1], fill=color, width=1)
        else:
            cv.create_line(*[c for p in P for c in p], fill=color, width=2)
        for x, y in P:
            cv.create_oval(x - 3, y - 3, x + 3, y + 3, fill=color, outline="")

    def _form_icons_tick(self):
        try:
            n = self._form_preview_count()
            key = (n, self.swarm_form_shape)
            if key != self._form_icon_key:
                self._form_icon_key = key
                for shp, (cell, cv, lb) in self._form_shape_btns.items():
                    on = (shp == self.swarm_form_shape)
                    col = COL_VALUE if on else COL_TEXT_DIM
                    cell.config(highlightbackground=COL_VALUE if on else COL_BORDER)
                    lb.config(fg=COL_VALUE if on else COL_TEXT_DIM)
                    self._draw_shape_icon(cv, shp, n, col, 44, 34)
                self._draw_shape_icon(self._form_head_icon, self.swarm_form_shape, n, COL_VALUE, 22, 18)
        except (tk.TclError, AttributeError):
            return
        self.root.after(400, self._form_icons_tick)

    def _on_pick_formation_shape(self, key):
        self.swarm_form_shape = key
        self._form_icon_key = None
        name = dict(FORMATION_SHAPES)[key]
        msg = f">>> Swarm formation shape set to {name}"
        if self._sw:
            msg += " (applies from the next AUTO MODE -- not changed mid-flight)"
        self._append_log(msg)

    def _set_form_status(self, text, color=None):
        try:
            self.form_status.config(text=text, fg=color or COL_TEXT_DIM)
        except (tk.TclError, AttributeError):
            pass

    def _refresh_form_status(self):
        if self._sw:
            return
        if self.swarm_form_d:
            self._set_form_status(f"\u25cf ACTIVE  \u00b7  {self.swarm_form_d:g} m", COL_VALUE)
        else:
            self._set_form_status("\u25cb NOT SET", COL_TEXT_DIM)

    def _on_set_formation_distance(self):
        raw = self.form_entry.get().strip()
        try:
            d = float(raw)
        except ValueError:
            messagebox.showerror("Distance between", "Enter the distance in metres, e.g. 5")
            return
        if d < 0 or d > 200:
            messagebox.showerror("Distance between", "Enter a value between 0 and 200 metres.")
            return
        if d == 0:
            self.swarm_form_d = None
            self._append_log(">>> Swarm formation distance cleared (formation off)")
        else:
            self.swarm_form_d = d
            msg = f">>> Swarm formation distance set to {d:g} m"
            if self._sw:
                msg += " (applied live to the running formation)"
                self._swarm_bus_send({"k": "param", "mid": self._sw["mid"], "d": d}, self._sw["ids"])
            self._append_log(msg)
            if d < 3.0:
                self._append_log("!!! Under 3 m is tight for GPS-based flight -- expect little margin")
        self.form_entry.selection_clear()
        self.nav_panel.frame.focus_set()
        self._refresh_form_status()


##############################################################################
# SECTION: mission.py   (was gcs/app/swarm/mission.py)
##############################################################################
class SwarmMissionMixin:
    """AUTO MODE: start the swarm mission, send mission to agents, stop."""

    # ---------------- start / stop ----------------
    def _swarm_auto_start(self, sysids):
        if self._sw:
            self._append_log("!!! Swarm auto is already running -- ABORT MISSION to stop it")
            return
        if self._bus is None:
            self._append_log("!!! Swarm bus is not available -- cannot start swarm AUTO")
            return
        alt, speed = self._get_alt_speed()
        if alt is None:
            return
        primary = sysids[0]
        route = list(self.manual_waypoints.get(primary, []))
        if not route:
            messagebox.showerror(
                "No destination",
                "Click MARK WAYPOINTS and click the map to mark the destination point first\n"
                "(with all the swarm drones ticked).")
            return
        missing = [self._letter_for_sysid(s) for s in sysids if s not in self.swarm_positions]
        if missing:
            self._append_log(f"!!! No GPS position yet for Drone {', '.join(missing)} -- cannot start")
            return
        now = time.time()
        busy = [self._letter_for_sysid(s) for s in sysids
                if s in self._bus_states and now - self._bus_states[s]["rx"] < 2.0
                and self._bus_states[s].get("phase") in ("PREP", "WAIT", "FORM", "TRANSIT", "SETTLE")]
        if busy:
            self._append_log(f"!!! Agent of Drone {', '.join(busy)} is still running a mission -- "
                             f"press ABORT MISSION first")
            return
        self._swarm_clear_trails()
        self._swarm_triangle_clear()
        self._sw = {
            "ids": list(sysids), "d": self.swarm_form_d, "shape": self.swarm_form_shape, "alt": alt,
            "speed": max(0.5, speed / 3.6), "route": route,
            "o": list(self.swarm_positions[primary]),
            "mid": int(now * 1000) % (2 ** 31), "phase": "LAUNCH",
            "t0": now, "t_run": now, "t_send": 0.0, "t_stop": None,
            "dotted": False, "route_path": None, "route_upd": 0.0, "poll_n": 0,
        }
        letters = ", ".join(self._letter_for_sysid(s) for s in sysids)
        self._append_log(f">>> SWARM AUTO: Drone(s) {letters} | spacing {self.swarm_form_d:g} m | "
                         f"alt {alt:g} m | {len(route)} waypoint(s) -- handing the mission to the drones' agents")
        online = self._swarm_agents_online(sysids, now)
        for sid in sysids:
            if sid not in online:
                self._swarm_launch_agent(sid)
        self._tri_ids = list(sysids)
        self._tri_visible = False
        self._set_form_status("STARTING AGENTS", COL_WARN)

    def _swarm_send_mission(self, sw, targets, now):
        msg = {"k": "mission", "mid": sw["mid"], "ids": sw["ids"], "d": sw["d"], "alt": sw["alt"],
               "speed": sw["speed"], "route": [list(p) for p in sw["route"]], "o": list(sw["o"]),
               "shape": sw.get("shape", "polygon")}
        self._swarm_bus_send(msg, targets)
        sw["t_send"] = now

    def _swarm_auto_stop(self, reason, arrived=False):
        """Ends the GCS-side monitoring. (The agents decide for themselves what to do; the GCS
        sends them nothing here -- ABORT / RTL are handled in _broadcast.)"""
        sw = self._sw
        if not sw:
            return
        self._last_sw = dict(sw)
        self._sw = None
        if sw.get("route_path"):
            try:
                sw["route_path"].delete()
            except Exception:
                pass
        if sw.get("dotted"):
            self._redraw_manual_path(sw["ids"][0])     # the solid planned route is back
        if arrived:
            self._append_log(f">>> SWARM AUTO: {reason}")
            if sw.get("is_rtl"):
                self._set_form_status("\u2714 RTL COMPLETE \u00b7 landed on the home points", COL_VALUE)
            else:
                self._set_form_status("\u2714 ARRIVED \u00b7 formation held", COL_VALUE)
            self.root.after(8000, self._refresh_form_status)
        else:
            self._append_log(f"!!! SWARM AUTO stopped: {reason}")
            self._refresh_form_status()


##############################################################################
# SECTION: rtl.py   (was gcs/app/swarm/rtl.py)
##############################################################################
class SwarmRtlMixin:
    """Swarm RTL button logic."""

    def _on_swarm_rtl(self):
        """Commands the swarm to return to launch in formation, maintaining spacing and
        collision avoidance instead of breaking formation with individual autopilot RTL."""
        sw = self._sw
        last = getattr(self, "_last_sw", None)
        active_sw = sw or last
        selected = self._selected_sysids(warn=False)

        if not active_sw and (not self.swarm_form_d or len(selected) < 2):
            return False

        ids = list(active_sw["ids"]) if active_sw else list(selected)
        if len(ids) < 2:
            return False

        if self._bus is None:
            return False

        primary = ids[0]
        o = active_sw.get("o") if active_sw else (list(self.swarm_positions[primary]) if primary in self.swarm_positions else None)
        if not o:
            return False

        # Already returning and breaking formation / landing? Then a second RTL press must not restart anything.
        if active_sw and active_sw.get("is_rtl") and any(
                self._bus_states.get(s, {}).get("ls") not in (None, "") for s in ids):
            self._append_log(">>> RTL is already in progress -- the drones are landing on their home points")
            return True

        # RTL: fly DIRECTLY back (no backtracking through previous waypoints). The formation returns to the
        # centre of all the HOME points; there it is broken and each drone lands on its OWN home point.
        homes = [self._bus_states.get(s, {}).get("home") for s in ids]
        if all(homes):
            rtl_target = [sum(h[0] for h in homes) / len(homes), sum(h[1] for h in homes) / len(homes)]
        else:
            rtl_target = list(o)
        rtl_route = [rtl_target]

        d = active_sw.get("d", self.swarm_form_d or 10.0) if active_sw else (self.swarm_form_d or 10.0)
        shape = active_sw.get("shape", self.swarm_form_shape or "polygon") if active_sw else (self.swarm_form_shape or "polygon")
        alt = active_sw.get("alt") if active_sw else None
        speed = active_sw.get("speed") if active_sw else None
        if alt is None or speed is None:
            alt_calc, spd_calc = self._get_alt_speed()
            alt = alt or alt_calc or 10.0
            speed = speed or (max(0.5, spd_calc / 3.6) if spd_calc else 4.0)

        # Climb 5 m above the mission altitude for the return (a repeated RTL press keeps the same altitude)
        rtl_alt = float(alt) if (active_sw and active_sw.get("is_rtl")) else float(alt) + 5.0

        now = time.time()
        rtl_mid = int(now * 1000) % (2 ** 31)

        # Clear existing dotted route if any
        if sw and sw.get("route_path"):
            try:
                sw["route_path"].delete()
            except Exception:
                pass

        # Create new swarm monitoring state for the RTL phase (alt = rtl_alt so GCS tracks climb)
        self._sw = {
            "ids": list(ids), "d": d, "shape": shape, "alt": rtl_alt,
            "speed": speed, "route": rtl_route, "o": list(o),
            "mid": rtl_mid, "phase": "RUN",
            "t0": now, "t_run": now, "t_send": now, "t_stop": None,
            "dotted": True, "route_path": None, "route_upd": 0.0, "poll_n": 0,
            "is_rtl": True,
        }

        # Draw dotted return path directly from lead drone's current position to the return point
        lead_pos = self.swarm_positions.get(primary) or o
        pts = [tuple(lead_pos), tuple(rtl_target)]          # straight line: current pos -> return point
        self._sw["route_path"] = self._make_dotted_path(pts)

        msg = {
            "k": "rtl",
            "mid": rtl_mid,
            "ids": ids,
            "d": d,
            "alt": rtl_alt,      # agents will climb to this altitude before translating
            "speed": speed,
            "route": rtl_route,  # [centre of the home points] - direct return
            "o": list(o),
            "shape": shape,
        }
        self._swarm_bus_send(msg, ids)

        letters = ", ".join(self._letter_for_sysid(s) for s in ids)
        self._append_log(f">>> SWARM RTL: Drone(s) {letters} | climb +5 m, fly back in formation, then land "
                         f"one by one on the home points | spacing {d:g} m | alt {rtl_alt:g} m")
        self._set_form_status(f"RTL \u00b7 RETURNING IN FORMATION ({d:g} m)", COL_VALUE)
        self._tri_ids = list(ids)
        self._tri_visible = True

        for sysid in ids:
            self._cmd_push(sysid, "GCS", "SWARM RTL")
        return True


##############################################################################
# SECTION: supervisor.py   (was gcs/app/swarm/supervisor.py)
##############################################################################
class SwarmSupervisorMixin:
    """Per-tick mission supervision + formation triangle overlay on the map."""

    # ---------------- supervision (read-only window onto the agents) ----------------
    def _swarm_supervise(self, now):
        sw = self._sw
        if not sw:
            return
        ids, n = sw["ids"], len(sw["ids"])
        PH = {p: i for i, p in enumerate(self._SW_PHASES)}

        # ---- 1. agents must be online, then they get the mission ----
        if sw["phase"] == "LAUNCH":
            online = self._swarm_agents_online(ids, now)
            self._set_form_status(f"STARTING AGENTS \u00b7 {len(online)}/{n} online", COL_WARN)
            if len(online) == n:
                sw["phase"] = "RUN"
                sw["t_run"] = now
                self._swarm_send_mission(sw, ids, now)
            elif now - sw["t0"] > 40:
                late = ", ".join(self._letter_for_sysid(s) for s in ids if s not in online)
                self._swarm_auto_stop(f"agent for Drone {late} did not come online in 40 s "
                                      f"(is SITL running? see swarm_agent_<id>.log)")
            return

        mine = {sid: self._bus_states[sid] for sid in ids
                if sid in self._bus_states and self._bus_states[sid].get("mid") == sw["mid"]}
        if len(mine) < n:                                   # mission not accepted by everybody yet
            self._set_form_status("HANDING OVER RTL" if sw.get("is_rtl") else "HANDING OVER MISSION", COL_WARN)
            if now - sw["t_run"] > 10:
                late = ", ".join(self._letter_for_sysid(s) for s in ids if s not in mine)
                self._swarm_auto_stop(f"Drone {late} did not accept the {'RTL' if sw.get('is_rtl') else 'mission'} (agent busy?)")
            elif now - sw["t_send"] > 0.5:
                if sw.get("is_rtl"):
                    msg = {"k": "rtl", "mid": sw["mid"], "ids": sw["ids"], "d": sw["d"], "alt": sw["alt"],
                           "speed": sw["speed"], "route": [list(p) for p in sw["route"]], "o": list(sw["o"]),
                           "shape": sw.get("shape", "polygon")}
                    self._swarm_bus_send(msg, [s for s in ids if s not in mine])
                    sw["t_send"] = now
                else:
                    self._swarm_send_mission(sw, [s for s in ids if s not in mine], now)
            return

        phases = {sid: mine[sid].get("phase", "IDLE") for sid in ids}
        d = sw["d"]
        lead = mine[ids[0]]

        # ---- 2. map: covered trail (yellow) + planned route turns dotted once forming ----
        sw["poll_n"] += 1
        for sid in ids:
            st = mine[sid]
            pts = self.swarm_trail_pts.setdefault(sid, [])
            if self.gps_enabled and now - st["rx"] < 1.0 and (not pts or math.hypot(*self._ll_to_ne(pts[-1], st["lat"], st["lon"])) > 0.4):
                pts.append((st["lat"], st["lon"]))
        if sw["poll_n"] % 2 == 0:
            for sid in ids:
                pts = self.swarm_trail_pts.get(sid, [])
                if len(pts) < 2:
                    continue
                path = self.swarm_trail_paths.get(sid)
                if path is None or getattr(path, "deleted", False):
                    self.swarm_trail_paths[sid] = self.map_widget.set_path(list(pts), color=COL_TRAJECTORY, width=3)
                else:
                    path.set_position_list(list(pts))
        if not sw["dotted"] and min(PH.get(p, 0) for p in phases.values()) >= PH["FORM"] and \
                max(PH.get(p, 0) for p in phases.values()) < PH["STOPPED"]:
            sw["dotted"] = True
            old = self.manual_path.get(ids[0])
            if old:
                old.delete()
                self.manual_path[ids[0]] = None
            sw["route_path"] = self._make_dotted_path([tuple(sw["o"])] + list(sw["route"]))
            self._append_log(f">>> SWARM AUTO: all drones at altitude -- forming the triangle ({d:g} m between drones)")
        if sw["route_path"] and phases[ids[0]] == "TRANSIT" and now - sw["route_upd"] > 1.0:
            sw["route_upd"] = now
            try:
                R = lead.get("R", [0.0, 0.0])
                wi = int(lead.get("wi", 0))
                sw["route_path"].set_position_list(
                    [self._ne_to_ll(sw["o"], R[0], R[1])] + list(sw["route"][wi:]))
            except Exception:
                pass

        # ---- 3. has the mission ended? ----
        rtl = bool(sw.get("is_rtl"))
        landed = [sid for sid in ids if rtl and phases[sid] == "STOPPED"
                  and str(mine[sid].get("why") or "").startswith("RTL landed")]
        if rtl and len(landed) == n:
            self._swarm_auto_stop("return to launch complete -- every drone landed on its own home point", arrived=True)
            return
        stopped = [sid for sid in ids if phases[sid] == "STOPPED" and sid not in landed]
        if stopped:
            sw["t_stop"] = sw["t_stop"] or now
            if all(phases[s] in ("STOPPED", "DONE") for s in ids) or now - sw["t_stop"] > 3.0:
                first = mine[stopped[0]]
                self._swarm_auto_stop(f"Drone {self._letter_for_sysid(stopped[0])}: {first.get('why') or 'stopped'}")
            return
        silent = [sid for sid in ids if now - self._bus_states.get(sid, {}).get("rx", 0) > 4.0]
        if silent:
            letters = ", ".join(self._letter_for_sysid(s) for s in silent)
            self._swarm_auto_stop(f"lost the agent of Drone {letters} (no report for 4 s)")
            return
        if all(phases[s] == "DONE" for s in ids) and not rtl:
            self._swarm_auto_stop(f"destination reached, triangle held at {d:g} m", arrived=True)
            return
        # (RTL: the drones are above the launch area -- they now break the formation and land one by one,
        #  so the GCS keeps watching until every drone reports "RTL landed".)

        # ---- 4. status line ----
        low = min(phases.values(), key=lambda p: PH.get(p, 0))
        if low == "PREP":
            up = sum(1 for p in phases.values() if p != "PREP")
            self._set_form_status(f"TAKING OFF \u00b7 {up}/{n} at altitude", COL_WARN)
        elif low == "WAIT":
            self._set_form_status("WAITING FOR ALL DRONES", COL_WARN)
        elif low == "FORM":
            self._set_form_status(f"FORMING TRIANGLE \u00b7 {int(lead.get('inpos', 0))}/{n} in position", COL_WARN)
        elif low == "TRANSIT":
            if sw.get("is_rtl"):
                self._set_form_status(f"RTL EN ROUTE \u00b7 {float(lead.get('togo', 0.0)):.0f} m to launch \u00b7 {d:g} m spacing", COL_VALUE)
            else:
                self._set_form_status(f"EN ROUTE \u00b7 {float(lead.get('togo', 0.0)):.0f} m to go \u00b7 {d:g} m spacing", COL_VALUE)
        elif rtl and low in ("DONE", "STOPPED"):
            self._set_form_status(f"RTL \u00b7 LANDING ONE BY ONE ON HOME POINTS \u00b7 {len(landed)}/{n} landed", COL_VALUE)
        else:
            if sw.get("is_rtl"):
                self._set_form_status(f"RTL ARRIVING \u00b7 {int(lead.get('inpos', 0))}/{n} in position", COL_VALUE)
            else:
                self._set_form_status(f"ARRIVING \u00b7 {int(lead.get('inpos', 0))}/{n} in position", COL_VALUE)

    # ---------------- green triangle with live distances ----------------
    def _swarm_triangle_clear(self):
        for obj in list(self._tri_lines.values()) + list(self._tri_labels.values()):
            try:
                obj.delete()
            except Exception:
                pass
        self._tri_lines = {}
        self._tri_labels = {}
        self._tri_ids = None
        self._tri_visible = False

    def _swarm_triangle_update(self, now):
        """Green formation lines and distance labels are intentionally disabled."""
        return


##############################################################################
# SECTION: nav_panel.py   (was gcs/app/swarm/nav_panel.py)
##############################################################################
class NavPanelMixin:
    """Swarm Navigation panel: window, slide animation, card/log helpers."""

    # ---------------- SWARM NAV slide-in panel ----------------
    def _build_swarm_nav_panel(self):
        """Right-hand panel that slides in from the right edge up to the
        centre of the screen (covering what is behind it) and slides back
        out with the '>>' button. Five swarm-wide cards: mission progress, who-hears-whom
        links, shared state, autopilot commands and the live bus feed.
        """
        self._nav_prog = 0.0          # 0 = fully out (off-screen right), 1 = fully in
        self._nav_target = 0
        self._nav_job = None
        self._nav_refresh_job = None
        self._nav_sig = None
        self._nav_sections = {}       # sysid -> {"own": {key: Label}, "recv": {other_sysid: {key: Label}}}

        panel_w = lambda W, H: max(560, int(W * 0.5) - 8)
        p = GlassPanel(self, bg=COL_BTN, border=COL_BORDER, alpha=0.97,
                       start_visible=False, popup=True)
        p.set_geometry(
            relx=1.0, rely=0.0, anchor="ne",
            dx=lambda W, H: -8 + int((1.0 - self._nav_prog) * (panel_w(W, H) + 30)),
            dy=8, width=panel_w,
            height=lambda W, H: max(260, H - self.banner_panel.frame.winfo_reqheight() - 30))
        self.nav_panel = p

        head = tk.Frame(p.body, bg=COL_BTN)
        head.pack(fill="x", padx=12, pady=(12, 6))
        back = tk.Label(head, text=">>", font=("Segoe UI", 13, "bold"), fg=COL_TEXT,
                        bg=COL_BORDER, padx=14, pady=2, cursor="hand2")
        back.pack(side="left")
        back.bind("<Enter>", lambda e: back.config(bg="#4a4a50"))
        back.bind("<Leave>", lambda e: back.config(bg=COL_BORDER))
        back.bind("<Button-1>", lambda e: self._nav_slide(0))
        tk.Label(head, text="SWARM NAV", font=("Segoe UI", 15, "bold"), fg=COL_TEXT,
                 bg=COL_BTN).pack(side="left", padx=14)

        # Bottom of the panel is packed FIRST so it stays fixed (never scrolls):
        # formation bar lowest, the small note just above it, scroll area gets the rest.
        self._build_formation_bar(p.body)
        tk.Label(p.body, text="Everything above comes from the agents' real UDP swarm-bus traffic (agent N listens on "
                              "port 15000+N). Before AUTO MODE starts, drones show plain telemetry and are marked 'agent offline'.",
                 font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=COL_BTN, wraplength=520,
                 justify="left").pack(side="bottom", anchor="w", padx=14, pady=(0, 2))

        wrap = tk.Frame(p.body, bg=COL_BTN)
        wrap.pack(fill="both", expand=True, padx=10, pady=(0, 4))
        self._nav_canvas = tk.Canvas(wrap, bg=COL_BTN, highlightthickness=0)
        sb = tk.Scrollbar(wrap, orient="vertical", command=self._nav_canvas.yview)
        self._nav_canvas.configure(yscrollcommand=sb.set)
        self._nav_canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self._nav_container = tk.Frame(self._nav_canvas, bg=COL_BTN)
        win = self._nav_canvas.create_window((0, 0), window=self._nav_container, anchor="nw")
        self._nav_container.bind(
            "<Configure>", lambda e: self._nav_canvas.configure(scrollregion=self._nav_canvas.bbox("all")))
        self._nav_canvas.bind(
            "<Configure>", lambda e: self._nav_canvas.itemconfig(win, width=e.width))

        def _wheel(event):
            if getattr(event, "num", None) == 4:
                self._nav_canvas.yview_scroll(-1, "units")
            elif getattr(event, "num", None) == 5:
                self._nav_canvas.yview_scroll(1, "units")
            else:
                self._nav_canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")

        def _bind(_e):
            for ev in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
                self._nav_canvas.bind_all(ev, _wheel)

        def _unbind(_e):
            for ev in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
                self._nav_canvas.unbind_all(ev)

        self._nav_canvas.bind("<Enter>", _bind)
        self._nav_canvas.bind("<Leave>", _unbind)

    def _toggle_swarm_nav(self):
        if not self._nav_target and getattr(self, "_odo_target", 0):
            self._odo_slide(0)                    # SWARM NAV and VISUAL ODO share the same spot
        self._nav_slide(0 if self._nav_target else 1)

    def _nav_slide(self, target):
        self._nav_target = target
        if target:
            self.nav_panel.show()
            self._nav_refresh()
        if self._nav_job is None:
            self._nav_job = self.root.after(0, self._nav_step)

    def _nav_step(self):
        self._nav_job = None
        tgt, cur = self._nav_target, self._nav_prog
        delta = tgt - cur
        if abs(delta) < 0.012:
            prog = float(tgt)
        else:
            step = delta * 0.25
            if abs(step) < 0.012:
                step = 0.012 if delta > 0 else -0.012
            prog = cur + step
        self._nav_prog = prog
        self.nav_panel.apply()
        if tgt:
            self.nav_panel.lift()
        if prog != tgt:
            self._nav_job = self.root.after(16, self._nav_step)
        elif tgt == 0:
            self.nav_panel.hide()

    # ======================================================================
    #  SWARM NAV content (v5)
    #  No per-drone sections any more.  Five cards that explain the swarm as a whole:
    #    1 mission progress  - which step of the mission every drone is in
    #    2 communication     - who hears whom: rate / age / packet count of every link
    #    3 shared state      - what every agent broadcasts, side by side, + consensus check
    #    4 autopilot cmds    - what each agent really sends to its own ArduPilot
    #    5 live bus feed     - every message / event, in plain words
    # ======================================================================
    _NAV_CARD = "#232326"
    _NAV_TILE = "#1b1b1e"
    _NAV_DIM_CHIP = "#26262a"
    _NAV_PIPE = ["PREP", "WAIT", "FORM", "TRANSIT", "SETTLE", "DONE"]
    _NAV_PHASE_HELP = {
        "IDLE": "idle - waiting for a mission",
        "PREP": "switching to GUIDED, arming and taking off",
        "WAIT": "at altitude, waiting until every drone is ready",
        "FORM": "flying to its own slot in the triangle",
        "TRANSIT": "flying the route while holding the triangle shape",
        "SETTLE": "braking and holding position at the destination",
        "DONE": "mission complete - formation held",
        "STOPPED": "stopped",
    }
    _NAV_FIELDS = [("pos", "Position"), ("alt", "Altitude"), ("vel", "Velocity"), ("speed", "Ground speed"),
                   ("hdg", "Heading"), ("mode", "Flight mode"), ("armed", "Armed"),
                   ("phase", "Swarm phase"), ("slot", "Formation slot"), ("R", "Formation ref R"),
                   ("vr", "Ref speed"), ("wp", "Route progress"), ("emax", "Worst slot error")]
    _NAV_MONO = ("DejaVu Sans Mono", 8)
    # professional palette: neutral greys + one steel-blue accent, muted status colours
    _NV_ACC = "#6c9bd2"        # accent (headings bar, current stage, sent traffic)
    _NV_VAL = "#d8dce3"        # data values
    _NV_OK = "#5fae8f"         # healthy
    _NV_WARN = "#c9a05a"       # degraded / caution
    _NV_BAD = "#c26d6d"        # lost / stopped
    _NV_OFF = "#59606c"        # not running
    _NV_PAST_FG, _NV_PAST_BG = "#8fb0d6", "#26303d"
    _NV_ON_ACC = "#0f141b"     # text on accent fill
    _NV_DRONE = ["#6c9bd2", "#58b3a8", "#a393d6", "#9aa5b8", "#c99ac0"]   # drone identity dots
    COL_NAV_RX = "#a3b1c6"
    COL_NAV_GCS = "#c9ced8"

    def _nv_dc(self, sid):
        try:
            return self._NV_DRONE[self.swarm_order.index(sid) % len(self._NV_DRONE)]
        except ValueError:
            return self._NV_DRONE[0]

    # ---------- small widget helpers ----------
    def _nav_card(self, title, subtitle=""):
        CARD = self._NAV_CARD
        fr = tk.Frame(getattr(self, "_nav_parent", self._nav_container), bg=CARD, highlightthickness=1,
                      highlightbackground=COL_BORDER)
        fr.pack(fill="x", pady=6, padx=2)
        hd = tk.Frame(fr, bg=CARD)
        hd.pack(fill="x", padx=12, pady=(10, 0) if title else (6, 0))
        if title:
            tk.Frame(hd, bg=self._NV_ACC, width=3, height=16).pack(side="left", padx=(0, 8))
            tk.Label(hd, text=title, font=("Segoe UI", 11, "bold"), fg=COL_TEXT, bg=CARD).pack(side="left")
        badge = tk.Label(hd, text="", font=("Segoe UI", 9, "bold"), fg=COL_TEXT_DIM, bg=CARD)
        badge.pack(side="right")
        if subtitle:
            sub = tk.Label(fr, text=subtitle, font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=CARD,
                           anchor="w", justify="left", wraplength=600)
            sub.pack(fill="x", padx=12, pady=(2, 0))
            self._nav_wraps.append(sub)
        if title:
            tk.Frame(fr, bg=COL_BORDER, height=1).pack(fill="x", padx=12, pady=(7, 8))
        body = tk.Frame(fr, bg=CARD)
        body.pack(fill="x", padx=10, pady=(0, 10))
        return body, badge

    def _nav_name(self, parent, sid, bg):
        f = tk.Frame(parent, bg=bg)
        tk.Label(f, text=f"\u25cf Drone {self._letter_for_sysid(sid)}", font=("Segoe UI", 10, "bold"),
                 fg=self._nv_dc(sid), bg=bg).pack(side="left")
        tk.Label(f, text=f"sysid {sid}", font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=bg).pack(side="left", padx=(5, 0))
        return f

    def _nav_cell(self, parent, row, col):
        TILE = self._NAV_TILE
        f = tk.Frame(parent, bg=TILE, highlightthickness=1, highlightbackground=COL_BORDER)
        f.grid(row=row, column=col, sticky="nsew", padx=3, pady=3)
        top = tk.Label(f, text="--", font=("Segoe UI", 10, "bold"), fg=COL_TEXT_DIM, bg=TILE, anchor="w")
        top.pack(fill="x", padx=8, pady=(5, 0))
        bot = tk.Label(f, text="", font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=TILE, anchor="w", justify="left")
        bot.pack(fill="x", padx=8, pady=(0, 5))
        return {"f": f, "top": top, "bot": bot}

    def _nav_make_log(self, parent, height):
        box = tk.Frame(parent, bg=self._NAV_CARD)
        box.pack(fill="x", padx=2, pady=(2, 0))
        t = tk.Text(box, height=height, bg="#111113", fg=COL_TEXT, font=self._NAV_MONO,
                    relief="flat", highlightthickness=1, highlightbackground=COL_BORDER,
                    wrap="none", state="disabled", cursor="arrow", padx=6, pady=4)
        t.pack(side="left", fill="both", expand=True)
        # no scrollbar and no wheel-scrolling inside the box: the log always shows the newest lines,
        # and the mouse wheel over it scrolls the whole SWARM NAV panel instead
        t.bindtags(tuple(tg for tg in t.bindtags() if tg != "Text"))
        t.tag_configure("tx", foreground=self._NV_ACC)
        t.tag_configure("rx", foreground=self.COL_NAV_RX)
        t.tag_configure("ev", foreground=self._NV_WARN)
        t.tag_configure("gcs", foreground=self.COL_NAV_GCS)
        t.tag_configure("dim", foreground=COL_TEXT_DIM)
        for sid in self.swarm_order:
            t.tag_configure(f"dr{sid}", foreground=self._nv_dc(sid), font=(self._NAV_MONO[0], self._NAV_MONO[1], "bold"))
        return t

    def _nav_log_append(self, widget, rows):
        """rows = list of lines, every line a list of (text, tag) segments."""
        at_bottom = widget.yview()[1] >= 0.995          # follow new lines only if the user is not reading old ones
        widget.config(state="normal")
        for segs in rows:
            for text, tag in segs:
                widget.insert("end", text, tag)
            widget.insert("end", "\n")
        n = int(widget.index("end-1c").split(".")[0])
        if n > 3000:                                   # long history: ~3000 lines are kept
            widget.delete("1.0", f"{n - 2500}.0")
        if at_bottom:
            widget.see("end")
        widget.config(state="disabled")

    def _nav_legend(self, parent, items):
        f = tk.Frame(parent, bg=self._NAV_CARD)
        for color, text in items:
            tk.Label(f, text="\u25cf", font=("Segoe UI", 9), fg=color, bg=self._NAV_CARD).pack(side="left", padx=(0, 2))
            tk.Label(f, text=text, font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=self._NAV_CARD).pack(side="left", padx=(0, 12))
        return f


##############################################################################
# SECTION: nav_rebuild.py   (was gcs/app/swarm/nav_rebuild.py)
##############################################################################
class NavRebuildMixin:
    """Swarm Navigation panel: (re)build the per-drone cards."""

    # ---------- build ----------
    def _nav_rebuild(self):
        for w in self._nav_container.winfo_children():
            w.destroy()
        self._nav_wraps = []
        self._nav_parent = self._nav_container
        order = list(self.swarm_order)
        nv = self._nav_v = {"order": order}
        CARD, TILE = self._NAV_CARD, self._NAV_TILE
        if not order:
            tk.Label(self._nav_container, text="No drones connected yet.",
                     font=("Segoe UI", 11), fg=COL_TEXT_DIM, bg=COL_BTN).pack(pady=30)
            return

        # ---- A. what is happening right now ----
        body, nv["bA"] = self._nav_card("")
        nv["head"] = tk.Label(body, text="", font=("Segoe UI", 12, "bold"), fg=COL_TEXT, bg=CARD,
                              anchor="w", justify="left", wraplength=600)
        nv["head"].pack(fill="x", padx=2, pady=(0, 2))
        nv["sub"] = tk.Label(body, text="", font=("Segoe UI", 9), fg=COL_TEXT_DIM, bg=CARD,
                             anchor="w", justify="left", wraplength=600)
        nv["sub"].pack(fill="x", padx=2, pady=(0, 8))
        self._nav_wraps += [nv["head"], nv["sub"]]
        hrow = tk.Frame(body, bg=CARD)
        hrow.pack(fill="x", pady=(0, 8))
        nv["health"] = {}
        for key, cap in (("comms", "COMMUNICATION"), ("form", "FORMATION"), ("agree", "AGREEMENT")):
            hf = tk.Frame(hrow, bg=TILE, highlightthickness=1, highlightbackground=COL_BORDER)
            hf.pack(side="left", fill="x", expand=True, padx=(0, 6))
            tk.Label(hf, text=cap, font=("Segoe UI", 8, "bold"), fg=COL_TEXT_DIM, bg=TILE,
                     anchor="w").pack(fill="x", padx=9, pady=(6, 0))
            v = tk.Label(hf, text="--", font=("Segoe UI", 10, "bold"), fg=self._NV_OFF, bg=TILE, anchor="w")
            v.pack(fill="x", padx=9, pady=(0, 1))
            d = tk.Label(hf, text="", font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=TILE, anchor="w", justify="left")
            d.pack(fill="x", padx=9, pady=(0, 6))
            nv["health"][key] = (v, d)
        srow = tk.Frame(body, bg=TILE)
        srow.pack(fill="x")
        tk.Label(srow, text="MISSION STAGE", font=("Segoe UI", 8, "bold"), fg=COL_TEXT_DIM, bg=TILE
                 ).pack(side="left", padx=(9, 12), pady=7)
        nv["chips"] = {}
        for ph in self._NAV_PIPE + ["STOPPED"]:
            c = tk.Label(srow, text=ph, font=("Segoe UI", 8, "bold"), padx=7, pady=2, bg=TILE, fg=TILE)
            c.pack(side="left", padx=2)
            nv["chips"][ph] = c
        nv["stage_note"] = tk.Label(body, text="", font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=CARD,
                                    anchor="w", justify="left", wraplength=600)
        nv["stage_note"].pack(fill="x", padx=2, pady=(4, 0))
        self._nav_wraps.append(nv["stage_note"])

        # ---- B. live formation picture ----
        body, nv["bB"] = self._nav_card(
            "LIVE FORMATION \u2014 top view",
            "Drones as the agents see them, turned so the direction of flight points up. Line colour = how close the gap is "
            "to the target spacing; label = measured distance \u00b7 message rate between the two agents. "
            "Arrow = velocity, \u25c7 = shared formation reference point.")
        nv["cv"] = tk.Canvas(body, height=270, bg=TILE, highlightthickness=1, highlightbackground=COL_BORDER)
        nv["cv"].pack(fill="x", padx=2)

        # ---- C. do the agents agree? ----
        body, nv["bC"] = self._nav_card("")
        body.grid_columnconfigure(2, weight=1)
        nv["chk"] = {}
        self._NAV_CHECKS = [
            ("spacing", "Spacing", "gap between every pair vs the target"),
            ("speed", "Speed", "all drones fly at the same speed"),
            ("heading", "Heading", "all drones point the same way"),
            ("alt", "Altitude", "all drones at the same height"),
            ("ref", "Formation reference", "each agent's own copy of the shared reference point"),
            ("fresh", "Message freshness", "slowest link rate / oldest packet"),
            ("phase", "Mission stage", "every agent is in the same stage"),
        ]
        for i, (key, cap, why) in enumerate(self._NAV_CHECKS):
            bg = TILE if i % 2 == 0 else CARD
            ic = tk.Label(body, text="\u2014", font=("Segoe UI", 11, "bold"), fg=self._NV_OFF, bg=bg, width=3)
            ic.grid(row=i, column=0, sticky="nsew")
            tk.Label(body, text=cap, font=("Segoe UI", 9, "bold"), fg=COL_TEXT, bg=bg, anchor="w", width=19,
                     padx=4, pady=5).grid(row=i, column=1, sticky="nsew")
            val = tk.Label(body, text="--", font=("Segoe UI", 9), fg=self._NV_VAL, bg=bg, anchor="w", padx=6)
            val.grid(row=i, column=2, sticky="nsew")
            tk.Label(body, text=why, font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=bg, anchor="e",
                     padx=8).grid(row=i, column=3, sticky="nsew")
            nv["chk"][key] = (ic, val)

        # ---- D. activity feed (bus messages + autopilot commands, one timeline) ----
        body, nv["bD"] = self._nav_card(
            "ACTIVITY \u2014 what the agents said and did",
            "One timeline: messages between agents (BUS) and commands sent to the autopilots (CMD).")
        lg = self._nav_legend(body, [(self._NV_ACC, "sent / command"), (self.COL_NAV_RX, "received"),
                                     (self._NV_WARN, "decision / stage change"), (self.COL_NAV_GCS, "from the dashboard")])
        lg.pack(anchor="w", padx=2, pady=(0, 4))
        nv["feed"] = self._nav_make_log(body, 20)
        self._nav_feed_text = nv["feed"]
        self._nav_feed_shown = (self._feed[-1000][0] - 1) if len(self._feed) > 1000 else 0
        allq = sorted(q for s in order for q, _t, _x in self._cmd_log.get(s, []))
        nv["cmd_seen"] = (allq[-600] - 1) if len(allq) > 600 else 0
        self._nav_bus_hdr = nv["bD"]

        # ---- E. raw data, collapsed ----
        tg = tk.Label(self._nav_container, text="", font=("Segoe UI", 10, "bold"), fg=self._NV_ACC, bg=COL_BTN,
                      anchor="w", cursor="hand2", padx=12, pady=8)
        tg.pack(fill="x", padx=2, pady=(6, 0))
        raw = tk.Frame(self._nav_container, bg=COL_BTN)
        nv["raw"], nv["raw_tg"] = raw, tg
        def _toggle(_e=None):
            self._nav_raw_open = not self._nav_raw_open
            self._nav_apply_raw()
        tg.bind("<Button-1>", _toggle)
        self._nav_parent = raw
        # (raw cards use the generic card builder, which now packs into the raw frame)
        # ---- 2. communication matrix ----
        body, nv["b2"] = self._nav_card(
            "LINK MATRIX \u2014 who hears whom",
            "Each row is one drone's agent (the listener); each column is a sender. A cell shows how often packets from "
            "that sender really arrive at that listener over the UDP swarm bus, and how fresh they are. "
            "The last column is what the dashboard itself receives from every agent.")
        for c in range(1, len(order) + 2):
            body.grid_columnconfigure(c, weight=1, uniform="lk")
        tk.Label(body, text="LISTENER \u2193    SENDER \u2192", font=("Segoe UI", 8, "bold"),
                 fg=COL_TEXT_DIM, bg=CARD, anchor="w").grid(row=0, column=0, sticky="w", padx=(2, 8))
        for j, s in enumerate(order, start=1):
            self._nav_name(body, s, CARD).grid(row=0, column=j, sticky="w", padx=6, pady=(0, 2))
        tk.Label(body, text="DASHBOARD\n(GCS)", font=("Segoe UI", 10, "bold"), fg=self.COL_NAV_GCS,
                 bg=CARD, justify="left", anchor="w").grid(row=0, column=len(order) + 1, sticky="w", padx=6, pady=(0, 2))
        nv["cells"] = {}
        for i, r in enumerate(order, start=1):
            rf = tk.Frame(body, bg=CARD)
            rf.grid(row=i, column=0, sticky="w", padx=(2, 8))
            tk.Label(rf, text="hears", font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=CARD).pack(side="left", padx=(0, 5))
            self._nav_name(rf, r, CARD).pack(side="left")
            for j, s in enumerate(order, start=1):
                nv["cells"][(r, s)] = self._nav_cell(body, i, j)
            nv["cells"][(r, 0)] = self._nav_cell(body, i, len(order) + 1)
        lg = self._nav_legend(body, [(self._NV_OK, "healthy: \u2265 5 Hz and fresh"), (self._NV_WARN, "degraded"),
                                     (self._NV_BAD, "lost / nothing received"), (self._NV_OFF, "agent not running")])
        lg.grid(row=len(order) + 1, column=0, columnspan=len(order) + 2, sticky="w", padx=2, pady=(6, 0))
        nt = tk.Label(body, text="age = time since the last packet arrived.   \u0394 = distance between the position copy the "
                                 "listener holds and the sender's real position (small = well synchronised).",
                      font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=CARD, anchor="w", justify="left", wraplength=600)
        nt.grid(row=len(order) + 2, column=0, columnspan=len(order) + 2, sticky="w", padx=2, pady=(2, 0))
        self._nav_wraps.append(nt)

        # ---- 3. shared state side by side ----
        body, nv["b3"] = self._nav_card(
            "SHARED STATE \u2014 every agent side by side",
            "The packet each agent sends ~10 times per second to the other agents. All drones side by side, "
            "so you can see at a glance whether they agree.")
        body.grid_columnconfigure(0, weight=0)
        for c in range(1, len(order) + 1):
            body.grid_columnconfigure(c, weight=1, uniform="st")
        nv["col_status"] = {}
        for j, s in enumerate(order, start=1):
            hf = tk.Frame(body, bg=CARD)
            hf.grid(row=0, column=j, sticky="w", padx=8, pady=(0, 4))
            self._nav_name(hf, s, CARD).pack(anchor="w")
            st_l = tk.Label(hf, text="--", font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=CARD, anchor="w", justify="left")
            st_l.pack(anchor="w")
            nv["col_status"][s] = st_l
        nv["vals"] = {}
        for i, (key, cap) in enumerate(self._NAV_FIELDS, start=1):
            bg = TILE if i % 2 else CARD
            tk.Label(body, text=cap, font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=bg, anchor="nw", width=15,
                     padx=8, pady=3).grid(row=i, column=0, sticky="nsew")
            for j, s in enumerate(order, start=1):
                lb = tk.Label(body, text="--", font=("Segoe UI", 9), fg=self._NV_VAL, bg=bg, anchor="nw",
                              justify="left", padx=8, pady=3)
                lb.grid(row=i, column=j, sticky="nsew")
                nv["vals"][(key, s)] = lb
        nv["consensus"] = tk.Label(body, text="", font=("Segoe UI", 9, "bold"), fg=COL_TEXT_DIM, bg=CARD,
                                   anchor="w", justify="left", wraplength=600)
        nv["consensus"].grid(row=len(self._NAV_FIELDS) + 1, column=0, columnspan=len(order) + 1,
                             sticky="w", padx=4, pady=(8, 0))
        self._nav_wraps.append(nv["consensus"])

        # ---- 4. commands to the autopilots ----
        body, nv["b4"] = self._nav_card(
            "AUTOPILOT VELOCITY SET-POINTS",
            "What each agent really tells its own ArduPilot over MAVLink: a velocity set-point ~10 times per second, "
            "plus one-off commands (mode, arm, take-off, speed). [GCS] = sent by this dashboard, [agent] = sent by the drone's agent.")
        for c in range(1, 6):
            body.grid_columnconfigure(c, weight=1, uniform="cm")
        for c, h in enumerate(["", "North", "East", "Down", "Yaw", "Rate"]):
            tk.Label(body, text=h, font=("Segoe UI", 8, "bold"), fg=COL_TEXT_DIM, bg=CARD,
                     anchor="w").grid(row=0, column=c, sticky="w", padx=6)
        nv["cmdv"] = {}
        for i, s in enumerate(order, start=1):
            bg = TILE
            nf = tk.Frame(body, bg=bg)
            nf.grid(row=i, column=0, sticky="nsew", pady=1)
            self._nav_name(nf, s, bg).pack(anchor="w", padx=6, pady=4)
            vals = []
            for c in range(1, 6):
                lb = tk.Label(body, text="--", font=("DejaVu Sans Mono", 9), fg=self._NV_VAL, bg=bg, anchor="w", padx=6)
                lb.grid(row=i, column=c, sticky="nsew", pady=1)
                vals.append(lb)
            nv["cmdv"][s] = vals
        self._nav_parent = self._nav_container
        self._nav_apply_raw()

    def _nav_apply_raw(self):
        nv = self._nav_v
        if nv.get("raw") is None:
            return
        if self._nav_raw_open:
            nv["raw_tg"].config(text="\u25be  Raw data (per-drone values, link matrix, autopilot set-points) \u2014 click to hide")
            nv["raw"].pack(fill="x", after=nv["raw_tg"])
        else:
            nv["raw_tg"].config(text="\u25b8  Raw data (per-drone values, link matrix, autopilot set-points) \u2014 click to show")
            nv["raw"].pack_forget()

    def _nav_card_inner(self, grid_parent, row, ncols):
        """A full-width frame placed on the next grid row (used to host the command log)."""
        f = tk.Frame(grid_parent, bg=self._NAV_CARD)
        f.grid(row=row, column=0, columnspan=ncols, sticky="ew", pady=(8, 0))
        return f


##############################################################################
# SECTION: nav_story.py   (was gcs/app/swarm/nav_story.py)
##############################################################################
class NavStoryMixin:
    """Swarm Navigation panel: the plain-language 'story' text."""

    # ---------- story / health / agreement ----------
    def _nav_story(self, now, nv, order, sts, fresh, lks, lk_ok, snaps):
        PH = {p: i for i, p in enumerate(self._SW_PHASES)}
        live = [s for s in order if fresh[s]]
        n = len(order)
        act = [s for s in live if sts[s].get("phase") not in (None, "IDLE")]
        OK, WARN, BAD, OFF = self._NV_OK, self._NV_WARN, self._NV_BAD, self._NV_OFF
        form_d = None
        lead = sts[act[0]] if act else None
        if lead and lead.get("d"):
            form_d = float(lead["d"])
        formed = [s for s in act if sts[s].get("phase") in ("FORM", "TRANSIT", "SETTLE", "DONE")]

        # distances between every pair (from the agents' own positions)
        pos = {}
        for s in live:
            if sts[s].get("lat") is not None:
                pos[s] = (sts[s]["lat"], sts[s]["lon"])
        pair_d = {}
        ids = [s for s in order if s in pos]
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                pair_d[(a, b)] = math.hypot(*self._ll_to_ne(pos[a], pos[b][0], pos[b][1]))
        nv["pair_d"] = pair_d

        # link quality over every listener / sender pair
        hzs, ages, missing = [], [], 0
        for r in live:
            if not lk_ok[r]:
                continue
            for s in order:
                if s == r:
                    continue
                d = (lks[r].get("rx") or {}).get(str(s))
                if d:
                    hzs.append(float(d.get("hz", 0.0)))
                    ages.append(float(d.get("age", 0.0)))
                else:
                    missing += 1
        nv["hzs"], nv["ages"] = hzs, ages

        # ---- headline ----
        if not live:
            head = "The agents are not running, so there is no swarm activity yet."
            sub = (f"Press AUTO MODE to start a swarm mission (formation spacing: "
                   f"{self.swarm_form_d:g} m)." if self.swarm_form_d else
                   "Set the distance between drones in Formation Control, then press AUTO MODE.")
        elif not act:
            head = f"{len(live)} of {n} agents are online and idle, waiting for a mission."
            sub = "Mark a destination, set the spacing and press AUTO MODE."
        else:
            low = min((sts[s].get("phase") or "IDLE" for s in act), key=lambda p: PH.get(p, 0))
            k = len(act)
            _stp = [s for s in act if sts[s].get("phase") == "STOPPED"]
            _real = [s for s in _stp if not str(sts[s].get("why") or "").startswith("RTL landed")]
            if _real:
                stopped = _real[0]
                head = f"Mission stopped by Drone {self._letter_for_sysid(stopped)}: {sts[stopped].get('why') or 'no reason given'}."
            elif _stp or (low == "DONE" and any(sts[s].get("ls") for s in act)):
                head = (f"Returning home \u2014 the formation is broken and the drones land one by one on their own "
                        f"home points ({len(_stp)} of {k} landed).")
            elif low == "PREP":
                up = sum(1 for s in act if sts[s].get("phase") != "PREP")
                head = f"Drones are arming and taking off \u2014 {up} of {k} have reached altitude."
            elif low == "WAIT":
                head = "Every drone is at altitude. They are waiting for each other before forming up."
            elif low == "FORM":
                head = f"Drones are flying into the triangle \u2014 {int(lead.get('inpos', 0))} of {k} are in position."
            elif low == "TRANSIT":
                spd = sum(math.hypot(sts[s].get("vn") or 0, sts[s].get("ve") or 0) for s in act) / k
                togo = float(lead.get("togo") or 0.0)
                eta = f", about {int(togo / spd)} s left" if spd > 0.3 else ""
                head = f"Flying the route together in a triangle \u2014 {togo:.0f} m to the destination{eta}."
            elif low == "SETTLE":
                head = f"Arriving \u2014 braking and holding the formation ({int(lead.get('inpos', 0))} of {k} in position)."
            else:
                head = "Mission complete. The destination is reached and the triangle is held."
            bits = []
            if pair_d and formed:
                bits.append("gaps " + " / ".join(f"{v:.1f}" for v in pair_d.values()) + (f" m (target {form_d:g} m)" if form_d else " m"))
            if hzs:
                bits.append(f"{sum(hzs):.0f} messages/s between agents")
            sub = "  \u00b7  ".join(bits)
        nv["head"].config(text=head, fg=COL_TEXT if live else COL_TEXT_DIM)
        nv["sub"].config(text=sub)
        nv["bA"].config(text=f"\u25cf {len(live)}/{n} agents online" if live else "\u25cb agents offline",
                        fg=OK if len(live) == n else (WARN if live else OFF))

        # ---- stage bar for the whole swarm ----
        if act:
            low = min((sts[s].get("phase") or "IDLE" for s in act), key=lambda p: PH.get(p, 0))
            self._nav_style_chips(nv["chips"], "STOPPED" if any(
                sts[s].get("phase") == "STOPPED" and not str(sts[s].get("why") or "").startswith("RTL landed")
                for s in act) else low, True)
            phs = {s: sts[s].get("phase") for s in act}
            if len(set(phs.values())) == 1:
                note = f"All {len(act)} drones are in the same stage \u2014 {self._NAV_PHASE_HELP.get(low, '')}."
            else:
                note = "Stages differ: " + "  \u00b7  ".join(f"{self._letter_for_sysid(s)} {p}" for s, p in phs.items()) + \
                       "  \u2014  the swarm waits for the slowest drone."
        else:
            self._nav_style_chips(nv["chips"], "IDLE", False)
            note = ""
        nv["stage_note"].config(text=note)

        # ---- health tiles ----
        def tile(key, state, word, detail):
            v, d = nv["health"][key]
            col = {"ok": OK, "warn": WARN, "bad": BAD}.get(state, OFF)
            v.config(text=f"\u25cf {word}" if state != "off" else "\u25cb " + word, fg=col)
            d.config(text=detail)
        if not live:
            for k_ in ("comms", "form", "agree"):
                tile(k_, "off", "no data", "agents not running")
        else:
            if not hzs:
                tile("comms", "off", "no links yet", "waiting for the first packets")
            else:
                mh, ma = min(hzs), max(ages)
                st_ = "bad" if (missing or ma > 1.5 or mh <= 0) else ("ok" if (mh >= 5 and ma < 0.5) else "warn")
                tile("comms", st_, {"ok": "all links healthy", "warn": "some links slow", "bad": "link lost"}[st_],
                     f"slowest {mh:.1f} Hz \u00b7 oldest packet {ma * 1000:.0f} ms")
            es = [float(sts[s].get("emax") or 0.0) for s in formed]
            if es:
                e = max(es)
                st_ = "ok" if e < 1.0 else ("warn" if e < 3.0 else "bad")
                tile("form", st_, {"ok": "holding shape", "warn": "drifting", "bad": "out of shape"}[st_],
                     f"worst slot error {e:.2f} m")
            else:
                tile("form", "off", "not formed yet", "shown once the triangle forms")

        # ---- agreement checks ----
        def chk(key, state, text):
            ic, val = nv["chk"][key]
            sym = {"ok": "\u2714", "warn": "\u26a0", "bad": "\u2716"}.get(state, "\u2014")
            col = {"ok": OK, "warn": WARN, "bad": BAD}.get(state, OFF)
            ic.config(text=sym, fg=col)
            val.config(text=text, fg=self._NV_VAL if state != "off" else COL_TEXT_DIM)
        spreads = {}
        if len(act) >= 2:
            sp = [math.hypot(sts[s].get("vn") or 0, sts[s].get("ve") or 0) for s in act]
            spreads["speed"] = max(sp) - min(sp)
            al = [float(sts[s].get("alt") or 0.0) for s in act]
            spreads["alt"] = max(al) - min(al)
            hd = [sts[s].get("hdg") for s in act if sts[s].get("hdg") is not None]
            if len(hd) >= 2 and max(sp) > 0.5:
                spreads["heading"] = max(abs((a - b + 180) % 360 - 180) for a in hd for b in hd)
        Rs = [sts[s]["R"] for s in formed if sts[s].get("R")]
        if pair_d and formed and form_d:
            dev = max(abs(v - form_d) for v in pair_d.values())
            tol = max(1.0, 0.1 * form_d)
            chk("spacing", "ok" if dev < tol else ("warn" if dev < 2 * tol else "bad"),
                " / ".join(f"{self._letter_for_sysid(a)}\u2013{self._letter_for_sysid(b)} {v:.1f} m" for (a, b), v in pair_d.items())
                + f"   (target {form_d:g} m, off by {dev:.2f} m)")
        else:
            chk("spacing", "off", "shown once the drones are forming the triangle")
        for key, lim, unit, fmt in (("speed", 0.4, "m/s", "{:.2f}"), ("heading", 15, "\u00b0", "{:.0f}"), ("alt", 1.0, "m", "{:.2f}")):
            if key in spreads:
                v = spreads[key]
                chk(key, "ok" if v < lim else ("warn" if v < 2 * lim else "bad"),
                    ("all within " if v < lim else "differ by ") + fmt.format(v) + f" {unit}")
            else:
                chk(key, "off", "needs at least two active drones" + (" in motion" if key == "heading" else ""))
        if len(Rs) >= 2:
            sp_ = max(math.hypot(a[0] - b[0], a[1] - b[1]) for a in Rs for b in Rs)
            chk("ref", "ok" if sp_ < 1.0 else "warn",
                f"the {len(Rs)} copies differ by at most {sp_:.2f} m" + (" \u2014 consensus" if sp_ < 1.0 else " \u2014 still converging"))
            nv["agree"] = ("ok" if sp_ < 1.0 else "warn")
        else:
            chk("ref", "off", "shown once the triangle forms")
            nv["agree"] = None
        if hzs:
            mh, ma = min(hzs), max(ages)
            chk("fresh", "bad" if (missing or ma > 1.5) else ("ok" if mh >= 5 and ma < 0.5 else "warn"),
                f"slowest link {mh:.1f} Hz, oldest packet {ma * 1000:.0f} ms" + (f", {missing} link(s) silent" if missing else ""))
        else:
            chk("fresh", "off", "no packets yet")
        if act:
            ps = {sts[s].get("phase") for s in act}
            chk("phase", "ok" if len(ps) == 1 else "warn",
                (f"all in {next(iter(ps))}" if len(ps) == 1 else "differ: " + ", ".join(
                    f"{self._letter_for_sysid(s)} {sts[s].get('phase')}" for s in act)))
        else:
            chk("phase", "off", "no mission running")
        checks = [nv["chk"][k][0].cget("text") for k in nv["chk"]]
        bad = sum(1 for c in checks if c in ("\u26a0", "\u2716"))
        good = sum(1 for c in checks if c == "\u2714")
        if good + bad:
            nv["bC"].config(text=f"{good} of {good + bad} checks passing", fg=OK if not bad else WARN)
        else:
            nv["bC"].config(text="")
        if nv["agree"] is None and not (good + bad):
            tile("agree", "off", "no data", "")
        else:
            tile("agree", "ok" if not bad else "warn", "agents agree" if not bad else "differences",
                 f"{good} of {good + bad} checks passing")


##############################################################################
# SECTION: nav_update.py   (was gcs/app/swarm/nav_update.py)
##############################################################################
class NavUpdateMixin:
    """Swarm Navigation panel: snapshots + live value updates."""

    # ---------- data helpers ----------
    def _nav_snapshot(self, sysid):
        f = self.nav_fields.get(sysid, {})

        def num(key):
            try:
                return float(f.get(key))
            except (TypeError, ValueError):
                return None

        if f.get("lat") is None:
            coords = "--"
        else:
            coords = f"{f.get('lat')}\n{f.get('lon', '--')}\nalt {f.get('alt', '--')} m"
        gs, hdg, climb = num("groundspeed"), num("heading"), num("climb")
        if gs is None or hdg is None:
            vel = "--"
        else:
            vn = gs * math.cos(math.radians(hdg))
            ve = gs * math.sin(math.radians(hdg))
            vel = f"N {vn:+.2f}\nE {ve:+.2f} m/s"
        speed = f"{gs:.2f} m/s" if gs is not None else "--"
        heading = f"{hdg:.0f}\u00b0" if hdg is not None else "--"
        task = f.get("mode") or "--"
        return {"coords": coords, "vel": vel, "speed": speed, "heading": heading, "task": task}

    def _nav_refresh(self):
        if self._nav_refresh_job is not None:
            return
        self._nav_refresh_tick()

    def _nav_state_rows(self, st, sid):
        """Rows for the shared-state table from one agent's own broadcast."""
        out = {k: "--" for k, _c in self._NAV_FIELDS}
        lat, lon = st.get("lat"), st.get("lon")
        if lat is not None and lon is not None:
            out["pos"] = f"{lat:.6f}\n{lon:.6f}"
            out["alt"] = f"{float(st.get('alt') or 0.0):.2f} m"
        vn, ve = st.get("vn"), st.get("ve")
        if vn is not None and ve is not None:
            out["vel"] = f"N {vn:+.2f}\nE {ve:+.2f} m/s"
            out["speed"] = f"{math.hypot(vn, ve):.2f} m/s"
        if st.get("hdg") is not None:
            out["hdg"] = f"{float(st['hdg']):.0f}\u00b0"
        out["mode"] = str(st.get("mode") or "--")
        out["armed"] = "ARMED" if st.get("armed") else "disarmed"
        ph = st.get("phase") or "--"
        out["phase"] = ph
        formed = ph in ("FORM", "TRANSIT", "SETTLE", "DONE")
        slot = getattr(self, "_nav_slots", {}).get(sid)
        if formed and slot is not None:
            out["slot"] = f"slot {slot}" + (" (apex)" if slot == 0 else "")
        R = st.get("R")
        if formed and R:
            out["R"] = f"N {R[0]:+.1f}  E {R[1]:+.1f} m"
            out["vr"] = f"{float(st.get('vr') or 0.0):.1f} m/s"
            out["wp"] = f"waypoint {st.get('wi', 0)} \u00b7 {float(st.get('togo') or 0.0):.0f} m to go"
            out["emax"] = f"{float(st.get('emax') or 0.0):.2f} m"
        return out

    def _nav_style_chips(self, chips, phase, online):
        TILE, DIM = self._NAV_TILE, self._NAV_DIM_CHIP
        pipe = self._NAV_PIPE
        idx = pipe.index(phase) if (online and phase in pipe) else -1
        for i, ph in enumerate(pipe):
            c = chips[ph]
            if online and phase == "STOPPED" or idx < 0:
                c.config(text=ph, fg=self._NV_OFF, bg=DIM)
            elif i < idx:
                c.config(text="\u2713 " + ph, fg=self._NV_PAST_FG, bg=self._NV_PAST_BG)
            elif i == idx:
                c.config(text=("\u2714 " if ph == "DONE" else "\u25cf ") + ph, fg=self._NV_ON_ACC, bg=self._NV_ACC)
            else:
                c.config(text=ph, fg=self._NV_OFF, bg=DIM)
        sc = chips["STOPPED"]
        if online and phase == "STOPPED":
            sc.config(text="\u25a0 STOPPED", fg=self._NV_ON_ACC, bg=self._NV_BAD, padx=7)
        else:
            sc.config(text="", bg=TILE, padx=0)

    def _nav_link_color(self, hz, age):
        if age > 1.5 or hz <= 0.0:
            return self._NV_BAD
        return self._NV_OK if (hz >= 5.0 and age < 0.5) else self._NV_WARN

    # ---------- refresh ----------
    def _nav_update(self, now):
        nv = self._nav_v
        order = nv["order"]
        if not nv.get("cells") and order:
            return
        # keep the wrapped texts as wide as the panel
        try:
            w = max(300, self._nav_canvas.winfo_width() - 70)
            if w != nv.get("wrap_w"):
                nv["wrap_w"] = w
                for lb in self._nav_wraps:
                    lb.config(wraplength=w)
        except Exception:
            pass

        sts = {s: self._bus_states.get(s) for s in order}
        fresh = {s: (sts[s] is not None and now - sts[s]["rx"] < 2.0) for s in order}
        lks = {s: self._bus_links.get(s) for s in order}
        lk_ok = {s: (lks[s] is not None and now - lks[s]["rx_t"] < 3.0) for s in order}
        online = sum(1 for s in order if fresh[s])
        snaps = {s: self._nav_snapshot(s) for s in order}
        msgs_per_s = 0.0

        # ---------- A/C: story, health, agreement ----------
        self._nav_story(now, nv, order, sts, fresh, lks, lk_ok, snaps)
        # ---------- B: live formation picture ----------
        try:
            self._nav_draw(now, nv, order, sts, fresh, lks, lk_ok)
        except Exception as e:
            if not self._nav_err_shown:
                self._nav_err_shown = True
                self._append_log(f"!!! SWARM NAV picture error: {e}")

        # ---------- 2. communication matrix ----------
        for r in order:
            for s in order:
                cell = nv["cells"][(r, s)]
                top, bot, fr = cell["top"], cell["bot"], cell["f"]
                if r == s:
                    top.config(text="\u2014", fg=self._NV_OFF)
                    bot.config(text="itself\n\n")
                    continue
                if not lk_ok[r]:
                    top.config(text="\u25cb agent offline", fg=self._NV_OFF)
                    bot.config(text="listener not\nrunning\n")
                    continue
                d = (lks[r].get("rx") or {}).get(str(s))
                if not d:
                    top.config(text="\u25cb nothing yet", fg=self._NV_BAD)
                    bot.config(text="no packet\nreceived\n")
                    continue
                hz, age = float(d.get("hz", 0.0)), float(d.get("age", 0.0))
                msgs_per_s += hz
                col = self._nav_link_color(hz, age)
                delta = ""
                if fresh[s] and d.get("lat") is not None and sts[s].get("lat") is not None:
                    dn, de = self._ll_to_ne((sts[s]["lat"], sts[s]["lon"]), d["lat"], d["lon"])
                    delta = f"\n\u0394 position {math.hypot(dn, de):.1f} m"
                top.config(text=f"\u25cf {hz:.1f} Hz", fg=col)
                bot.config(text=f"{age * 1000:.0f} ms old\n#{d.get('n', 0)} pkts" + (delta or "\n"))
            cell = nv["cells"][(r, 0)]
            g = self._gcs_rx.get(r)
            if fresh[r] and g:
                age = now - sts[r]["rx"]
                hz = float(g["hz"]) if age < 1.5 else 0.0
                msgs_per_s += hz
                cell["top"].config(text=f"\u25cf {hz:.1f} Hz", fg=self._nav_link_color(hz, age))
                cell["bot"].config(text=f"{age * 1000:.0f} ms old\n#{g['n']} pkts\nmeasured by GCS")
            else:
                cell["top"].config(text="\u25cb nothing yet", fg=self._NV_OFF)
                cell["bot"].config(text="no report\nreceived\n")
        nv["b2"].config(text=f"\u2248 {msgs_per_s:.0f} messages/s on the bus" if msgs_per_s else "bus idle",
                        fg=self._NV_OK if msgs_per_s else COL_TEXT_DIM)

        # ---------- 3. shared state ----------
        Rs = []
        for s in order:
            st = sts[s]
            if fresh[s]:
                rows = self._nav_state_rows(st, s)
                col = self._NV_VAL
                if lk_ok[s]:
                    to = self._names(list(lks[s].get("to") or []) + [0])
                    status = f"agent online \u00b7 sends {float(lks[s].get('tx_hz', 0.0)):.1f} Hz \u2192 {to}"
                    stc = self._NV_OK
                else:
                    status, stc = "agent online", self._NV_OK
                if st.get("phase") in ("FORM", "TRANSIT", "SETTLE", "DONE") and st.get("R"):
                    Rs.append(st["R"])
            else:
                rows = {k: "--" for k, _c in self._NAV_FIELDS}
                sn = snaps[s]
                parts = sn["coords"].split("\n")
                if len(parts) == 3:
                    rows["pos"] = "\n".join(parts[:2])
                    rows["alt"] = parts[2][4:]
                rows["vel"], rows["speed"], rows["hdg"], rows["mode"] = sn["vel"], sn["speed"], sn["heading"], sn["task"]
                col = COL_TEXT_DIM
                status, stc = "agent offline \u00b7 plain telemetry", COL_TEXT_DIM
            nv["col_status"][s].config(text=status, fg=stc)
            for key, _cap in self._NAV_FIELDS:
                lb = nv["vals"][(key, s)]
                c = col
                if fresh[s] and key == "emax" and rows[key] != "--":
                    e = float(sts[s].get("emax") or 0.0)
                    c = self._NV_OK if e < 1.0 else (self._NV_WARN if e < 3.0 else self._NV_BAD)
                elif fresh[s] and key == "armed":
                    c = self._NV_WARN if sts[s].get("armed") else COL_TEXT_DIM
                lb.config(text=rows[key], fg=c)
        if len(Rs) >= 2:
            spread = max(math.hypot(a[0] - b[0], a[1] - b[1]) for a in Rs for b in Rs)
            ok = spread < 1.0
            nv["consensus"].config(
                text=(("\u2714" if ok else "\u26a0") + f"  Consensus: the {len(Rs)} agents' formation-reference copies differ by at most "
                      f"{spread:.2f} m \u2014 " + ("they agree on where the formation is." if ok else "they are still converging.")),
                fg=self._NV_OK if ok else self._NV_WARN)
        else:
            nv["consensus"].config(text="Consensus: shown once the drones start forming the triangle.", fg=COL_TEXT_DIM)

        # ---------- 4. commands to the autopilots ----------
        for s in order:
            vals = nv["cmdv"][s]
            vel = lks[s].get("vel") if lk_ok[s] else None
            if vel:
                vn, ve, vd, yaw = vel
                hz = float(lks[s].get("vel_hz", 0.0))
                texts = [f"{vn:+.2f} m/s", f"{ve:+.2f} m/s", f"{vd:+.2f} m/s",
                         "--" if yaw is None else f"{yaw:.0f}\u00b0",
                         f"{hz:.1f} Hz #{lks[s].get('vel_n', 0)}" if hz > 0 else "holding"]
                for lb, t in zip(vals, texts):
                    lb.config(text=t, fg=self._NV_VAL)
            else:
                for lb, t in zip(vals, ["--", "--", "--", "--", "agent not sending"]):
                    lb.config(text=t, fg=COL_TEXT_DIM)
        # ---------- D. one activity timeline ----------
        rows = []
        for _q, ts, txt in [x for x in self._feed if x[0] > self._nav_feed_shown]:
            self._nav_feed_shown = max(self._nav_feed_shown, _q)
            head = txt.split("   ")[0]
            tag = "gcs" if txt.startswith("GCS") else ("ev" if "phase" in txt else
                                                       ("tx" if "\u2192" in head else ("rx" if "\u2190" in head else "ev")))
            segs = [(f"{ts}  ", "dim"), ("BUS  ", "dim"), (txt, tag)]
            m = re.search(r"phase \w+ \u2192 (\w+)\s*$", txt)
            if m:
                segs.append((f"   \u2014 {self._NAV_PHASE_HELP.get(m.group(1), '')}", "dim"))
            rows.append((ts, 0, segs))
        for s in order:
            for q, ts, txt in self._cmd_log.get(s, []):
                if q > nv["cmd_seen"]:
                    rows.append((ts, 1, [(f"{ts}  ", "dim"), ("CMD  ", "dim"),
                                         (f"{self._letter_for_sysid(s)}  ", f"dr{s}"),
                                         (txt, "gcs" if txt.startswith("[GCS]") else "tx")]))
        if rows:
            nv["cmd_seen"] = max([nv["cmd_seen"]] + [q for s in order for q, _t, _x in self._cmd_log.get(s, [])])
            rows.sort(key=lambda r: (r[0], r[1]))
            self._nav_log_append(nv["feed"], [r[2] for r in rows])
        nv["bD"].config(text=f"{self._feed_seq} bus messages so far" if self._feed_seq else "no traffic yet",
                        fg=COL_TEXT_DIM)


##############################################################################
# SECTION: nav_draw.py   (was gcs/app/swarm/nav_draw.py)
##############################################################################
class NavDrawMixin:
    """Swarm Navigation panel: canvas drawing + refresh timer."""

    # ---------- live formation picture ----------
    def _nav_draw(self, now, nv, order, sts, fresh, lks, lk_ok):
        cv = nv["cv"]
        W, H = max(300, cv.winfo_width()), int(cv.cget("height"))
        cv.delete("all")
        TILE = self._NAV_TILE
        live = [s for s in order if fresh[s] and sts[s].get("lat") is not None]
        src_live = len(live) >= 1
        pos = {}
        if src_live:
            for s in order:
                if fresh[s] and sts[s].get("lat") is not None:
                    pos[s] = (sts[s]["lat"], sts[s]["lon"])
        else:
            for s in order:
                if s in self.swarm_positions:
                    pos[s] = tuple(self.swarm_positions[s])
        if not pos:
            cv.create_text(W / 2, H / 2, text="No GPS positions yet", fill=COL_TEXT_DIM, font=("Segoe UI", 10))
            return
        o = next(iter(pos.values()))
        ne = {s: self._ll_to_ne(o, p[0], p[1]) for s, p in pos.items()}
        cn = sum(v[0] for v in ne.values()) / len(ne)
        ce = sum(v[1] for v in ne.values()) / len(ne)
        # direction of flight (mean velocity); north-up when (nearly) standing still
        vn = sum((sts[s].get("vn") or 0.0) for s in pos if fresh[s]) / max(1, sum(1 for s in pos if fresh[s]))
        ve = sum((sts[s].get("ve") or 0.0) for s in pos if fresh[s]) / max(1, sum(1 for s in pos if fresh[s]))
        moving = src_live and math.hypot(vn, ve) > 0.4
        hdg = math.atan2(ve, vn) if moving else (nv.get("last_hdg", 0.0))
        nv["last_hdg"] = hdg
        ch, sh = math.cos(hdg), math.sin(hdg)

        def rot(n_, e_):
            n_, e_ = n_ - cn, e_ - ce
            return (e_ * ch - n_ * sh, n_ * ch + e_ * sh)        # (right, forward)

        pts = {s: rot(*ne[s]) for s in ne}
        ref = None
        if src_live and self._sw:
            lead = next((sts[s] for s in order if fresh[s] and sts[s].get("R")), None)
            if lead and lead.get("phase") in ("FORM", "TRANSIT", "SETTLE", "DONE"):
                R = lead["R"]
                ref = rot(*self._ll_to_ne(o, *self._ne_to_ll(self._sw["o"], R[0], R[1])))
        ext = max([3.0] + [max(abs(p[0]), abs(p[1])) for p in pts.values()] + ([max(abs(ref[0]), abs(ref[1]))] if ref else []))
        sc = min(W * 0.36, H * 0.36) / ext
        cx, cy = W / 2, H / 2 + 6

        def sp(p):
            return cx + p[0] * sc, cy - p[1] * sc

        # faint grid + direction marker
        for k in (-1, 0, 1):
            cv.create_line(cx + k * W * 0.22, 14, cx + k * W * 0.22, H - 8, fill="#222227")
        cv.create_line(14, cy, W - 14, cy, fill="#222227")
        cv.create_text(W - 12, 12, anchor="ne", fill=COL_TEXT_DIM, font=("Segoe UI", 8),
                       text=(f"flight direction \u2191   heading {(math.degrees(hdg) % 360):.0f}\u00b0" if moving else "north \u2191   (not moving)"))
        if moving:
            cv.create_polygon(cx - 6, 30, cx + 6, 30, cx, 14, fill=self._NV_OFF, outline="")
        if not src_live:
            cv.create_text(12, 12, anchor="nw", fill=self._NV_WARN, font=("Segoe UI", 8, "bold"),
                           text="agents offline \u2014 positions from plain telemetry")
        # links
        d_t = None
        for s in order:
            if fresh[s] and sts[s].get("d"):
                d_t = float(sts[s]["d"]); break
        ids = [s for s in order if s in pts]
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                dist = math.hypot(ne[a][0] - ne[b][0], ne[a][1] - ne[b][1])
                col = self._NV_OFF
                if src_live and d_t and all(sts[x].get("phase") in ("FORM", "TRANSIT", "SETTLE", "DONE") for x in (a, b) if fresh.get(x)):
                    dev = abs(dist - d_t)
                    tol = max(1.0, 0.1 * d_t)
                    col = self._NV_OK if dev < tol else (self._NV_WARN if dev < 2 * tol else self._NV_BAD)
                (x1, y1), (x2, y2) = sp(pts[a]), sp(pts[b])
                cv.create_line(x1, y1, x2, y2, fill=col, width=2)
                hz = None
                rates = []
                for r_, s_ in ((a, b), (b, a)):
                    if lk_ok.get(r_):
                        dd = (lks[r_].get("rx") or {}).get(str(s_))
                        if dd:
                            rates.append(float(dd.get("hz", 0.0)))
                if rates:
                    hz = min(rates)
                txt = f"{dist:.1f} m" + (f"  \u00b7  {hz:.0f} Hz" if hz is not None else "")
                mx, my = (x1 + x2) / 2, (y1 + y2) / 2
                cv.create_rectangle(mx - 5 * len(txt) / 1.6 - 4, my - 9, mx + 5 * len(txt) / 1.6 + 4, my + 9,
                                    fill=TILE, outline="")
                cv.create_text(mx, my, text=txt, fill=COL_TEXT, font=("Segoe UI", 8, "bold"))
        # reference point
        if ref:
            x, y = sp(ref)
            cv.create_polygon(x, y - 7, x + 7, y, x, y + 7, x - 7, y, outline=self._NV_ACC, fill="", width=2)
            cv.create_text(x + 11, y, anchor="w", text="ref", fill=self._NV_ACC, font=("Segoe UI", 8))
        # drones
        for s in ids:
            x, y = sp(pts[s])
            col = self._nv_dc(s)
            if src_live and fresh.get(s):
                vv = (sts[s].get("ve") or 0.0, sts[s].get("vn") or 0.0)
                vr, vf = vv[0] * ch - vv[1] * sh, vv[1] * ch + vv[0] * sh
                L_ = min(55.0, math.hypot(vr, vf) * 22)
                if L_ > 4:
                    cv.create_line(x, y, x + vr * 22 * (L_ / max(1e-6, math.hypot(vr, vf) * 22)),
                                   y - vf * 22 * (L_ / max(1e-6, math.hypot(vr, vf) * 22)),
                                   fill=col, width=2, arrow="last")
            cv.create_oval(x - 13, y - 13, x + 13, y + 13, fill=col, outline="#0f141b", width=2)
            cv.create_text(x, y, text=self._letter_for_sysid(s), fill="#0f141b", font=("Segoe UI", 10, "bold"))
            slot = self._nav_slots.get(s)
            tag = ""
            if src_live and fresh.get(s) and sts[s].get("phase") in ("FORM", "TRANSIT", "SETTLE", "DONE") and slot is not None:
                tag = "apex" if slot == 0 else f"slot {slot}"
            if src_live and fresh.get(s):
                tag = (tag + "  " if tag else "") + f"{sts[s].get('mode') or ''}"
            if tag:
                cv.create_text(x, y + 24, text=tag, fill=COL_TEXT_DIM, font=("Segoe UI", 8))
        nv["bB"].config(text=f"scale: grid line \u2248 {W * 0.22 / sc:.0f} m", fg=COL_TEXT_DIM)

    def _nav_refresh_tick(self):
        self._nav_refresh_job = None
        if not self._nav_target and self._nav_prog == 0.0:
            return
        sig = tuple(self.swarm_order)
        if sig != self._nav_sig:
            self._nav_sig = sig
            self._nav_rebuild()
        try:
            self._nav_update(time.time())
        except Exception as e:
            if not self._nav_err_shown:
                self._nav_err_shown = True
                self._append_log(f"!!! SWARM NAV refresh error: {e}")
        self._nav_refresh_job = self.root.after(300, self._nav_refresh_tick)


##############################################################################
# SECTION: map3d.py   (was gcs_3d.py)
##############################################################################
try:
    import psutil
except ImportError:
    psutil = None



##############################################################################
# SECTION: map3d.py   (was gcs/app/map/map3d.py)
##############################################################################
class Map3DMixin:
    """3D Map button: launches / tracks / kills the drone_3d.py process."""

    def _build_3d_map_button(self, toolbar):
        """The '3D' button itself doubles as its own status indicator:
        a status dot (grey when idle, green and gently pulsing while
        drone_3d.py is running) and a small x live inside the button, right
        after the label. Clicking the label/dot area launches the script;
        clicking the x (only "live" once something is running) kills it."""
        btn = tk.Frame(toolbar, bg=COL_BTN, highlightthickness=1,
                        highlightbackground=COL_BORDER, highlightcolor=COL_BORDER,
                        cursor="hand2")
        self.map3d_btn = btn

        self.map3d_label = tk.Label(btn, text="3D", font=("Segoe UI", 9),
                                     fg=COL_TEXT, bg=COL_BTN, cursor="hand2")
        self.map3d_label.pack(side="left", padx=(10, 6), pady=6)

        self.map3d_dot = tk.Canvas(btn, width=10, height=10, bg=COL_BTN,
                                    highlightthickness=0, cursor="hand2")
        self.map3d_dot.pack(side="left", padx=(0, 6), pady=6)
        self.map3d_dot_id = self.map3d_dot.create_oval(1, 1, 9, 9,
                                                         fill=COL_DOT_OFF, outline="")

        self.map3d_close = tk.Canvas(btn, width=16, height=16, bg=COL_BTN,
                                      highlightthickness=0, cursor="hand2")
        self.map3d_close.pack(side="left", padx=(0, 10), pady=6)
        self.map3d_close_bg = self.map3d_close.create_oval(1, 1, 15, 15,
                                                             fill=COL_BTN, outline="")
        self.map3d_close_l1 = self.map3d_close.create_line(5, 5, 11, 11,
                                                             fill=COL_DOT_OFF, width=1.6)
        self.map3d_close_l2 = self.map3d_close.create_line(11, 5, 5, 11,
                                                             fill=COL_DOT_OFF, width=1.6)

        hover_targets = (btn, self.map3d_label, self.map3d_dot, self.map3d_close)

        def _on_enter(_e):
            for w in hover_targets:
                w.config(bg=COL_BTN_HOVER)
            self.map3d_dot.itemconfig(
                self.map3d_dot_id,
                fill=(COL_DOT_ON if self.map3d_process is not None else COL_DOT_OFF))

        def _on_leave(_e):
            for w in hover_targets:
                w.config(bg=COL_BTN)

        def _on_close_enter(_e):
            if self.map3d_process is not None:
                self.map3d_close.itemconfig(self.map3d_close_bg, fill=COL_WARN)

        def _on_close_leave(_e):
            self.map3d_close.itemconfig(self.map3d_close_bg, fill=COL_BTN_HOVER)

        for w in (btn, self.map3d_label, self.map3d_dot):
            w.bind("<Enter>", _on_enter)
            w.bind("<Leave>", _on_leave)
            w.bind("<Button-1>", lambda e: self._launch_3d_map())

        self.map3d_close.bind("<Enter>", _on_close_enter)
        self.map3d_close.bind("<Leave>", _on_close_leave)
        self.map3d_close.bind("<Button-1>", lambda e: self._terminate_3d_map())

        btn.pack(fill="x", pady=3)

    def _launch_3d_map(self):
        if self.map3d_process is not None and self.map3d_process.poll() is None:
            # Already running -- flash the button border instead of
            # spawning a second copy.
            self._flash_3d_map_button()
            return

        drone_dir = PROJECT_ROOT
        script_path = os.path.join(drone_dir, "drone_3d.py")
        if not os.path.isfile(script_path):
            messagebox.showerror(
                "3D",
                f"Could not find drone_3d.py in:\n{drone_dir}"
            )
            return

        # A copy left over from an earlier click/session (before this
        # button tracked/killed its own children) can still be holding
        # a window, port, or GL context open, which silently blocks a
        # fresh one from opening. Clear those out first.
        self._kill_stray_3dmap_processes()

        try:
            self.map3d_process = subprocess.Popen(
                [sys.executable, script_path], cwd=drone_dir,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
        except Exception as e:
            messagebox.showerror("3D", f"Failed to launch drone_3d.py:\n{e}")
            return

        self.map3d_pulse_on = False
        self._pulse_3d_map_dot()
        # Give it a beat to crash-on-startup before the first poll, so a
        # bad launch surfaces as an error dialog instead of just "nothing
        # happened".
        self.map3d_poll_after_id = self.root.after(700, self._poll_3d_map_process)

    def _kill_stray_3dmap_processes(self):
        """Best-effort cleanup of any drone_3d.py process already running --
        including ones from before this dashboard session existed -- so
        the new launch isn't silently blocked by a leftover instance."""
        if psutil is not None:
            def _matches(proc):
                try:
                    cmdline = proc.info.get("cmdline") or []
                except Exception:
                    return False
                return (proc.pid != os.getpid()
                        and any("drone_3d.py" in str(part) for part in cmdline))

            for proc in psutil.process_iter(["pid", "cmdline"]):
                if _matches(proc):
                    try:
                        proc.terminate()
                    except Exception:
                        pass

            deadline = time.time() + 0.8
            while time.time() < deadline:
                if not any(_matches(p) for p in psutil.process_iter(["pid", "cmdline"])):
                    break
                time.sleep(0.1)
        else:
            # No psutil available -- fall back to a blunt OS-level sweep.
            try:
                if os.name == "nt":
                    subprocess.run(
                        ["wmic", "process", "where",
                         "CommandLine like '%drone_3d.py%'", "delete"],
                        capture_output=True
                    )
                else:
                    subprocess.run(["pkill", "-f", "drone_3d.py"], capture_output=True)
                time.sleep(0.3)
            except Exception:
                pass

    def _set_3d_map_idle_visuals(self):
        self.map3d_dot.itemconfig(self.map3d_dot_id, fill=COL_DOT_OFF)
        self.map3d_close.itemconfig(self.map3d_close_l1, fill=COL_DOT_OFF)
        self.map3d_close.itemconfig(self.map3d_close_l2, fill=COL_DOT_OFF)

    def _set_3d_map_running_visuals(self):
        self.map3d_close.itemconfig(self.map3d_close_l1, fill=COL_TEXT)
        self.map3d_close.itemconfig(self.map3d_close_l2, fill=COL_TEXT)

    def _pulse_3d_map_dot(self):
        # Smooth two-tone breathing effect -- alternates between the
        # bright "on" green and a dimmer shade of it, only while running.
        if self.map3d_process is None:
            return
        self._set_3d_map_running_visuals()
        self.map3d_pulse_on = not self.map3d_pulse_on
        color = COL_DOT_ON if self.map3d_pulse_on else "#1f6b45"
        self.map3d_dot.itemconfig(self.map3d_dot_id, fill=color)
        self.map3d_pulse_after_id = self.root.after(650, self._pulse_3d_map_dot)

    def _flash_3d_map_button(self):
        # Briefly brighten the button border to draw the eye back to it
        # when the user clicks "3D Map" while it's already running.
        self.map3d_btn.config(highlightbackground=COL_VALUE, highlightcolor=COL_VALUE)
        self.root.after(350, lambda: self.map3d_btn.config(
            highlightbackground=COL_BORDER, highlightcolor=COL_BORDER))

    def _poll_3d_map_process(self):
        # Detects the case where the user closed the 3D map window
        # themselves (or it crashed on startup) -- the subprocess exits
        # and the button should fall back to its idle look on its own
        # rather than lying about it still running.
        if self.map3d_process is None:
            return
        ret = self.map3d_process.poll()
        if ret is not None:
            proc = self.map3d_process
            self.map3d_process = None
            if self.map3d_pulse_after_id is not None:
                self.root.after_cancel(self.map3d_pulse_after_id)
                self.map3d_pulse_after_id = None
            self._set_3d_map_idle_visuals()
            if ret != 0:
                err_text = ""
                try:
                    err_text = (proc.stderr.read() or "").strip()
                except Exception:
                    pass
                tail = "\n".join(err_text.splitlines()[-15:]) if err_text else \
                    "(no error output captured)"
                messagebox.showerror(
                    "3D",
                    f"drone_3d.py exited unexpectedly (code {ret}):\n\n{tail}"
                )
            return
        self.map3d_poll_after_id = self.root.after(1000, self._poll_3d_map_process)

    def _terminate_3d_map(self):
        proc = self.map3d_process
        self.map3d_process = None
        if self.map3d_pulse_after_id is not None:
            self.root.after_cancel(self.map3d_pulse_after_id)
            self.map3d_pulse_after_id = None
        if self.map3d_poll_after_id is not None:
            self.root.after_cancel(self.map3d_poll_after_id)
            self.map3d_poll_after_id = None
        self._set_3d_map_idle_visuals()
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
            except Exception:
                pass


##############################################################################
# SECTION: visual_odo.py   (NEW -- VISUAL ODO panel: GPS switch, manual home, Gazebo bridge)
##############################################################################
import shutil
import collections


def _quat_to_rpy(qx, qy, qz, qw):
    """Quaternion -> (roll, pitch, yaw) in radians (ZYX convention)."""
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def _mix_hex(a, b, t):
    """Colour between hex colours a and b (t = 0 -> a, t = 1 -> b)."""
    ca = [int(a[i:i + 2], 16) for i in (1, 3, 5)]
    cb = [int(b[i:i + 2], 16) for i in (1, 3, 5)]
    return "#%02x%02x%02x" % tuple(int(round(x + (y - x) * t)) for x, y in zip(ca, cb))


class GzTextParser:
    """Reads the protobuf TEXT that `gz topic -e -t <topic>` prints, one line at a time.

    emit_on = name of the top-level block that is the LAST field of one message
              (IMU -> "linear_acceleration", Pose -> "orientation");
    emit_on = "*" -> every finished top-level block is returned on its own
              (Pose_V -> {"pose": {...}} once per entity).
    feed(line) returns a finished message dict, or None while a message is still coming in.
    """

    def __init__(self, emit_on):
        self.emit_on = emit_on
        self.stack = []        # names of the blocks we are inside
        self.data = []         # one dict per open block
        self.msg = {}          # finished top-level blocks / scalars of the current message

    @staticmethod
    def _value(v):
        v = v.strip()
        if v[:1] == '"':
            return v.strip('"')
        try:
            return float(v)
        except ValueError:
            return v

    def feed(self, line):
        s = line.strip()
        if not s:
            return None
        if s.endswith("{"):
            self.stack.append(s[:-1].strip())
            self.data.append({})
            return None
        if s == "}":
            if not self.stack:
                return None
            name, d = self.stack.pop(), self.data.pop()
            if self.stack:                       # nested block -> belongs to its parent
                self.data[-1][name] = d
                return None
            if self.emit_on == "*":
                return {name: d}
            self.msg[name] = d
            if name == self.emit_on:
                done, self.msg = self.msg, {}
                return done
            return None
        key, _, val = s.partition(":")
        (self.data[-1] if self.stack else self.msg)[key.strip()] = self._value(val)
        return None


def _q_mul(a, b):
    """Quaternion product a*b, both as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _q_rot(q, v):
    """Rotate vector v by quaternion q (body -> world)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx))


def _q_rot_inv(q, v):
    """Rotate vector v by the INVERSE of quaternion q (world -> body)."""
    qx, qy, qz, qw = -q[0], -q[1], -q[2], q[3]
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx))


class GazeboWatcher(threading.Thread):
    """Background 'is a Gazebo simulation running?' probe -- this is what makes the connection automatic.

    Every few seconds it runs `gz topic -l`.  .present turns True as soon as a simulation with a pose /
    model / camera topic is visible, and False again when it disappears.  The dashboard starts and stops
    the bridge + camera by itself from that, so nothing has to be switched on by hand.
    """

    def __init__(self, period=3.0):
        super().__init__(daemon=True)
        self.period = period
        self._halt = threading.Event()
        self.present = False
        self.gz_missing = False
        self.topics = []

    def stop(self):
        self._halt.set()

    def run(self):
        while not self._halt.is_set():
            gz = shutil.which("gz") or shutil.which("ign")
            if not gz:
                self.gz_missing, self.present = True, False
                self._halt.wait(10.0)
                continue
            self.gz_missing = False
            topics = None
            try:
                out = subprocess.run([gz, "topic", "-l"], capture_output=True, text=True, timeout=6)
                if out.returncode == 0:
                    topics = [ln.strip() for ln in out.stdout.splitlines() if ln.strip().startswith("/")]
            except (OSError, subprocess.TimeoutExpired):
                topics = None
            self.topics = topics or []
            self.present = bool(topics) and any(
                "/pose/info" in t or "/model/" in t or "camera" in t.lower() or "imu" in t.lower() for t in topics)
            self._halt.wait(self.period)


class GazeboBridge(threading.Thread):
    """Gets IMU + pose (position / altitude / attitude / speed) out of a running Gazebo simulation.

    Fully automatic: it finds the drone model by itself (from the topics, `gz model --list`, or the
    names in the pose stream), the pose topic and the IMU topic.  Nothing has to be typed in.

    Data path: the gz-transport Python bindings when they are installed (fast, binary), otherwise the
    `gz topic -e` command line (works with nothing extra installed).  Everything is converted to the
    convention telemetry uses (aerospace body frame x forward / y right / z down, heading clockwise
    from north), so the Gazebo and Telemetry sources can be swapped without touching anything else.

    If the world has NO IMU sensor (very common in 'photo' / camera-only worlds where the drone is
    moved by set_pose), the IMU is DERIVED from the pose stream: gyro from the change of orientation,
    accelerometer = (acceleration - gravity) rotated into the body frame.  It is flagged 'derived'.
    """

    RAD2DEG = 180.0 / math.pi
    G = 9.80665
    _NOT_VEHICLE = {"ground_plane", "sun", "default", "world", "ground", "grid", "photo_plane", "terrain"}
    _VEHICLE_HINT = ("iris", "drone", "uav", "quad", "copter", "x500", "x3", "vo", "camera_drone")
    _LINK_HINT = ("link", "rotor", "prop", "sensor", "camera", "base", "ground", "sun", "::")

    def __init__(self, world="", model="", imu_topic="", pose_topic=""):
        super().__init__(daemon=True)
        self.cfg = {"world": world.strip(), "model": model.strip(),
                    "imu_topic": imu_topic.strip(), "pose_topic": pose_topic.strip()}
        self._halt = threading.Event()
        self._restart = threading.Event()
        self._lock = threading.Lock()
        self._procs = []
        self._node = None
        self._subs = []
        self._force_text = False
        self.state = "starting"          # starting / waiting / running / error
        self.detail = "starting ..."
        self.topics = {"imu": None, "pose": None}
        self.model = ""
        self.mode = ""                   # "bindings" / "gz command"
        self.imu = None                  # real IMU: {"t", "accel" (m/s2), "gyro" (deg/s)}
        self.imu_d = None                # IMU derived from the pose stream (same layout, + "derived": True)
        self.pose = None                 # {"t", "x", "y", "z", "roll", "pitch", "heading", "climb", "gspeed", ...}
        self.rate = {"imu": 0.0, "pose": 0.0}
        self._count = {"imu": 0, "pose": 0}
        self._t_rate = time.time()
        self._last_data = 0.0
        self._auto_name = None
        self._hdr_t = None               # simulation time of the Pose_V being read (header stamp)
        self._ref = None                 # reference sample for the differentiation
        self._vel = None
        self._acc = None
        self._gyr = None
        self._dr = None                  # IMU dead-reckoning state {"p", "v", "q"}  (ENU world, FLU body)
        self._dr_t = None
        self.est = None                  # latest dead-reckoned pose (what the GCS shows in IMU mode)
        self._fixes = 0                  # absolute position fixes injected so far (visual / GeoAnchor hook)
        self._hist = collections.deque(maxlen=3000)   # (wall time, x, y) of the estimate, for latency compensation
        self.baro_aid = True             # barometer (stand-in: Gazebo z + noise) keeps the altitude from drifting
        self._baro_t = None
        self.vis = {"acc": 0, "rej": 0, "rej_run": 0, "last_corr": None, "last_t": None,
                    "last_gain": None, "bias": (0.0, 0.0)}

    # ---------- public ----------
    def configure(self, world, model, imu_topic, pose_topic):
        self.cfg = {"world": world.strip(), "model": model.strip(),
                    "imu_topic": imu_topic.strip(), "pose_topic": pose_topic.strip()}
        self._restart.set()

    def stop(self):
        self._halt.set()
        self._restart.set()
        self._kill_readers()

    def snapshot(self):
        with self._lock:
            imu = self.imu if (self.topics.get("imu") and self.imu) else self.imu_d
            return {"state": self.state, "detail": self.detail, "topics": dict(self.topics),
                    "model": self.model, "mode": self.mode,
                    "imu": dict(imu) if imu else None,
                    "pose": dict(self.pose) if self.pose else None,
                    "est": self._est_with_error(),
                    "imu_real": bool(self.topics.get("imu")),
                    "fixes": self._fixes,
                    "vis": dict(self.vis),
                    "rate": dict(self.rate)}

    def _est_with_error(self):
        """Dead-reckoned pose + its distance to the Gazebo ground truth (truth is ONLY used for this readout)."""
        if not self.est:
            return None
        e = dict(self.est)
        if self.pose:
            e["err_h"] = math.hypot(e["x"] - self.pose["x"], e["y"] - self.pose["y"])
            e["err_v"] = abs(e["z"] - self.pose["z"])
        else:
            e["err_h"] = e["err_v"] = None
        return e

    def reanchor(self):
        """Throw the dead-reckoned state away; it restarts from the next ground-truth pose."""
        with self._lock:
            self._dr = None
            self._dr_t = None
            self.est = None
            self._hist.clear()
            self._baro_t = None
            self.vis.update(last_t=None, rej_run=0)

    def _est_at(self, t):
        """Estimated (x, y) at wall time t (linear interpolation in the history), or None."""
        h = self._hist
        if len(h) < 2 or t < h[0][0] - 0.5:
            return None
        lst = list(h)
        if t >= lst[-1][0]:
            return lst[-1][1], lst[-1][2]
        lo, hi = 0, len(lst) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if lst[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, x0, y0 = lst[lo]
        t1, x1, y1 = lst[hi]
        a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        return x0 + a * (x1 - x0), y0 + a * (y1 - y0)

    def inject_fix(self, x, y, t_cap=None, gain=0.8, kv=0.3, kb=0.03, force=False):
        """ABSOLUTE position fix from the visual localiser (GeoAnchor) -- the PRIMARY position source.
        (x east, y north) in local Gazebo metres, measured from the camera frame taken at wall time t_cap.

        The IMU estimate only has to carry the position BETWEEN fixes, so a fix is trusted strongly:
          * innovation  e = fix - estimate AT THE TIME THE FRAME WAS TAKEN  (the localiser needs time to
            compute, so the frame is old when the fix arrives -- comparing to the CURRENT estimate would
            wrongly cancel the motion made since)
          * position  += gain * e                      (gain 0.8: mostly jump to the visual position)
          * velocity  += kv * e / T                    (learns the velocity error that caused the drift)
          * accel bias -= kb * 2 e / T^2               (learns the accelerometer bias, so the next gap drifts less)
        A fix that disagrees with the estimate by more than the gate is rejected as an outlier
        (a wrong map match must not teleport the drone); after 3 rejects in a row the visual position is
        believed anyway (the IMU estimate is the one that was lost).
        Returns {"ok", "err", ...}; err = size of the correction in metres."""
        now = time.time()
        with self._lock:
            if self._dr is None:
                return {"ok": False, "why": "IMU estimate not started"}
            s, vis = self._dr, self.vis
            ref = self._est_at(t_cap if t_cap else now)
            if ref is None:
                ref = (s["p"][0], s["p"][1])
            ex, ey = x - ref[0], y - ref[1]
            err = math.hypot(ex, ey)
            t_prev = vis["last_t"]
            T = min(20.0, max(0.7, now - t_prev)) if t_prev else None
            gate = 40.0 + 3.0 * (T or 0.0)
            first = t_prev is None
            if not (force or first) and err > gate:
                vis["rej"] += 1
                vis["rej_run"] += 1
                if vis["rej_run"] < 3:
                    return {"ok": False, "why": f"outlier ({err:.0f} m > gate {gate:.0f} m)", "err": err}
            vis["rej_run"] = 0
            s["p"][0] += gain * ex
            s["p"][1] += gain * ey
            for i in range(len(self._hist)):                      # keep the history consistent with the correction
                tt, hx, hy = self._hist[i]
                self._hist[i] = (tt, hx + gain * ex, hy + gain * ey)
            if T is not None and err <= gate:
                s["v"][0] += kv * ex / T
                s["v"][1] += kv * ey / T
                b = s["b"]
                b[0] = max(-0.08, min(0.08, b[0] - kb * 2.0 * ex / (T * T)))
                b[1] = max(-0.08, min(0.08, b[1] - kb * 2.0 * ey / (T * T)))
                vis["bias"] = (b[0], b[1])
            vis.update(acc=vis["acc"] + 1, last_corr=err, last_t=now, last_gain=gain)
            self._fixes += 1
            return {"ok": True, "err": err, "T": T, "first": first}

    def _baro_update(self, now, z_true):
        """Barometer aid for the VERTICAL channel (caller holds the lock).  An IMU alone drifts away in
        altitude within a minute (0.01 m/s2 of bias = 50 m after 100 s), so -- exactly like a Pixhawk -- the
        altitude is held by a barometer.  Stand-in: Gazebo z + 0.25 m of noise.  Third-order complementary
        filter (position, velocity, accelerometer bias), all poles at w = 0.8 rad/s."""
        s = self._dr
        last, self._baro_t = self._baro_t, now
        if s is None or last is None:
            return
        dt = now - last
        if dt <= 0.0 or dt > 0.5:
            return
        w = 0.8
        e = (z_true + random.gauss(0.0, 0.25)) - s["p"][2]
        s["p"][2] += 3.0 * w * dt * e
        s["v"][2] += 3.0 * w * w * dt * e
        s["b"][2] -= w ** 3 * dt * e

    # ---------- internals ----------
    def _set(self, state, detail):
        with self._lock:
            self.state, self.detail = state, detail

    @staticmethod
    def _gz():
        return shutil.which("gz") or shutil.which("ign")

    def _list_topics(self, gz):
        try:
            out = subprocess.run([gz, "topic", "-l"], capture_output=True, text=True, timeout=6)
        except (OSError, subprocess.TimeoutExpired):
            return None
        if out.returncode != 0:
            return None
        return [ln.strip() for ln in out.stdout.splitlines() if ln.strip().startswith("/")]

    def _model_names(self, gz, topics):
        """Candidate drone models: from topic names (/model/<m>/...) and from `gz model --list`."""
        names = []
        for t in topics:
            m = re.search(r"/model/([^/]+)/", t)
            if m and m.group(1) not in names:
                names.append(m.group(1))
        try:
            out = subprocess.run([gz, "model", "--list"], capture_output=True, text=True, timeout=6)
            if out.returncode == 0:
                for ln in out.stdout.splitlines():
                    m = re.match(r"^\s*-\s+(\S+)\s*$", ln)
                    if m and m.group(1) not in names:
                        names.append(m.group(1))
        except (OSError, subprocess.TimeoutExpired):
            pass
        names = [n for n in names if n.lower() not in self._NOT_VEHICLE]
        names.sort(key=lambda n: 0 if any(h in n.lower() for h in self._VEHICLE_HINT) else 1)
        return names

    def _choose(self, gz, topics):
        """Pick (imu_topic, pose_topic, model) from what Gazebo is publishing."""
        cfg = self.cfg
        model, world, imu = cfg["model"], cfg["world"], cfg["imu_topic"]
        if not model:
            names = self._model_names(gz, topics)
            # a model that also owns an imu / camera topic is certainly the vehicle
            for n in names:
                if any(f"/model/{n}/" in t and ("imu" in t.lower() or "camera" in t.lower()) for t in topics):
                    model = n
                    break
            if not model and names:
                model = names[0]
        if not imu:
            cands = [t for t in topics if t.lower().endswith("/imu")] or [t for t in topics if "imu" in t.lower()]
            if model:
                cands = [t for t in cands if f"/model/{model}/" in t] or cands
            imu = cands[0] if cands else None
        if imu and not model:
            m = re.search(r"/model/([^/]+)/", imu)
            model = m.group(1) if m else ""
        if not world:
            for t in ([imu] if imu else []) + topics:
                m = re.search(r"/world/([^/]+)/", t)
                if m:
                    world = m.group(1)
                    break
        pose = cfg["pose_topic"]
        if not pose:
            if world and f"/world/{world}/pose/info" in topics:
                pose = f"/world/{world}/pose/info"
            elif model and f"/model/{model}/pose" in topics:
                pose = f"/model/{model}/pose"
            else:
                ws = sorted({m.group(1) for t in topics for m in [re.match(r"/world/([^/]+)/pose/info$", t)] if m})
                if ws:
                    pose = f"/world/{ws[0]}/pose/info"
        return imu, pose, model

    # ---------- readers ----------
    @classmethod
    def _bindings(cls):
        import importlib
        for tv, mv in ((13, 10), (14, 11), (12, 9)):
            try:
                node = importlib.import_module(f"gz.transport{tv}").Node
                kinds = {"pose_v": importlib.import_module(f"gz.msgs{mv}.pose_v_pb2").Pose_V,
                         "pose": importlib.import_module(f"gz.msgs{mv}.pose_pb2").Pose,
                         "imu": importlib.import_module(f"gz.msgs{mv}.imu_pb2").IMU}
                return node, kinds
            except Exception:
                continue
        return None

    @staticmethod
    def _pb_pose(p):
        return {"name": p.name,
                "position": {"x": p.position.x, "y": p.position.y, "z": p.position.z},
                "orientation": {"x": p.orientation.x, "y": p.orientation.y,
                                "z": p.orientation.z, "w": p.orientation.w}}

    def _subscribe_bindings(self, b, topic, kind, model):
        node, kinds = b
        if self._node is None:
            self._node = node()
        by_name = topic.endswith("pose/info")
        if kind == "imu":
            def cb(m):
                la, av = m.linear_acceleration, m.angular_velocity
                ts = None
                try:
                    ts = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
                except Exception:
                    pass
                o = m.orientation
                self._on_imu({"linear_acceleration": {"x": la.x, "y": la.y, "z": la.z},
                              "angular_velocity": {"x": av.x, "y": av.y, "z": av.z},
                              "orientation": {"x": o.x, "y": o.y, "z": o.z, "w": o.w}}, ts)
            ok = self._node.subscribe(kinds["imu"], topic, cb)
        elif by_name:
            def cb(m):
                ts = None
                try:
                    ts = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
                except Exception:
                    pass
                for p in m.pose:
                    if not model or p.name == model:
                        self._on_pose(self._pb_pose(p), model, ts, True)
            ok = self._node.subscribe(kinds["pose_v"], topic, cb)
        else:
            def cb(m):
                self._on_pose(self._pb_pose(m), model, None, False)
            ok = self._node.subscribe(kinds["pose"], topic, cb)
        if ok is False:
            raise RuntimeError(f"could not subscribe to {topic}")
        self._subs.append(topic)

    def _spawn(self, gz, topic, kind, model):
        proc = subprocess.Popen([gz, "topic", "-e", "-t", topic], stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self._procs.append(proc)
        threading.Thread(target=self._reader, args=(proc, kind, topic, model), daemon=True).start()

    def _kill_readers(self):
        for p in self._procs:
            try:
                p.kill()
            except Exception:
                pass
        self._procs = []
        if self._node is not None:
            for t in self._subs:
                try:
                    self._node.unsubscribe(t)
                except Exception:
                    pass
        self._subs = []
        self._node = None

    def _reader(self, proc, kind, topic, model):
        by_name = topic.endswith("pose/info")
        if kind == "imu":
            parser = GzTextParser("linear_acceleration")
        elif by_name:
            parser = GzTextParser("*")            # Pose_V: header, then one {"pose": ...} per entity
        else:
            parser = GzTextParser("orientation")  # single Pose message
        try:
            for line in proc.stdout:
                msg = parser.feed(line)
                if msg is None:
                    continue
                if kind == "imu":
                    self._on_imu(msg)
                elif by_name:
                    if "header" in msg:                       # simulation time of this Pose_V
                        st = msg["header"].get("stamp") if isinstance(msg["header"], dict) else None
                        if isinstance(st, dict):
                            self._hdr_t = float(st.get("sec", 0.0)) + float(st.get("nsec", 0.0)) * 1e-9
                    elif isinstance(msg.get("pose"), dict):   # NOTE: anything that is not a 'pose' block is ignored
                        self._on_pose(msg["pose"], model, self._hdr_t, True)
                else:
                    self._on_pose(msg, model, None, False)
        except Exception:
            pass

    # ---------- message handlers ----------
    def _tick_rate(self, kind):
        self._count[kind] += 1
        now = time.time()
        if now - self._t_rate >= 1.0:
            dt = now - self._t_rate
            self.rate = {k: self._count[k] / dt for k in self._count}
            self._count = {k: 0 for k in self._count}
            self._t_rate = now

    def _on_imu(self, m, ts=None):
        la, av = m.get("linear_acceleration", {}), m.get("angular_velocity", {})
        if ts is None:
            st = m.get("header", {}).get("stamp") if isinstance(m.get("header"), dict) else None
            if isinstance(st, dict):
                ts = float(st.get("sec", 0.0)) + float(st.get("nsec", 0.0)) * 1e-9
        k = self.RAD2DEG
        f_flu = (la.get("x", 0.0), la.get("y", 0.0), la.get("z", 0.0))     # specific force, Gazebo body frame
        w_flu = (av.get("x", 0.0), av.get("y", 0.0), av.get("z", 0.0))     # angular rate, rad/s
        # Gazebo body frame is FLU (x forward, y left, z up) -> aerospace FRD (x forward, y right, z down)
        accel = (f_flu[0], 0.0 - f_flu[1], 0.0 - f_flu[2])
        gyro = (w_flu[0] * k, 0.0 - w_flu[1] * k, 0.0 - w_flu[2] * k)
        with self._lock:
            now = time.time()
            self.imu = {"t": now, "accel": accel, "gyro": gyro, "derived": False}
            self._last_data = now
            self._tick_rate("imu")
            self._dr_step(ts if ts else now, f_flu, w_flu, self._ref_attitude(m.get("orientation")))

    @staticmethod
    def _ref_attitude(o):
        """Attitude (x, y, z, w) reported by the IMU source (a Cube's EKF attitude, or a Gazebo IMU sensor),
        or None when the message has none (all zeros / not a unit quaternion)."""
        if not isinstance(o, dict):
            return None
        q = (o.get("x", 0.0), o.get("y", 0.0), o.get("z", 0.0), o.get("w", 0.0))
        n = math.sqrt(sum(c * c for c in q))
        return tuple(c / n for c in q) if 0.98 < n < 1.02 else None

    def _is_vehicle_name(self, name):
        n = (name or "").lower()
        return bool(n) and n not in self._NOT_VEHICLE and not any(h in n for h in self._LINK_HINT)

    def _on_pose(self, p, model, ts, by_name):
        if not isinstance(p, dict):
            return
        name = p.get("name")
        if by_name:
            if model:
                if name != model:
                    return
            else:                                    # model unknown: lock on to the first vehicle-looking entity
                if self._auto_name is None and self._is_vehicle_name(name):
                    self._auto_name = name
                if name != self._auto_name:
                    return
        pos, ori = p.get("position"), p.get("orientation")
        if not isinstance(pos, dict):
            return
        if ori is None:
            q = (0.0, 0.0, 0.0, 1.0)
        else:
            q = (ori.get("x", 0.0), ori.get("y", 0.0), ori.get("z", 0.0), ori.get("w", 1.0))
        roll, pitch, yaw = _quat_to_rpy(*q)
        now = time.time()
        pos3 = (pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0))
        t_sim = ts if ts else now
        with self._lock:
            self._differentiate(t_sim, pos3, q, now)
            if self.baro_aid:
                self._baro_update(now, pos3[2])
            vx, vy, vz = self._vel if self._vel else (0.0, 0.0, 0.0)
            self.pose = {"t": now, "x": pos3[0], "y": pos3[1], "z": pos3[2],
                         "roll": roll * self.RAD2DEG,
                         "pitch": 0.0 - pitch * self.RAD2DEG,                       # FLU pitch -> aerospace pitch
                         "heading": (90.0 - yaw * self.RAD2DEG) % 360.0,         # ENU yaw -> compass heading
                         "climb": vz, "vx": vx, "vy": vy, "gspeed": math.hypot(vx, vy), "q": q}
            self._last_data = now
            self._tick_rate("pose")

    def _differentiate(self, t, p, q, now):
        """Velocity / acceleration / angular rate from successive poses (caller holds the lock)."""
        ref = self._ref
        if ref is None or t < ref["t"] - 1e-6:          # first sample, or the simulation was reset
            self._ref = {"t": t, "p": p, "q": q, "v": None}
            self._vel = self._acc = self._gyr = None
            return
        dt = t - ref["t"]
        if dt < 0.04:                                   # ~25 Hz is plenty and keeps the noise low
            return
        v = tuple((a - b) / dt for a, b in zip(p, ref["p"]))
        self._vel = v if self._vel is None else tuple(0.5 * a + 0.5 * b for a, b in zip(v, self._vel))
        if ref["v"] is not None:
            a_raw = tuple((a - b) / dt for a, b in zip(v, ref["v"]))
            self._acc = a_raw if self._acc is None else tuple(0.65 * o + 0.35 * n for o, n in zip(self._acc, a_raw))
        qr = _q_mul((-ref["q"][0], -ref["q"][1], -ref["q"][2], ref["q"][3]), q)   # body-frame rotation since ref
        if qr[3] < 0.0:
            qr = tuple(-c for c in qr)
        s = math.sqrt(qr[0] ** 2 + qr[1] ** 2 + qr[2] ** 2)
        ang = 2.0 * math.atan2(s, qr[3])
        w = (0.0, 0.0, 0.0) if s < 1e-9 else tuple(c / s * ang / dt for c in qr[:3])
        self._gyr = w if self._gyr is None else tuple(0.5 * o + 0.5 * n for o, n in zip(self._gyr, w))
        self._ref = {"t": t, "p": p, "q": q, "v": v}
        if self._acc is not None and self._gyr is not None:
            f_world = (self._acc[0], self._acc[1], self._acc[2] + self.G)       # specific force (ENU)
            fb = _q_rot_inv(q, f_world)                                         # FLU body frame
            k = self.RAD2DEG
            self.imu_d = {"t": now, "derived": True,
                          "accel": (fb[0], 0.0 - fb[1], 0.0 - fb[2]),
                          "gyro": (self._gyr[0] * k, 0.0 - self._gyr[1] * k, 0.0 - self._gyr[2] * k)}
            if not self.topics.get("imu"):
                self._tick_rate("imu")
                self._dr_step(t, fb, self._gyr, tuple(q))

    # ---------- IMU dead reckoning ----------
    def _dr_init(self):
        """Anchor the estimator on the current ground-truth pose (start position, orientation, velocity).
        This is the ONLY place the truth pose enters the estimate."""
        p = self.pose
        if not p:
            return False
        v = self._vel or (0.0, 0.0, 0.0)
        self._dr = {"p": [p["x"], p["y"], p["z"]], "v": list(v), "q": list(p["q"]), "b": [0.0, 0.0, 0.0]}
        self._dr_t = None
        return True

    def _dr_step(self, t, f, w, q_ref=None):
        """One strapdown step (caller holds the lock).
        f = specific force (accelerometer, m/s2) and w = angular rate (rad/s), both in the Gazebo body frame.
        attitude : gyro integration, with a gentle tilt correction from the accelerometer (Mahony filter)
        velocity : rotate f into the world frame, add gravity back, integrate
        position : integrate the velocity
        q_ref    : attitude reported by the IMU source (the Cube's EKF attitude).  When present the attitude follows
                   it (the gyro only bridges the gaps) - the accelerometer cannot tell tilt from acceleration, so
                   without it the attitude drifts away with the gyro bias.
        """
        if self._dr is None and not self._dr_init():
            return
        s = self._dr
        last, self._dr_t = self._dr_t, t
        if last is None:
            return
        dt = t - last
        if dt > 2.0:                                    # IMU stream was away for a while (script restarted):
            self._dr = None                             # the old estimate is meaningless -> re-anchor on the
            self._dr_t = None                           # next ground-truth pose, like a Cube power-cycle
            return
        if dt <= 0.0 or dt > 0.5:
            return
        q = tuple(s["q"])
        fn = math.sqrt(f[0] ** 2 + f[1] ** 2 + f[2] ** 2)
        fw0 = _q_rot(q, f)
        # linear acceleration of the estimate, smoothed over ~30 ms so sensor noise does not open the gate
        al = dt / (0.03 + dt)
        lp = s.setdefault("lin_lp", [0.0, 0.0, 0.0])
        for i, c in enumerate((fw0[0], fw0[1], fw0[2] - self.G)):
            lp[i] += al * (c - lp[i])
        lin = math.sqrt(lp[0] ** 2 + lp[1] ** 2 + lp[2] ** 2)
        # Tilt correction from the accelerometer is only valid while the drone is NOT accelerating (then the
        # accelerometer sees gravity only).  During a manoeuvre it would mistake acceleration for tilt, bend the
        # attitude and then integrate that error into velocity.  So the weight falls off smoothly: full strength at
        # constant velocity / hover, almost zero as soon as the drone accelerates (0.05 m/s2 -> 37 %, 0.15 -> 0 %).
        wt = math.exp(-(lin / 0.05) ** 2)
        if q_ref is None and fn > 1e-6 and wt > 1e-3 and abs(fn - self.G) < 0.03 * self.G:
            up = _q_rot_inv(q, (0.0, 0.0, 1.0))                     # where the filter thinks 'up' is, in the body
            fh = (f[0] / fn, f[1] / fn, f[2] / fn)
            kp = 0.3 * wt
            w = (w[0] + kp * (fh[1] * up[2] - fh[2] * up[1]),
                 w[1] + kp * (fh[2] * up[0] - fh[0] * up[2]),
                 w[2] + kp * (fh[0] * up[1] - fh[1] * up[0]))
        ang = math.sqrt(w[0] ** 2 + w[1] ** 2 + w[2] ** 2) * dt
        if ang > 1e-12:
            sn = math.sin(ang / 2.0) / (ang / dt)
            dq = (w[0] * sn, w[1] * sn, w[2] * sn, math.cos(ang / 2.0))
        else:
            dq = (0.5 * w[0] * dt, 0.5 * w[1] * dt, 0.5 * w[2] * dt, 1.0)
        q = _q_mul(q, dq)
        if q_ref is not None:                                       # follow the reported attitude (50 ms smoothing)
            if sum(a * b for a, b in zip(q, q_ref)) < 0.0:
                q_ref = tuple(-c for c in q_ref)
            ab = dt / (0.05 + dt)
            q = tuple(a + ab * (b - a) for a, b in zip(q, q_ref))
        n = math.sqrt(sum(c * c for c in q)) or 1.0
        q = tuple(c / n for c in q)
        fw = _q_rot(q, f)                                           # specific force in the world (ENU) frame
        b = s["b"]                                                 # learned accelerometer bias (visual + baro aiding)
        a = (fw[0] - b[0], fw[1] - b[1], fw[2] - self.G - b[2])      # + gravity  ->  true acceleration
        v, p = s["v"], s["p"]
        for i in range(3):
            p[i] += v[i] * dt + 0.5 * a[i] * dt * dt
            v[i] += a[i] * dt
        s["q"] = list(q)
        roll, pitch, yaw = _quat_to_rpy(*q)
        self._hist.append((time.time(), p[0], p[1]))
        self.est = {"t": time.time(), "x": p[0], "y": p[1], "z": p[2],
                    "roll": roll * self.RAD2DEG, "pitch": 0.0 - pitch * self.RAD2DEG,
                    "heading": (90.0 - yaw * self.RAD2DEG) % 360.0,
                    "climb": v[2], "vx": v[0], "vy": v[1], "gspeed": math.hypot(v[0], v[1]), "q": q}

    # ---------- main loop ----------
    def run(self):
        while not self._halt.is_set():
            self._restart.clear()
            gz = self._gz()
            if not gz:
                self._set("error", "'gz' command not found - is Gazebo installed and on the PATH?")
                self._halt.wait(5.0)
                continue
            topics = self._list_topics(gz)
            if topics is None:
                self._set("waiting", "no answer from Gazebo (is the simulator running?)")
                self._halt.wait(3.0)
                continue
            if not topics:
                self._set("waiting", "no Gazebo topics visible - is the simulator running (same user / GZ_PARTITION)?")
                self._halt.wait(3.0)
                continue
            imu, pose, model = self._choose(gz, topics)
            if not pose and not imu:
                self._set("waiting", "Gazebo is up but no pose / IMU topic was found yet")
                self._halt.wait(3.0)
                continue
            with self._lock:
                self.topics = {"imu": imu, "pose": pose}
                self.model = model
                self._auto_name = None
                self._ref = self._vel = self._acc = self._gyr = None
                self.imu = self.imu_d = self.pose = None
                self._dr = self.est = self._dr_t = None
                self._last_data = 0.0
            b = None if self._force_text else self._bindings()
            try:
                if b:
                    try:
                        if imu:
                            self._subscribe_bindings(b, imu, "imu", model)
                        if pose:
                            self._subscribe_bindings(b, pose, "pose", model)
                        self.mode = "bindings"
                    except Exception:
                        self._kill_readers()
                        b = None
                if not b:
                    if imu:
                        self._spawn(gz, imu, "imu", model)
                    if pose:
                        self._spawn(gz, pose, "pose", model)
                    self.mode = "gz command"
            except OSError as e:
                self._set("error", f"could not start gz: {e}")
                self._halt.wait(5.0)
                continue
            bits = [f"model '{model}'" if model else "model: first vehicle in the pose stream",
                    "IMU ok" if imu else "no IMU sensor -> IMU derived from pose",
                    "pose ok" if pose else "no pose topic"]
            self._set("running", " \u00b7 ".join(bits))
            started = probed = time.time()
            while not self._halt.is_set() and not self._restart.is_set():
                self._halt.wait(1.0)
                if self._procs and any(p.poll() is not None for p in self._procs):
                    break                                   # a gz reader died (simulator closed) -> detect again
                if not imu and not self.cfg["imu_topic"] and time.time() - probed >= 2.0:
                    # connected without an IMU topic (IMU derived from the pose): the script that publishes the
                    # IMU (fly_smooth.py) may start later -> look again every ~2 s and reconnect when it shows up
                    probed = time.time()
                    again = self._list_topics(gz)
                    if again and any(t.lower().endswith("/imu") for t in again):
                        break
                if time.time() - max(self._last_data, started) > 6.0:
                    if self.mode == "bindings":
                        self._force_text = True             # bindings gave nothing -> use the command line
                    break
            self._kill_readers()
        self._kill_readers()


class GazeboCamera(threading.Thread):
    """Live camera frames out of a running Gazebo simulation (the drone's camera).

    1. It lists the Gazebo topics with `gz topic -l` and picks the one that carries gz.msgs.Image
       (a name that ends in /image or contains 'camera'; the model typed in the bridge card is preferred;
       a topic typed in the 'Camera topic' box is used as it is).
    2. It subscribes to it through gz-transport.  Best (smooth video): the Python bindings
           sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-protobuf
       Without them it falls back to `gz topic -e -t <topic> -n 1` (one frame, a few times per second:
       works with nothing extra installed, but is only a slow preview).
    The newest frame is kept in self.frame = {"t", "w", "h", "fmt", "data"} for the GUI to draw.
    """

    _SKIP = ("camera_info", "depth", "points", "segmentation", "label", "thermal", "bounding")
    _PIX = {1: "L_INT8", 3: "RGB_INT8", 4: "RGBA_INT8", 5: "BGRA_INT8", 8: "BGR_INT8"}

    def __init__(self, topic="", model=""):
        super().__init__(daemon=True)
        self.cfg = {"topic": topic.strip(), "model": model.strip()}
        self._halt = threading.Event()
        self._restart = threading.Event()
        self._lock = threading.Lock()
        self.state = "starting"          # starting / waiting / running / error
        self.detail = "starting ..."
        self.topic = None
        self.mode = ""                   # "stream" (python bindings) / "preview" (gz command line)
        self.frame = None
        self.fps = 0.0
        self._n = 0
        self._t_fps = time.time()
        self._last_store = 0.0

    # ---------- public ----------
    def configure(self, topic, model):
        self.cfg = {"topic": topic.strip(), "model": model.strip()}
        self._restart.set()

    def stop(self):
        self._halt.set()
        self._restart.set()

    def snapshot(self):
        with self._lock:
            return {"state": self.state, "detail": self.detail, "topic": self.topic, "mode": self.mode,
                    "frame": self.frame, "fps": self.fps}

    # ---------- internals ----------
    def _set(self, state, detail):
        with self._lock:
            self.state, self.detail = state, detail

    def _store(self, w, h, fmt, data):
        now = time.time()
        with self._lock:
            self.frame = {"t": now, "w": int(w), "h": int(h), "fmt": fmt, "data": data}
            self._n += 1
            if now - self._t_fps >= 1.0:
                self.fps, self._n, self._t_fps = self._n / (now - self._t_fps), 0, now

    @staticmethod
    def _gz():
        return shutil.which("gz") or shutil.which("ign")

    @staticmethod
    def _run(args, timeout):
        try:
            out = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return out.stdout if out.returncode == 0 else None

    def _is_image_topic(self, gz, topic):
        out = self._run([gz, "topic", "-i", "-t", topic], 4)
        return bool(out) and "msgs.Image" in out

    def _choose(self, gz, topics):
        if self.cfg["topic"]:
            return self.cfg["topic"]
        model = self.cfg["model"]
        cands = []
        for t in topics:
            low = t.lower()
            if any(k in low for k in self._SKIP):
                continue
            if low.endswith("/image") or low.endswith("/image_raw") or "camera" in low or "image" in low or "/cam" in low:
                cands.append(t)
        cands.sort(key=lambda t: (0 if model and f"/model/{model}/" in t else 1,
                                  0 if t.lower().endswith("/image") else 1, len(t)))
        for t in cands[:6]:
            if self._is_image_topic(gz, t):
                return t
        return None

    @classmethod
    def _bindings(cls):
        """(Node, Image) of the gz-transport python bindings, or None if they are not installed."""
        import importlib
        for tv, mv in ((13, 10), (14, 11), (12, 9)):
            try:
                node = importlib.import_module(f"gz.transport{tv}").Node
                image = importlib.import_module(f"gz.msgs{mv}.image_pb2").Image
                return node, image
            except Exception:
                continue
        return None

    # --- frames from the bindings (callback runs on the gz-transport thread) ---
    def _on_msg(self, msg):
        now = time.time()
        if now - self._last_store < 0.05:               # at most ~20 frames/s are kept
            return
        self._last_store = now
        try:
            fmt = msg.DESCRIPTOR.fields_by_name["pixel_format_type"].enum_type.values_by_number[
                msg.pixel_format_type].name
        except Exception:
            fmt = "RGB_INT8"
        self._store(msg.width, msg.height, fmt, bytes(msg.data))

    # --- one frame from the command-line text output ---
    @classmethod
    def _parse_text(cls, text):
        depth, w, h, fmt, data = 0, 0, 0, "RGB_INT8", None
        for line in text.splitlines():
            s = line.strip()
            if s.endswith("{"):
                depth += 1
                continue
            if s == "}":
                depth = max(0, depth - 1)
                continue
            if depth:
                continue
            key, _, val = s.partition(":")
            key, val = key.strip(), val.strip()
            try:
                if key == "width":
                    w = int(float(val))
                elif key == "height":
                    h = int(float(val))
                elif key in ("pixel_format_type", "pixel_format"):
                    fmt = cls._PIX.get(int(val), fmt) if val.isdigit() else val
                elif key == "data" and val.startswith('"'):
                    body = val[1:-1] if val.endswith('"') else val[1:]
                    data = body.encode("latin-1", "ignore").decode("unicode_escape").encode("latin-1", "ignore")
            except ValueError:
                continue
        if data is None or w <= 0 or h <= 0:
            return None
        return w, h, fmt, data

    def _watch(self, started):
        """Wait while frames arrive; return when we should re-detect (restart / stop / nothing for 6 s)."""
        while not self._halt.is_set() and not self._restart.is_set():
            self._restart.wait(0.5)
            with self._lock:
                t = self.frame["t"] if self.frame else 0.0
            if time.time() - max(t, started) > 6.0:
                return

    def _run_stream(self, topic, node_cls, image_cls):
        node = node_cls()
        if not node.subscribe(image_cls, topic, self._on_msg):
            self._set("error", f"could not subscribe to {topic}")
            self._halt.wait(4.0)
            return
        self.mode = "stream"
        self._set("running", f"streaming {topic}")
        self._watch(time.time())
        try:
            node.unsubscribe(topic)
        except Exception:
            pass

    def _run_preview(self, gz, topic):
        self.mode = "preview"
        self._set("running", f"preview of {topic} (install python3-gz-transport13 python3-gz-msgs10 for smooth video)")
        started, last_ok = time.time(), time.time()
        while not self._halt.is_set() and not self._restart.is_set():
            t0 = time.time()
            out = self._run([gz, "topic", "-e", "-t", topic, "-n", "1"], 8)
            got = self._parse_text(out) if out else None
            if got:
                self._store(*got)
                last_ok = time.time()
            elif time.time() - max(last_ok, started) > 8.0:
                return
            self._restart.wait(max(0.0, 0.25 - (time.time() - t0)))

    def run(self):
        while not self._halt.is_set():
            self._restart.clear()
            gz = self._gz()
            if not gz:
                self._set("error", "'gz' command not found - is Gazebo installed and on the PATH?")
                self._halt.wait(5.0)
                continue
            out = self._run([gz, "topic", "-l"], 6)
            topics = [ln.strip() for ln in (out or "").splitlines() if ln.strip().startswith("/")]
            if not topics:
                self._set("waiting", "no Gazebo topics visible - is the simulator running?")
                self._halt.wait(3.0)
                continue
            topic = self._choose(gz, topics)
            with self._lock:
                self.topic = topic
            if not topic:
                self._set("waiting", "Gazebo is up but no camera image topic was found yet")
                self._halt.wait(3.0)
                continue
            try:
                b = self._bindings()
                if b:
                    self._run_stream(topic, *b)
                else:
                    self._run_preview(gz, topic)
            except Exception as e:
                self._set("error", f"camera: {e}")
                self._halt.wait(3.0)


class ToggleSwitch(tk.Canvas):
    """A small sliding on/off switch (the knob glides from one end to the other)."""

    def __init__(self, parent, value=True, command=None, bg=COL_BTN, width=54, height=28,
                 on_color="#3f8f6b", off_color="#4a4a52"):
        super().__init__(parent, width=width, height=height, bg=bg, highlightthickness=0, cursor="hand2")
        self._sw_w, self._sw_h = width, height
        self._on_c, self._off_c = on_color, off_color
        self._cmd = command
        self.value = bool(value)
        self._pos = 1.0 if self.value else 0.0
        self._job = None
        self.bind("<Button-1>", lambda e: self.toggle())
        self._draw()

    def toggle(self):
        self.set(not self.value, notify=True)

    def set(self, value, notify=False):
        value = bool(value)
        changed = value != self.value
        self.value = value
        if self._job is None:
            self._job = self.after(0, self._step)
        if changed and notify and self._cmd:
            self._cmd(self.value)

    def _step(self):
        self._job = None
        tgt = 1.0 if self.value else 0.0
        d = tgt - self._pos
        self._pos = tgt if abs(d) < 0.06 else self._pos + d * 0.35
        self._draw()
        if self._pos != tgt:
            self._job = self.after(16, self._step)

    def _draw(self):
        self.delete("all")
        w, h = self._sw_w, self._sw_h
        col = _mix_hex(self._off_c, self._on_c, self._pos)
        self.create_oval(0, 0, h, h, fill=col, outline="")
        self.create_oval(w - h, 0, w, h, fill=col, outline="")
        self.create_rectangle(h / 2, 0, w - h / 2, h, fill=col, outline="")
        x = 3 + self._pos * (w - h)
        self.create_oval(x, 3, x + h - 6, h - 3, fill="#f2f2f2", outline="")


class OdoPanelMixin:
    """VISUAL ODO panel -- slides in exactly like SWARM NAV.

    * GPS switch: OFF = the dashboard drops every GPS value it would receive
    * manual GPS input + SET: marks a home location on the main map
    * data source: GAZEBO (bridge to the running simulator) or TELEMETRY (the MAVLink link),
      by default following the GCS SIM / TELE mode
    * live IMU / attitude / altitude readout of the active source

    For the GeoAnchor code that comes next:  self.odo_sample()  returns the latest IMU /
    attitude / altitude of the active source, self.gps_enabled and self.odo_home hold the
    GPS switch and the manual home location.
    """

    _ODO_FRESH_S = 1.5

    # ------------------------------------------------------------------ state
    def _odo_init_state(self):
        self.odo_enabled = False             # MASTER switch at the top of the VISUAL ODO panel (default OFF)
        self._odo_gps_saved = True           # GPS switch value remembered while the master switch is OFF
        self.gps_enabled = True
        self.odo_home = None                 # (lat, lon) set by hand in the VISUAL ODO panel
        self._odo_home_marker = None
        self._odo_prog = 0.0                 # 0 = fully out (off-screen right), 1 = fully in
        self._odo_target = 0
        self._odo_job = None
        self._odo_follow = True              # source follows the GCS SIM / TELE mode
        self._odo_src_manual = "gazebo"      # used when _odo_follow is False
        self._odo_started = False            # becomes True the first time the panel is opened
        self._odo_bridge = None
        self._cam = None                     # GazeboCamera thread (runs while the source is GAZEBO)
        self._odo_wraps = []
        # --- Gazebo auto-connect + live telemetry feed ---
        self.gz_auto = True                  # detect a running Gazebo and connect by itself (no manual setup)
        self.gz_dr = True                    # drone position / attitude come from the IMU (dead reckoning), not from the truth pose
        self._gz_trail = []                  # yellow: where the drone THINKS it went (IMU estimate)
        self._gz_truth = []                  # light blue: where Gazebo says it really went (comparison only)
        self._gz_trail_path = None
        self._gz_truth_path = None
        self._gz_trail_n = 0
        self._gz_watch = None                # GazeboWatcher thread
        self._gz_org = None                  # local (x, y) <-> lat / lon origin
        self.gz_k = 1.0                      # real metres per Gazebo metre (found by GeoAnchor at the first fix)
        self.gz_hdg = 90.0                   # compass bearing of Gazebo +x (the camera's image-up)
        self.gz_baro = True                  # barometer aid for the altitude
        self._ga = {"state": "off", "text": "", "n": 0, "inl": None, "mode": "", "ms": None, "lock": None,
                    "last_rx": None, "last_res": None, "log_t": 0.0, "hist": []}
        self._ga_proc = None
        self._ga_logf = None
        self._ga_sock = None
        self._gz_live = False
        self._gz_was_live = False
        self._gz_err_logged = False
        self._gz_logged_connect = False
        self._odo_tele = {"imu": None, "att": None, "alt": None, "hdg": None, "climb": None,
                          "n": 0, "t0": time.time(), "hz": 0.0}
        self._fl_init_state()                # FLIGHT LOG card: recorder + replay state

    # ------------------------------------------------------------------ build
    def _build_odo_panel(self):
        panel_w = lambda W, H: max(560, int(W * 0.5) - 8)
        p = GlassPanel(self, bg=COL_BTN, border=COL_BORDER, alpha=0.97,
                       start_visible=False, popup=True)
        p.set_geometry(
            relx=1.0, rely=0.0, anchor="ne",
            dx=lambda W, H: -8 + int((1.0 - self._odo_prog) * (panel_w(W, H) + 30)),
            dy=8, width=panel_w,
            height=lambda W, H: max(260, H - self.banner_panel.frame.winfo_reqheight() - 30))
        self.odo_panel = p

        head = tk.Frame(p.body, bg=COL_BTN)
        head.pack(fill="x", padx=12, pady=(12, 6))
        back = tk.Label(head, text=">>", font=("Segoe UI", 13, "bold"), fg=COL_TEXT,
                        bg=COL_BORDER, padx=14, pady=2, cursor="hand2")
        back.pack(side="left")
        back.bind("<Enter>", lambda e: back.config(bg="#4a4a50"))
        back.bind("<Leave>", lambda e: back.config(bg=COL_BORDER))
        back.bind("<Button-1>", lambda e: self._odo_slide(0))
        tk.Label(head, text="VISUAL ODO", font=("Segoe UI", 15, "bold"), fg=COL_TEXT,
                 bg=COL_BTN).pack(side="left", padx=(14, 10))
        self._odo_master = ToggleSwitch(head, value=False, command=self._odo_on_master, bg=COL_BTN,
                                        width=44, height=22)
        self._odo_master.pack(side="left")
        self._odo_master_lbl = tk.Label(head, text="OFF", font=("Segoe UI", 10, "bold"),
                                        fg=COL_TEXT_DIM, bg=COL_BTN)
        self._odo_master_lbl.pack(side="left", padx=8)
        self._odo_head_src = tk.Label(head, text="", font=("Segoe UI", 9, "bold"), fg=COL_TEXT_DIM, bg=COL_BTN)
        self._odo_head_src.pack(side="right", padx=6)

        wrap = tk.Frame(p.body, bg=COL_BTN)
        wrap.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self._odo_canvas = tk.Canvas(wrap, bg=COL_BTN, highlightthickness=0)
        sb = tk.Scrollbar(wrap, orient="vertical", command=self._odo_canvas.yview)
        self._odo_canvas.configure(yscrollcommand=sb.set)
        self._odo_canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self._odo_container = tk.Frame(self._odo_canvas, bg=COL_BTN)
        win = self._odo_canvas.create_window((0, 0), window=self._odo_container, anchor="nw")
        self._odo_container.bind(
            "<Configure>", lambda e: self._odo_canvas.configure(scrollregion=self._odo_canvas.bbox("all")))

        def _resize(e):
            self._odo_canvas.itemconfig(win, width=e.width)
            for w in self._odo_wraps:
                try:
                    w.config(wraplength=max(200, e.width - 50))
                except tk.TclError:
                    pass
        self._odo_canvas.bind("<Configure>", _resize)

        def _wheel(event):
            if getattr(event, "num", None) == 4:
                self._odo_canvas.yview_scroll(-1, "units")
            elif getattr(event, "num", None) == 5:
                self._odo_canvas.yview_scroll(1, "units")
            else:
                self._odo_canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")

        def _bind(_e):
            for ev in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
                self._odo_canvas.bind_all(ev, _wheel)

        def _unbind(_e):
            for ev in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
                self._odo_canvas.unbind_all(ev)

        self._odo_canvas.bind("<Enter>", _bind)
        self._odo_canvas.bind("<Leave>", _unbind)

        self._build_odo_gps_card()
        self._build_odo_home_card()
        self._build_odo_source_card()
        self._build_odo_geoanchor_card()          # _GA_CLEAN_UI: GeoAnchor first, then the live sensor data
        self._build_odo_data_card()
        self._build_odo_bridge_card()
        self._build_odo_flight_card()             # FLIGHT LOG: record + synchronized replay (bottom of the panel)
        self._gz_watch = GazeboWatcher()
        self._gz_watch.start()
        self.root.after(300, self._odo_tick)
        self.root.after(600, self._gz_tick)

    def _odo_card(self, title, subtitle=""):
        CARD = self._NAV_CARD
        fr = tk.Frame(self._odo_container, bg=CARD, highlightthickness=1, highlightbackground=COL_BORDER)
        fr.pack(fill="x", pady=6, padx=2)
        hd = tk.Frame(fr, bg=CARD)
        hd.pack(fill="x", padx=12, pady=(10, 0))
        tk.Frame(hd, bg=self._NV_ACC, width=3, height=16).pack(side="left", padx=(0, 8))
        tk.Label(hd, text=title, font=("Segoe UI", 11, "bold"), fg=COL_TEXT, bg=CARD).pack(side="left")
        badge = tk.Label(hd, text="", font=("Segoe UI", 9, "bold"), fg=COL_TEXT_DIM, bg=CARD)
        badge.pack(side="right")
        if subtitle:
            sub = tk.Label(fr, text=subtitle, font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=CARD,
                           anchor="w", justify="left", wraplength=560)
            sub.pack(fill="x", padx=12, pady=(2, 0))
            self._odo_wraps.append(sub)
        tk.Frame(fr, bg=COL_BORDER, height=1).pack(fill="x", padx=12, pady=(7, 8))
        body = tk.Frame(fr, bg=CARD)
        body.pack(fill="x", padx=12, pady=(0, 12))
        return body, badge

    def _odo_entry(self, parent, width, default=""):
        e = tk.Entry(parent, width=width, font=("Segoe UI", 10), bg="#111113", fg=COL_TEXT,
                     insertbackground=COL_TEXT, relief="flat", highlightthickness=1,
                     highlightbackground=COL_BORDER, highlightcolor=self._NV_ACC)
        if default:
            e.insert(0, default)
        return e

    def _odo_button(self, parent, text, command, kind="default", width=10):
        b = self._ui_button(parent, text, command, kind=kind)
        b.config(width=width)
        return b

    # ---- card 1: GPS on / off ----
    def _build_odo_gps_card(self):
        CARD = self._NAV_CARD
        body, self._odo_gps_badge = self._odo_card(
            "GPS", "OFF = the dashboard drops every GPS value it would receive (telemetry latitude / "
                   "longitude, satellites, drone position on the map) - from every source. ON = normal.")
        row = tk.Frame(body, bg=CARD)
        row.pack(fill="x")
        tk.Label(row, text="GPS", font=("Segoe UI", 12, "bold"), fg=COL_TEXT, bg=CARD).pack(side="left")
        self._odo_gps_switch = ToggleSwitch(row, value=True, command=self._odo_on_gps_toggle, bg=CARD,
                                            width=38, height=20)
        self._odo_gps_switch.pack(side="left", padx=14)
        self._odo_gps_state = tk.Label(row, text="", font=("Segoe UI", 10, "bold"), bg=CARD)
        self._odo_gps_state.pack(side="left")
        self._odo_gps_refresh()

    def _odo_on_master(self, state):
        """Master VISUAL ODO switch: ON = Gazebo bridge + camera run (and the GPS block works);
        OFF = everything stopped and the GPS block is released (normal GCS)."""
        self.odo_enabled = bool(state)
        if self.odo_enabled:
            self._odo_started = True
            if not self._odo_gps_saved:
                self._odo_gps_switch.set(False)
                self._odo_on_gps_toggle(False)
        else:
            self._odo_gps_saved = self.gps_enabled
            if not self.gps_enabled:
                self._odo_gps_switch.set(True)
                self._odo_on_gps_toggle(True, force=True)
        self._odo_master_lbl.config(text="ON" if self.odo_enabled else "OFF",
                                    fg=self._NV_OK if self.odo_enabled else COL_TEXT_DIM)
        self._append_log(">>> VISUAL ODO switched ON" if self.odo_enabled else ">>> VISUAL ODO switched OFF")
        self._odo_tick(reschedule=False)
        self._cam_key = None

    def _odo_on_gps_toggle(self, state, force=False):
        if not state and not self.odo_enabled and not force:
            self._odo_gps_switch.set(True)
            self._odo_gps_state.config(text="Switch VISUAL ODO ON first", fg=self._NV_WARN)
            self.root.after(1800, self._odo_gps_refresh)
            return
        self.gps_enabled = bool(state)
        self._odo_gps_refresh()
        self._gps_apply()

    def _odo_gps_refresh(self):
        if self.gps_enabled:
            self._odo_gps_state.config(text="ON  -  GPS data is received", fg=self._NV_OK)
            self._odo_gps_badge.config(text="\u25cf ON", fg=self._NV_OK)
        else:
            self._odo_gps_state.config(text="OFF  -  GPS data is blocked", fg=self._NV_BAD)
            self._odo_gps_badge.config(text="\u25cf BLOCKED", fg=self._NV_BAD)

    def _gps_text(self, v):
        return f"{v:.6f}" if self.gps_enabled else "GPS OFF"

    def _gps_apply(self):
        """Repaint every GPS readout after the switch changed."""
        on = self.gps_enabled
        if on:
            self.gps_overlay_label.config(text="GPS: --", fg=COL_TEXT)
        else:
            self.gps_overlay_label.config(text="GPS: OFF", fg=COL_ARMED)
        for sid in list(self.swarm_widgets):
            for key in ("lat", "lon"):
                self._update_swarm_field(sid, key, "--" if on else "GPS OFF", color=None if on else COL_ARMED)
        for key in ("lat", "lon"):
            w = self.values.get(key)
            if w is not None:
                w.config(text="--" if on else "GPS OFF", fg=COL_VALUE if on else COL_ARMED)
        self._append_log(">>> GPS switched ON - GPS data is received again" if on else
                         ">>> GPS switched OFF - all GPS data is blocked in the dashboard")

    # ---- card 2: manual home location ----
    def _build_odo_home_card(self):
        CARD = self._NAV_CARD
        body, self._odo_home_badge = self._odo_card(
            "HOME LOCATION  -  manual GPS",
            "Left-click the main map: the coordinate is shown and copied automatically. Paste it here "
            "(e.g.  28.455365, 77.499867)  and press SET or Enter. A pin 'H' is placed on the main map.")
        row = tk.Frame(body, bg=CARD)
        row.pack(fill="x")
        tk.Label(row, text="Lat, Lon", font=("Segoe UI", 9), fg=COL_TEXT_DIM, bg=CARD).pack(side="left")
        self._odo_home_entry = self._odo_entry(row, 34)
        self._odo_home_entry.pack(side="left", padx=(8, 14), ipady=3)
        self._odo_home_entry.bind("<Return>", lambda e: self._odo_set_home())
        self._odo_button(row, "SET", self._odo_set_home, kind="primary", width=8).pack(side="left", ipady=2)
        self._odo_button(row, "CLEAR", self._odo_clear_home, kind="danger", width=8).pack(side="left", padx=(6, 0), ipady=2)
        self._odo_home_lbl = tk.Label(body, text="No home location set", font=("Segoe UI", 9),
                                      fg=COL_TEXT_DIM, bg=CARD, anchor="w")
        self._odo_home_lbl.pack(fill="x", pady=(8, 0))

    @staticmethod
    def _odo_parse_latlon(text):
        """'28.455365, 77.499867'  ->  (28.455365, 77.499867); None if it is not two numbers."""
        nums = re.findall(r"[-+]?\d+(?:\.\d+)?", text or "")
        if len(nums) != 2:
            return None
        return float(nums[0]), float(nums[1])

    def _odo_set_home(self):
        ll = self._odo_parse_latlon(self._odo_home_entry.get())
        if ll is None:
            self._odo_home_lbl.config(text="Paste two numbers, e.g.  28.455365, 77.499867", fg=self._NV_BAD)
            return
        lat, lon = ll
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            self._odo_home_lbl.config(text="Latitude must be -90..90 and longitude -180..180", fg=self._NV_BAD)
            return
        self.odo_home = (lat, lon)
        self._odo_home_entry.delete(0, "end")
        self._odo_home_entry.insert(0, f"{lat:.7f}, {lon:.7f}")
        if self._odo_home_marker is not None:
            try:
                self._odo_home_marker.delete()
            except Exception:
                pass
            self._odo_home_marker = None
        try:
            self._odo_home_marker = self.map_widget.set_marker(
                lat, lon, icon=self._get_wp_pin("H"), icon_anchor="s")
            self.map_widget.set_position(lat, lon)
        except Exception as e:
            self._append_log(f"!!! Could not draw home marker: {e}")
        try:                                           # the drone starts from the home location
            self.last_lat, self.last_lon = lat, lon
            if self.drone_marker:
                self.drone_marker.set_position(lat, lon)
        except Exception:
            pass
        self._append_log(f"  start the SITL drone there with:  sim_vehicle.py -v ArduCopter ... "
                         f"--custom-location={lat:.7f},{lon:.7f},<alt_m>,0")
        self._odo_home_lbl.config(text=f"Home set  \u00b7  {lat:.7f}, {lon:.7f}   (pin 'H' + drone on the map)", fg=self._NV_OK)
        self._odo_home_badge.config(text="\u25cf SET", fg=self._NV_OK)
        self._append_log(f">>> VISUAL ODO home location set: {lat:.7f}, {lon:.7f}")

    def _odo_clear_home(self):
        if self._odo_home_marker is not None:
            try:
                self._odo_home_marker.delete()
            except Exception:
                pass
            self._odo_home_marker = None
        self.odo_home = None
        self._odo_home_entry.delete(0, "end")
        self._odo_home_lbl.config(text="No home location set", fg=COL_TEXT_DIM)
        self._odo_home_badge.config(text="")

    # ---- card 3: data source (GAZEBO / TELEMETRY) ----
    def _build_odo_source_card(self):
        CARD = self._NAV_CARD
        body, self._odo_src_badge = self._odo_card(
            "DATA SOURCE",
            "Where the IMU, attitude and altitude come from. GAZEBO = the running simulator (through the "
            "Gazebo bridge). TELEMETRY = the real MAVLink link of the flight controller.")
        seg = tk.Frame(body, bg=COL_BORDER, padx=1, pady=1)
        seg.pack(anchor="w")
        self._odo_seg = {}
        for key, text in (("gazebo", "GAZEBO"), ("telemetry", "TELEMETRY")):
            lb = tk.Label(seg, text=text, font=("Segoe UI", 9, "bold"), width=11, pady=3, cursor="hand2")
            lb.pack(side="left")
            lb.bind("<Button-1>", lambda e, k=key: self._odo_pick_source(k))
            self._odo_seg[key] = lb
        row = tk.Frame(body, bg=CARD)
        row.pack(fill="x", pady=(10, 0))
        self._odo_follow_sw = ToggleSwitch(row, value=True, command=self._odo_on_follow, bg=CARD,
                                           width=32, height=16)
        self._odo_follow_sw.pack(side="left")
        tk.Label(row, text="Follow the GCS mode  (SIM \u2192 Gazebo,  TELE \u2192 Telemetry)",
                 font=("Segoe UI", 9), fg=COL_TEXT_DIM, bg=CARD).pack(side="left", padx=10)
        # live indicator
        ind = tk.Frame(body, bg=CARD)
        ind.pack(fill="x", pady=(10, 0))
        self._odo_led = tk.Canvas(ind, width=14, height=14, bg=CARD, highlightthickness=0)
        self._odo_led.pack(side="left")
        self._odo_led_id = self._odo_led.create_oval(2, 2, 12, 12, fill=self._NV_OFF, outline="")
        self._odo_ind_lbl = tk.Label(ind, text="", font=("Segoe UI", 10, "bold"), fg=COL_TEXT_DIM, bg=CARD)
        self._odo_ind_lbl.pack(side="left", padx=8)

    def _odo_on_follow(self, state):
        self._odo_follow = bool(state)
        if not self._odo_follow:
            self._odo_src_manual = self._odo_src()
        self._odo_refresh_ui()

    def _odo_pick_source(self, key):
        self._odo_follow = False
        self._odo_follow_sw.set(False)
        self._odo_src_manual = key
        self._odo_refresh_ui()

    def _odo_src(self):
        """'gazebo' or 'telemetry' - the source that is active right now."""
        if self._odo_follow:
            mode = getattr(self.mav, "mode", None) or (self._current_target[0] if self._current_target else "SIM")
            return "gazebo" if mode == "SIM" else "telemetry"
        return self._odo_src_manual

    # ---- card 4: live data ----
    def _build_odo_data_card(self):
        CARD, TILE = self._NAV_CARD, self._NAV_TILE
        body, self._odo_data_badge = self._odo_card("LIVE SENSOR DATA")
        grid = tk.Frame(body, bg=CARD)
        grid.pack(fill="x")
        for c in range(3):
            grid.grid_columnconfigure(c, weight=1, uniform="odo")
        spec = [("ACCEL X  (m/s\u00b2)", "ax"), ("ACCEL Y  (m/s\u00b2)", "ay"), ("ACCEL Z  (m/s\u00b2)", "az"),
                ("GYRO X  (\u00b0/s)", "gx"), ("GYRO Y  (\u00b0/s)", "gy"), ("GYRO Z  (\u00b0/s)", "gz"),
                ("ROLL  (\u00b0)", "roll"), ("PITCH  (\u00b0)", "pitch"), ("HEADING  (\u00b0)", "yaw"),
                ("ALTITUDE  (m)", "alt"), ("CLIMB  (m/s)", "climb"), ("DATA AGE", "age")]
        self._odo_vals = {}
        for i, (cap, key) in enumerate(spec):
            f = tk.Frame(grid, bg=TILE, highlightthickness=1, highlightbackground=COL_BORDER)
            f.grid(row=i // 3, column=i % 3, sticky="nsew", padx=3, pady=3)
            tk.Label(f, text=cap, font=("Segoe UI", 7, "bold"), fg=COL_TEXT_DIM, bg=TILE,
                     anchor="w").pack(fill="x", padx=8, pady=(6, 0))
            v = tk.Label(f, text="--", font=("DejaVu Sans Mono", 13, "bold"), fg=self._NV_VAL, bg=TILE, anchor="w")
            v.pack(fill="x", padx=8, pady=(0, 6))
            self._odo_vals[key] = v
        tk.Label(body, text="Body frame: x forward, y right, z down (same as the autopilot).  Accel includes "
                            "gravity (about -9.8 on z when level).",
                 font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=CARD, anchor="w", justify="left",
                 wraplength=560).pack(fill="x", pady=(6, 0))
        self._odo_wraps.append(body.winfo_children()[-1])

    # ---- card: GeoAnchor visual positioning (primary) + IMU (carries the position between fixes) ----
    _GA_HOME = os.path.expanduser("~/drone_workspace/projects/Visual Odometry")
    _GA_UDP_PORT = 15900
    _GA_STATE_PORT = 15901              # attitude / altitude forwarded to the node (real flight)
    _GA_MODES_NO_FIX = ("odometry only", "blurry, match skipped")     # no new absolute map match -> not a fix

    def _ga_default_map(self):
        for c in (os.path.join(PROJECT_ROOT, "real_map.jpg"), os.path.join(PROJECT_ROOT, "real_map.tif"),
                  os.path.join(self._GA_HOME, "GeoAnchor", "real_map.jpg"),
                  os.path.join(self._GA_HOME, "real_map.jpg"),
                  os.path.join(self._GA_HOME, "galgotias_world", "real_map.jpg")):
            if os.path.isfile(c):
                return c
        return ""

    def _build_odo_geoanchor_card(self):
        CARD, TILE = self._NAV_CARD, self._NAV_TILE
        body, self._ga_badge = self._odo_card(
            "GEOANCHOR  -  visual positioning",
            "The camera image is matched against the satellite map (direct feature matching every 2 s) and gives an "
            "ABSOLUTE position - this is the primary position source and removes the drift. The IMU only carries "
            "the position between two visual fixes; the barometer holds the altitude.")
        row = tk.Frame(body, bg=CARD)
        row.pack(fill="x")
        tk.Label(row, text="Visual positioning", font=("Segoe UI", 12, "bold"), fg=COL_TEXT, bg=CARD).pack(side="left")
        self._ga_switch = ToggleSwitch(row, value=False, command=self._ga_on_toggle, bg=CARD, width=38, height=20)
        self._ga_switch.pack(side="left", padx=14)
        self._ga_state_lbl = tk.Label(row, text="OFF", font=("Segoe UI", 10, "bold"), fg=COL_TEXT_DIM, bg=CARD)
        self._ga_state_lbl.pack(side="left")

        grid = tk.Frame(body, bg=CARD)
        grid.pack(fill="x", pady=(10, 0))
        grid.grid_columnconfigure(1, weight=1)
        self._ga_cfg = {}
        for r, (key, label, default, hint) in enumerate((
                ("alt", "Cruise altitude (m)", "150", "same as CRUISE_ALT in fly_smooth.py"),
                ("camera", "Camera (real drone)", "", "empty = Gazebo.  real: 0 / /dev/video0 / rtsp://... / file.mp4"),
                ("radius", "Map margin (m)", "150", "extra map around HOME + all marked points (internet needed once)"),
                ("python", "GeoAnchor python", os.path.join(self._GA_HOME, "GeoAnchor", ".venv", "bin", "python"),
                 "python with opencv + numpy (the GeoAnchor venv works)"))):
            tk.Label(grid, text=label, font=("Segoe UI", 9), fg=COL_TEXT_DIM, bg=CARD,
                     anchor="w").grid(row=r, column=0, sticky="w", pady=3, padx=(0, 10))
            e = self._odo_entry(grid, 44, default)
            e.grid(row=r, column=1, sticky="ew", pady=3, ipady=3)
            tk.Label(grid, text=hint, font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=CARD,
                     anchor="w").grid(row=r, column=2, sticky="w", padx=(8, 0))
            self._ga_cfg[key] = e
        class _Fixed:                                   # no box any more: the value is fixed
            def __init__(self, v):
                self.v = v
            def get(self):
                return self.v
        self._ga_cfg["map"] = _Fixed("auto")           # satellite tiles around HOME (cached -> works offline)
        self._ga_cfg["bounds"] = _Fixed("")

        brow = tk.Frame(body, bg=CARD)
        brow.pack(fill="x", pady=(8, 0))
        self._ga_baro_sw = ToggleSwitch(brow, value=True, command=self._ga_on_baro, bg=CARD, width=32, height=16)
        self._ga_baro_sw.pack(side="left")
        tk.Label(brow, text="Barometer holds the altitude  (stand-in: Gazebo z + 0.25 m noise; a Pixhawk has a real one)",
                 font=("Segoe UI", 9), fg=COL_TEXT_DIM, bg=CARD).pack(side="left", padx=10)

        # --- big fusion indicator: which source is carrying the position right now ---
        box = tk.Frame(body, bg=TILE, highlightthickness=1, highlightbackground=COL_BORDER)
        box.pack(fill="x", pady=(10, 0))
        top = tk.Frame(box, bg=TILE)
        top.pack(fill="x", padx=10, pady=(8, 0))
        self._ga_led = tk.Canvas(top, width=14, height=14, bg=TILE, highlightthickness=0)
        self._ga_led.pack(side="left")
        self._ga_led_id = self._ga_led.create_oval(2, 2, 12, 12, fill=self._NV_OFF, outline="")
        self._ga_src_lbl = tk.Label(top, text="IMU ONLY", font=("Segoe UI", 12, "bold"), fg=COL_TEXT_DIM, bg=TILE)
        self._ga_src_lbl.pack(side="left", padx=8)
        self._ga_node_lbl = tk.Label(box, text="", font=("Segoe UI", 9), fg=COL_TEXT_DIM, bg=TILE, anchor="w",
                                     justify="left", wraplength=560)
        self._ga_node_lbl.pack(fill="x", padx=10, pady=(2, 0))
        self._ga_stat_lbl = tk.Label(box, text="", font=("DejaVu Sans Mono", 9), fg=self._NV_VAL, bg=TILE,
                                     anchor="w", justify="left")
        self._ga_stat_lbl.pack(fill="x", padx=10, pady=(4, 8))
        self._odo_wraps.append(self._ga_node_lbl)

    def _ga_on_baro(self, state):
        self.gz_baro = bool(state)
        self._append_log(">>> Barometer aid " + ("ON (altitude held by the barometer)" if self.gz_baro
                                                  else "OFF (altitude from the IMU only - it will drift)"))

    def _ga_on_toggle(self, state):
        if state:
            if not self._ga_start():
                self._ga_switch.set(False)
        else:
            self._ga_stop()
            self._append_log(">>> GeoAnchor visual positioning OFF - the IMU estimate is on its own again")

    def _ga_points(self):
        """HOME + every marked waypoint (all drones) as [(lat, lon), ...] - the map has to cover all of them."""
        pts = []
        if self.odo_home:
            pts.append((float(self.odo_home[0]), float(self.odo_home[1])))
        for wl in list(self.manual_waypoints.values()):
            for p in wl:
                q = (float(p[0]), float(p[1]))
                if q not in pts:
                    pts.append(q)
        return pts

    def _ga_start(self):
        if self._ga_proc is not None and self._ga_proc.poll() is None:
            return True
        cfg = {k: e.get().strip() for k, e in self._ga_cfg.items()}
        node = os.path.join(PROJECT_ROOT, "geoanchor_node.py")
        problem = None
        if not os.path.isfile(node):
            problem = f"geoanchor_node.py not found next to this file ({node})"
        elif cfg["map"].lower() in ("", "auto") and not self.odo_home:
            problem = "map  auto  needs the HOME position - set it in VISUAL ODO first (paste lat, lon and press SET)"
        elif cfg["map"].lower() not in ("", "auto") and not os.path.isfile(os.path.expanduser(cfg["map"])):
            problem = "Satellite map not found - type the path of the map image, or  auto  to use tiles around HOME"
        elif self._odo_src() == "telemetry" and not cfg["camera"]:
            problem = "real drone: type the camera source (0, /dev/video0, rtsp://... or a video file) in the GEOANCHOR card"
        elif not os.path.isfile(os.path.expanduser(cfg["python"])):
            problem = f"GeoAnchor python not found: {cfg['python']}"
        try:
            alt = float(cfg["alt"])
        except ValueError:
            alt, problem = 150.0, problem or "Cruise altitude must be a number"
        if problem:
            self._ga.update(state="error", text=problem)
            self._append_log(f"!!! GeoAnchor: {problem}")
            return False
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("127.0.0.1", self._GA_UDP_PORT))
            sock.setblocking(False)
        except OSError as e:
            self._ga.update(state="error", text=f"UDP port {self._GA_UDP_PORT} busy ({e})")
            self._append_log(f"!!! GeoAnchor: UDP port {self._GA_UDP_PORT} is busy ({e})")
            return False
        cmd = [os.path.expanduser(cfg["python"]), "-u", node, "--map",
               ("auto" if cfg["map"].lower() in ("", "auto") else os.path.expanduser(cfg["map"])),
               "--alt", f"{alt:g}", "--port", str(self._GA_UDP_PORT), "--state-port", str(self._GA_STATE_PORT)]
        if self.odo_home:
            cmd += ["--home", "%.7f,%.7f" % tuple(self.odo_home)]
        if cfg["camera"]:
            cmd += ["--video", cfg["camera"]]
        pts = self._ga_points()
        self._ga_area_warned = False
        try:
            margin = float(cfg["radius"])
        except ValueError:
            margin = 150.0
        if len(pts) >= 2:                                   # HOME + at least one marked point: cover all of them
            cmd += ["--points", ";".join("%.7f,%.7f" % p for p in pts), "--margin-m", f"{margin:g}"]
            self._ga_area = (pts, margin)
            self._append_log(f">>> GeoAnchor map area: HOME + {len(pts) - 1} marked point(s) + {margin:g} m margin")
        else:                                               # no target marked yet: 500 m around HOME
            cmd += ["--radius-m", "500"]
            self._ga_area = (pts[:1], 500.0) if pts else None
            self._append_log(">>> GeoAnchor map area: 500 m around HOME (no waypoint marked yet)")
        if cfg["bounds"]:
            cmd += ["--bounds", cfg["bounds"]]
        try:
            self._ga_logf = open(os.path.join(PROJECT_ROOT, "geoanchor_node.log"), "w")
            self._ga_proc = subprocess.Popen(cmd, stdout=self._ga_logf, stderr=subprocess.STDOUT, cwd=PROJECT_ROOT)
        except Exception as e:
            sock.close()
            self._ga.update(state="error", text=f"could not start the GeoAnchor node: {e}")
            self._append_log(f"!!! GeoAnchor: could not start the node: {e}")
            return False
        self._ga_sock = sock
        self._ga.update(state="loading", text="starting GeoAnchor (first start embeds the map tiles - can take minutes) ...",
                        n=0, inl=None, mode="", ms=None, lock=None, last_rx=None, last_res=None, hist=[])
        self._ga_clear_dots()
        self.gz_k, self.gz_hdg = 1.0, 90.0
        self._append_log(">>> GeoAnchor started - the camera image now gives the absolute position, the IMU carries it "
                         "between fixes (log: geoanchor_node.log)")
        return True

    def _ga_stop(self):
        p, self._ga_proc = self._ga_proc, None
        if p is not None and p.poll() is None:
            try:
                p.terminate()
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    p.kill()
            except Exception:
                pass
        if self._ga_logf is not None:
            try:
                self._ga_logf.close()
            except Exception:
                pass
            self._ga_logf = None
        if self._ga_sock is not None:
            try:
                self._ga_sock.close()
            except OSError:
                pass
            self._ga_sock = None
        self._ga.update(state="off", text="", lock=None)

    def _ga_poll(self):
        """Read what the GeoAnchor node reports; every ga_fix goes straight into the fusion filter."""
        sock = self._ga_sock
        if sock is None:
            return
        now_ = time.time()
        if now_ - self.__dict__.get("_ga_state_t", 0.0) >= 0.1:        # roll / pitch / heading / altitude -> node
            self._ga_state_t = now_
            try:
                s_ = self.odo_sample()
                sock.sendto(json.dumps({"k": "ga_state", "t": now_, "roll": s_["roll"], "pitch": s_["pitch"],
                                        "heading": s_["heading"], "alt": s_["alt"], "fresh": s_["fresh"]}).encode(),
                            ("127.0.0.1", self._GA_STATE_PORT))
            except Exception:
                pass
        area = self.__dict__.get("_ga_area")
        if area and now_ - self.__dict__.get("_ga_area_t", 0.0) >= 2.0 and not self.__dict__.get("_ga_area_warned"):
            self._ga_area_t = now_                          # a point marked later, outside the downloaded map?
            try:
                pts0, mg = area
                lat0 = sum(p[0] for p in pts0) / len(pts0)
                dlat, dlon = mg / 111320.0, mg / (111320.0 * math.cos(math.radians(lat0)))
                lo_la, hi_la = min(p[0] for p in pts0) - dlat, max(p[0] for p in pts0) + dlat
                lo_lo, hi_lo = min(p[1] for p in pts0) - dlon, max(p[1] for p in pts0) + dlon
                if any(not (lo_la <= p[0] <= hi_la and lo_lo <= p[1] <= hi_lo) for p in self._ga_points()):
                    self._ga_area_warned = True
                    self._append_log("!!! GeoAnchor: a marked point is OUTSIDE the downloaded map - switch Visual "
                                     "positioning off and on again so the map covers it (internet needed once)")
            except Exception:
                pass
        for _ in range(100):
            try:
                data, _a = sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError, OSError):
                break
            try:
                m = json.loads(data.decode())
            except ValueError:
                continue
            kind = m.get("k") if isinstance(m, dict) else None
            if kind == "ga_status":
                self._ga["state"], self._ga["text"] = m.get("state", ""), m.get("text", "")
            elif kind == "ga_fix":
                try:
                    self._ga_handle_fix(m)
                except Exception as e:
                    self._append_log(f"!!! GeoAnchor fix error: {e}")
        p = self._ga_proc
        if p is not None and p.poll() is not None:
            self._ga_proc = None
            self._ga.update(state="error", text=f"node stopped (exit code {p.returncode}) - see geoanchor_node.log")
            self._append_log(f"!!! GeoAnchor node exited with code {p.returncode} - see geoanchor_node.log")

    def _ga_handle_fix(self, m):
        ga = self._ga
        ga["n"] += 1
        ga["inl"], ga["mode"], ga["ms"], ga["last_rx"] = m.get("inl"), m.get("mode", ""), m.get("ms"), time.time()
        br, org = self._odo_bridge, self._gz_org
        if br is None or org is None:
            ga["last_res"] = "visual fix only (no Gazebo bridge)"
            try:
                self._ga_mark_fix(float(m["lat"]), float(m["lon"]), True, m.get("inl"), None)
            except Exception as e:
                self._append_log(f"!!! red-dot error: {e}")
            return
        if m.get("first") or ga["lock"] is None:
            k, h = float(m.get("scale", 1.0)), float(m.get("hdg", 90.0))
            if (k, h) != (self.gz_k, self.gz_hdg):
                self.gz_k, self.gz_hdg = k, h
                self._gz_clear_trail()
            ga["lock"] = (k, h)
            self._append_log(f">>> GeoAnchor LOCKED on the satellite map: world scale x{k:g}, heading {h:.0f}, "
                             f"{m.get('inl')} inliers")
        if m.get("mode") in self._GA_MODES_NO_FIX:
            ga["last_res"] = f"{m.get('mode')} - no new map match, not used as a fix"
            return
        x, y = self._gz_xy(org, float(m["lat"]), float(m["lon"]))
        inl = int(m.get("inl") or 0)
        gain = 0.6 + 0.3 * min(1.0, max(0.0, (inl - 40) / 120.0))      # more inliers -> trust the fix more
        res = br.inject_fix(x, y, t_cap=m.get("t"), gain=gain)
        try:
            self._ga_mark_fix(float(m["lat"]), float(m["lon"]), True, inl, res.get("err"))
        except Exception as e:
            self._append_log(f"!!! red-dot error: {e}")
        if res.get("ok"):
            ga["last_res"] = f"accepted, corrected the IMU estimate by {res['err']:.1f} m"
            if res.get("first"):
                self._append_log(f">>> First visual fix: the IMU estimate was {res['err']:.1f} m from the map position "
                                 f"(drift + home-location error) - corrected")
            elif res["err"] > 15.0 and time.time() - ga["log_t"] > 2.0:
                ga["log_t"] = time.time()
                self._append_log(f">>> Visual fix corrected a {res['err']:.1f} m drift of the IMU estimate")
        else:
            ga["last_res"] = f"rejected: {res.get('why')}"
            if time.time() - ga["log_t"] > 2.0:
                ga["log_t"] = time.time()
                self._append_log(f"!!! Visual fix rejected: {res.get('why')}")

    # ---- red dots: one per GeoAnchor fix, joined into the visually measured trajectory ----
    def _ga_dot_icon(self, color):
        cache = self.__dict__.setdefault("_ga_dot_icons", {})
        ic = cache.get(color)
        if ic is None:
            im = Image.new("RGBA", (14, 14), (0, 0, 0, 0))
            ImageDraw.Draw(im).ellipse((1, 1, 12, 12), fill=color, outline="#ffffff", width=1)
            ic = ImageTk.PhotoImage(im)
            cache[color] = ic
        return ic

    def _ga_clear_dots(self):
        for mk in self.__dict__.get("_ga_marks", []):
            try:
                mk.delete()
            except Exception:
                pass
        p = self.__dict__.get("_ga_path")
        if p is not None:
            try:
                p.delete()
            except Exception:
                pass
        self._ga_marks, self._ga_pts, self._ga_path = [], [], None

    def _ga_mark_fix(self, lat, lon, ok, inl, err):
        ga = self._ga
        ga["hist"].append((time.time(), lat, lon, ok, inl, err))
        del ga["hist"][:-8]
        if "_ga_marks" not in self.__dict__:
            self._ga_clear_dots()
        mk = self.map_widget.set_marker(lat, lon, icon=self._ga_dot_icon("#2a7fff" if (inl or 0) >= 40 else "#ff2020"),
                                        icon_anchor="center")
        self._ga_marks.append(mk)
        if len(self._ga_marks) > 600:                         # keep the map light
            try:
                self._ga_marks.pop(0).delete()
            except Exception:
                pass
        if ok:
            self._ga_pts.append((lat, lon))
            if len(self._ga_pts) >= 2:
                if self._ga_path is None:
                    self._ga_path = self.map_widget.set_path(list(self._ga_pts), color="#ff2020", width=2)
                else:
                    self._ga_path.set_position_list(list(self._ga_pts))

    def _ga_refresh_ui(self):
        ga = self._ga
        on = self._ga_proc is not None and self._ga_proc.poll() is None
        stc = {"off": COL_TEXT_DIM, "loading": self._NV_WARN, "waiting": self._NV_WARN, "searching": self._NV_WARN,
               "tracking": self._NV_OK, "error": self._NV_BAD}.get(ga["state"], COL_TEXT_DIM)
        self._ga_state_lbl.config(text=(ga["state"].upper() if ga["state"] != "off" else "OFF"), fg=stc)
        self._ga_badge.config(text="● " + ga["state"].upper() if ga["state"] != "off" else "", fg=stc)
        self._ga_node_lbl.config(text=ga["text"] if ga["state"] != "off" else
                                 "Off: the position comes from the IMU alone and drifts.", fg=stc)
        br = self._odo_bridge
        snap = br.snapshot() if br is not None else None
        vis = snap["vis"] if snap else None
        now = time.time()
        age = (now - vis["last_t"]) if vis and vis["last_t"] else None
        if on and age is not None and age < 6.0:
            color, src = self._NV_OK, "VISUAL FIX  ·  position anchored to the map"
        elif on and age is not None:
            color, src = self._NV_WARN, f"IMU HOLD  ·  {age:.0f} s since the last visual fix"
        elif on:
            color, src = self._NV_WARN, "IMU ONLY  ·  waiting for the first visual fix"
        else:
            color, src = self._NV_OFF, "IMU ONLY"
        self._ga_led.itemconfig(self._ga_led_id, fill=color)
        self._ga_src_lbl.config(text=src, fg=color)
        lines = []
        if on or ga["n"]:
            lines.append(f"fixes received {ga['n']}   accepted {vis['acc'] if vis else 0}   "
                         f"rejected {vis['rej'] if vis else 0}")
            if ga["mode"]:
                lines.append(f"matcher: {ga['mode']}   inliers {ga['inl']}   {ga['ms'] or 0:.0f} ms/fix")
            if ga["lock"]:
                lines.append(f"world scale x{ga['lock'][0]:g}   heading {ga['lock'][1]:.0f}°")
            if ga["last_res"]:
                lines.append(f"last fix: {ga['last_res']}")
            h = ga["hist"]
            if len(h) >= 2:
                lines.append(f"fix interval ~{(h[-1][0] - h[0][0]) / (len(h) - 1):.1f} s   (red dots on the map)")
            for ts, la, lo, ok_, il, er in reversed(h):
                lines.append(time.strftime("%H:%M:%S", time.localtime(ts)) + f"  {la:.6f} {lo:.6f}  "
                             + ("OK " if ok_ else "REJ") + f"  inl {il}  corr {er or 0.0:.1f} m")
            if vis and vis["bias"] != (0.0, 0.0):
                lines.append(f"learned accel bias  x {vis['bias'][0] * 1000:+.1f}  y {vis['bias'][1] * 1000:+.1f} mm/s²")
        self._ga_stat_lbl.config(text="\n".join(lines))

    # ---- card 5: Gazebo bridge settings ----
    def _build_odo_bridge_card(self):
        CARD = self._NAV_CARD
        body, self._odo_bridge_badge = self._odo_card(
            "GAZEBO BRIDGE",
            "Nothing to fill in: with auto-connect ON the dashboard finds the running "
            "simulation, the drone model, the pose / IMU topics and the camera by itself and connects "
            "(no VISUAL ODO switch needed).")
        class _Auto:                                    # no boxes any more: everything is auto-detected
            def get(self):
                return ""
        self._odo_cfg = {k: _Auto() for k in ("world", "model", "imu_topic", "pose_topic", "cam_topic")}
        arow = tk.Frame(body, bg=CARD)
        arow.pack(fill="x", pady=(10, 0))
        self._gz_auto_sw = ToggleSwitch(arow, value=True, command=self._gz_on_auto, bg=CARD, width=32, height=16)
        self._gz_auto_sw.pack(side="left")
        tk.Label(arow, text="Auto-detect and connect to a running Gazebo  (feeds Telemetry, gauges, map and camera)",
                 font=("Segoe UI", 9), fg=COL_TEXT_DIM, bg=CARD).pack(side="left", padx=10)
        drow = tk.Frame(body, bg=CARD)
        drow.pack(fill="x", pady=(8, 0))
        self._gz_dr_sw = ToggleSwitch(drow, value=True, command=self._gz_on_dr, bg=CARD, width=32, height=16)
        self._gz_dr_sw.pack(side="left")
        tk.Label(drow, text="Move the drone from the IMU (dead reckoning).  OFF = use Gazebo's own pose directly",
                 font=("Segoe UI", 9), fg=COL_TEXT_DIM, bg=CARD).pack(side="left", padx=10)
        brow = tk.Frame(body, bg=CARD)
        brow.pack(anchor="w", pady=(8, 0))
        self._odo_button(brow, "APPLY / RECONNECT", self._odo_apply_bridge, width=20).pack(side="left", ipady=2)
        self._odo_button(brow, "RE-ANCHOR IMU", self._gz_reanchor, width=16).pack(side="left", padx=(8, 0), ipady=2)
        self._odo_bridge_lbl = tk.Label(body, text="Bridge not started yet (it starts when GAZEBO is the active source).",
                                        font=("Segoe UI", 9), fg=COL_TEXT_DIM, bg=CARD, anchor="w",
                                        justify="left", wraplength=560)
        self._odo_bridge_lbl.pack(fill="x", pady=(8, 0))
        self._odo_wraps.append(self._odo_bridge_lbl)

    def _gz_on_dr(self, state):
        self.gz_dr = bool(state)
        self._gz_clear_trail()
        self._append_log(">>> Drone position now comes from " +
                         ("the IMU (dead reckoning)" if self.gz_dr else "Gazebo's ground-truth pose (NOT the IMU)"))

    def _gz_reanchor(self):
        if self._odo_bridge is not None:
            self._odo_bridge.reanchor()
            self._gz_clear_trail()
            self._append_log(">>> IMU estimate re-anchored on the Gazebo pose (drift reset to zero)")

    def _gz_clear_trail(self):
        self._gz_trail, self._gz_truth = [], []
        for attr in ("_gz_trail_path", "_gz_truth_path"):
            p = getattr(self, attr, None)
            if p is not None:
                try:
                    p.delete()
                except Exception:
                    pass
                setattr(self, attr, None)

    def _gz_on_auto(self, state):
        self.gz_auto = bool(state)
        self._append_log(">>> Gazebo auto-connect " + ("ON" if self.gz_auto else "OFF"))
        self._odo_tick(reschedule=False)

    def _odo_cfg_values(self):
        return [self._odo_cfg[k].get().strip() for k in ("world", "model", "imu_topic", "pose_topic")]

    def _odo_apply_bridge(self):
        vals = self._odo_cfg_values()
        if self._odo_bridge is not None:
            self._odo_bridge.configure(*vals)
        self._cam_apply()
        self._odo_started = True
        self._odo_tick(reschedule=False)

    # ------------------------------------------------------------------ data
    def _odo_primary(self):
        return self.swarm_order[0] if self.swarm_order else 1

    def _odo_feed(self, msg_type, data):
        """Called for every message the telemetry loop receives: keeps the TELEMETRY source up to date."""
        try:
            self._fl_rec_feed(msg_type, data)                # FLIGHT LOG: automatic telemetry recording
        except Exception:
            pass
        if msg_type not in ("IMU", "ATTITUDE", "POSITION", "VFR"):
            return
        if data.get("gz"):                                   # produced by the Gazebo feed, not by a flight controller
            return
        if data.get("sysid", 1) != self._odo_primary():
            return
        tl, now = self._odo_tele, time.time()
        if msg_type == "IMU":
            try:
                self.values["accel"].config(text="%+.2f  %+.2f  %+.2f" % tuple(data["accel"]))
                self.values["gyro"].config(text="%+.1f  %+.1f  %+.1f" % tuple(data["gyro"]))
            except Exception:
                pass
            tl["imu"] = {"t": now, "accel": data["accel"], "gyro": data["gyro"]}
            tl["n"] += 1
            if now - tl["t0"] >= 1.0:
                tl["hz"], tl["n"], tl["t0"] = tl["n"] / (now - tl["t0"]), 0, now
        elif msg_type == "ATTITUDE":
            tl["att"] = {"t": now, "roll": data["roll"], "pitch": data["pitch"], "yaw": data.get("yaw")}
        elif msg_type == "POSITION":
            tl["alt"] = {"t": now, "v": data["alt"]}      # relative altitude (barometer / EKF) - not GPS
        elif msg_type == "VFR":
            tl["hdg"] = {"t": now, "v": data["heading"]}
            tl["climb"] = {"t": now, "v": data["climb"]}

    def odo_sample(self):
        """Latest IMU / attitude / altitude of the ACTIVE source (what the GeoAnchor code reads).

        Keys: source ('gazebo' / 'telemetry'), t (time of the newest data), age (s), fresh (bool),
        accel [ax, ay, az] m/s2, gyro [gx, gy, gz] deg/s, roll / pitch / heading deg, alt m,
        climb m/s, rate_hz, gps_enabled, home (lat, lon) or None.  Missing values are None.
        """
        rs = self._rp_sample()                       # FLIGHT LOG replay running: its IMU / attitude are the live data
        if rs is not None:
            return rs
        src, now = self._odo_src(), time.time()
        out = {"source": src, "t": None, "age": None, "fresh": False, "accel": None, "gyro": None,
               "roll": None, "pitch": None, "heading": None, "alt": None, "climb": None,
               "rate_hz": 0.0, "gps_enabled": self.gps_enabled, "home": self.odo_home,
               "enabled": self.odo_enabled}
        times = []
        if src == "gazebo":
            snap = self._odo_bridge.snapshot() if self._odo_bridge is not None else None
            if snap:
                imu, pose = snap["imu"], snap["pose"]
                if imu:
                    out["accel"], out["gyro"] = list(imu["accel"]), list(imu["gyro"])
                    times.append(imu["t"])
                ps = snap["est"] if self.gz_dr else pose          # IMU mode: the dead-reckoned pose, never the truth
                if ps:
                    out.update(roll=ps["roll"], pitch=ps["pitch"], heading=ps["heading"],
                               alt=ps["z"], climb=ps["climb"])
                    times.append(ps["t"])
                out["rate_hz"] = snap["rate"]["imu"]
        else:
            tl = self._odo_tele
            if tl["imu"]:
                out["accel"], out["gyro"] = list(tl["imu"]["accel"]), list(tl["imu"]["gyro"])
                times.append(tl["imu"]["t"])
            if tl["att"]:
                out["roll"], out["pitch"] = tl["att"]["roll"], tl["att"]["pitch"]
                if tl["att"].get("yaw") is not None:
                    out["heading"] = tl["att"]["yaw"] % 360.0
                times.append(tl["att"]["t"])
            if tl["hdg"] and out["heading"] is None:
                out["heading"] = tl["hdg"]["v"]
            if tl["alt"]:
                out["alt"] = tl["alt"]["v"]
                times.append(tl["alt"]["t"])
            if tl["climb"]:
                out["climb"] = tl["climb"]["v"]
            out["rate_hz"] = tl["hz"]
        if times:
            out["t"] = max(times)
            out["age"] = now - out["t"]
            out["fresh"] = out["age"] < self._ODO_FRESH_S
        return out

    # ------------------------------------------------------------------ refresh loop
    def _odo_tick(self, reschedule=True):
        try:
            self._odo_manage_bridge()
            self._odo_refresh_ui()
        except tk.TclError:
            return
        except Exception as e:                                   # never let the loop die
            try:
                self._append_log(f"!!! VISUAL ODO refresh error: {e}")
            except Exception:
                pass
        if reschedule:
            self.root.after(200, self._odo_tick)

    # ------------------------------------------------------------------ Gazebo auto-connect
    def _gz_real_link_up(self):
        return bool(getattr(self.mav, "connected", False)) and getattr(self.mav, "mode", None) == "REAL"

    def _gz_mav_fresh(self):
        """True while a MAVLink vehicle (SITL ArduPilot or a real one) is sending data - MAVLink always wins."""
        now = time.time()
        return any(now - t < 3.0 for t in list(getattr(self.mav, "link_last_seen", {}).values()))

    def _gz_wanted(self):
        """Should the Gazebo bridge + camera be running right now?"""
        if self.odo_enabled and self._odo_src() == "gazebo":
            return True
        w = self._gz_watch
        return bool(self.gz_auto and w is not None and w.present and not self._gz_real_link_up())

    def _odo_manage_bridge(self):
        if self._odo_bridge is not None:
            self._odo_bridge.baro_aid = self.gz_baro
        want = self._gz_wanted()
        if want and self._odo_bridge is None:
            self._odo_bridge = GazeboBridge(*self._odo_cfg_values())
            self._odo_bridge.start()
        elif not want and self._odo_bridge is not None:
            self._odo_bridge.stop()
            self._odo_bridge = None

    def _odo_refresh_ui(self):
        src = self._odo_src()
        names = {"gazebo": "GAZEBO", "telemetry": "TELEMETRY"}
        for key, lb in self._odo_seg.items():
            on = key == src
            lb.config(bg=self._NV_ACC if on else COL_BTN, fg=self._NV_ON_ACC if on else COL_TEXT_DIM)
        s = self.odo_sample()
        # ---- indicator ----
        if not self.odo_enabled and self._odo_bridge is None:
            color = self._NV_OFF
            text = ("Auto-connect is ON  \u00b7  waiting for a Gazebo simulation ..." if self.gz_auto else
                    "VISUAL ODO is OFF  \u00b7  switch it on (top of this panel) to connect")
        elif src == "gazebo":
            snap = self._odo_bridge.snapshot() if self._odo_bridge is not None else None
            if snap is None or snap["state"] in ("starting", "waiting"):
                color, text = self._NV_WARN, "GAZEBO  \u00b7  " + (snap["detail"] if snap else "starting ...")
            elif snap["state"] == "error":
                color, text = self._NV_BAD, "GAZEBO  \u00b7  " + snap["detail"]
            elif s["fresh"]:
                color, text = self._NV_OK, f"GAZEBO  \u00b7  live  \u00b7  IMU {s['rate_hz']:.0f} Hz" + \
                                           ("  (derived from pose)" if snap["imu"] and snap["imu"].get("derived") else "")
            else:
                color, text = self._NV_WARN, "GAZEBO  \u00b7  bridge connected, no data coming in (is the sim running / unpaused?)"
        else:
            if s["fresh"]:
                color, text = self._NV_OK, "TELEMETRY  \u00b7  live from the flight controller" + \
                                           (f"  \u00b7  IMU {s['rate_hz']:.0f} Hz" if s["rate_hz"] >= 0.5 else "")
            elif self.mav.connected:
                color, text = self._NV_WARN, "TELEMETRY  \u00b7  link is up but no attitude / IMU data yet"
            else:
                color, text = self._NV_BAD, "TELEMETRY  \u00b7  not connected"
        if self._rp["state"] == "running":
            color, text = self._NV_OK, "FLIGHT REPLAY  \u00b7  recorded IMU / attitude / altitude (not the live source)"
        self._odo_led.itemconfig(self._odo_led_id, fill=color)
        self._odo_ind_lbl.config(text=text, fg=color)
        self._odo_src_badge.config(text="AUTO" if self._odo_follow else "MANUAL", fg=COL_TEXT_DIM)
        self._odo_head_src.config(text=f"\u25cf {names[src]}", fg=color)
        self._odo_data_badge.config(text=names[src], fg=color)
        # ---- values ----
        def put(key, val, fmt):
            lab = self._odo_vals[key]
            lab.config(text="--" if val is None else fmt % val, fg=self._NV_VAL if s["fresh"] else COL_TEXT_DIM)
        a, g = s["accel"] or (None, None, None), s["gyro"] or (None, None, None)
        for key, v in zip(("ax", "ay", "az"), a):
            put(key, v, "%+8.3f")
        for key, v in zip(("gx", "gy", "gz"), g):
            put(key, v, "%+8.2f")
        put("roll", s["roll"], "%+8.2f")
        put("pitch", s["pitch"], "%+8.2f")
        put("yaw", s["heading"], "%8.1f")
        put("alt", s["alt"], "%8.2f")
        put("climb", s["climb"], "%+8.2f")
        self._ga_refresh_ui()
        self._odo_vals["age"].config(text="--" if s["age"] is None else f"{s['age'] * 1000:6.0f} ms",
                                     fg=self._NV_VAL if s["fresh"] else COL_TEXT_DIM)
        # ---- bridge card ----
        if self._odo_bridge is not None:
            snap = self._odo_bridge.snapshot()
            col = {"running": self._NV_OK, "error": self._NV_BAD}.get(snap["state"], self._NV_WARN)
            lines = [f"\u25cf {snap['state'].upper()}  -  {snap['detail']}"]
            if snap["model"]:
                lines.append(f"Model: {snap['model']}   (read through: {snap['mode'] or '...'})")
            if self.gz_dr:
                e = snap.get("est")
                lines.append("MOTION from: IMU dead reckoning  (truth pose = start anchor + drift readout only)" +
                             (f"   drift now {e['err_h']:.2f} m" if e and e.get("err_h") is not None else ""))
            else:
                lines.append("MOTION from: Gazebo ground-truth pose  (IMU not used for motion)")
            if not snap["imu_real"] and snap["state"] == "running":
                lines.append("!! this world has no IMU sensor: the IMU is derived from the truth pose, "
                             "so the 'IMU' path is NOT independent")
            if snap["topics"]["imu"]:
                lines.append(f"IMU:  {snap['topics']['imu']}   ({snap['rate']['imu']:.0f} Hz)")
            elif snap["state"] == "running":
                lines.append(f"IMU:  no IMU sensor in this world -> derived from the pose   ({snap['rate']['imu']:.0f} Hz)")
            if snap["topics"]["pose"]:
                lines.append(f"Pose: {snap['topics']['pose']}   ({snap['rate']['pose']:.0f} Hz)")
            cam = self._cam.snapshot() if self._cam is not None else None
            if cam and cam["topic"]:
                lines.append(f"Camera: {cam['topic']}   ({cam['fps']:.0f} fps, {cam['mode']})")
            elif cam:
                lines.append(f"Camera: {cam['detail']}")
            self._odo_bridge_lbl.config(text="\n".join(lines), fg=col)
            self._odo_bridge_badge.config(text=snap["state"].upper(), fg=col)
        else:
            self._odo_bridge_lbl.config(
                text="Bridge is stopped (it only runs while VISUAL ODO is ON and GAZEBO is the active source).", fg=COL_TEXT_DIM)
            self._odo_bridge_badge.config(text="STOPPED", fg=COL_TEXT_DIM)

    # ------------------------------------------------------------------ Gazebo -> GCS telemetry feed
    def _gz_tick(self):
        try:
            self._gz_pump()
        except tk.TclError:
            return
        except Exception as e:                                   # never let the loop die
            if not self._gz_err_logged:
                self._gz_err_logged = True
                try:
                    self._append_log(f"!!! Gazebo feed error: {e}")
                except Exception:
                    pass
        self.root.after(100, self._gz_tick)

    def _gz_origin(self, pose):
        """Local Gazebo (x east, y north) -> lat / lon.  The drone starts at the HOME location (or, if no home
        is set, at the centre of the map) and moves from there."""
        home = self.odo_home
        org = self._gz_org
        if org is None or org["home"] != home or (time.time() - org["seen"] > 5.0 and self._ga["lock"] is None):
            if home:
                lat0, lon0 = home
            else:
                try:
                    lat0, lon0 = self.map_widget.get_position()
                except Exception:
                    lat0, lon0 = self.last_lat, self.last_lon
            org = self._gz_org = {"x0": pose["x"], "y0": pose["y"], "lat0": lat0, "lon0": lon0,
                                  "home": home, "seen": time.time()}
            self._gz_clear_trail()
        org["seen"] = time.time()
        return org

    def _gz_ll(self, org, x, y):
        """Local Gazebo (x, y) -> lat / lon.  Gazebo +x points to compass bearing gz_hdg (90 = east), +y is 90 deg
        to its left, and 1 Gazebo metre = gz_k real metres (both found by GeoAnchor when it locks on)."""
        h = math.radians(self.gz_hdg)
        dx, dy = x - org["x0"], y - org["y0"]
        dn = self.gz_k * (dx * math.cos(h) + dy * math.sin(h))
        de = self.gz_k * (dx * math.sin(h) - dy * math.cos(h))
        lat = org["lat0"] + dn / 111320.0
        lon = org["lon0"] + de / (111320.0 * max(0.05, math.cos(math.radians(org["lat0"]))))
        return lat, lon

    def _gz_xy(self, org, lat, lon):
        """Inverse of _gz_ll: lat / lon -> local Gazebo (x, y)."""
        h = math.radians(self.gz_hdg)
        dn = (lat - org["lat0"]) * 111320.0
        de = (lon - org["lon0"]) * 111320.0 * max(0.05, math.cos(math.radians(org["lat0"])))
        k = max(1e-6, self.gz_k)
        return (org["x0"] + (dn * math.cos(h) + de * math.sin(h)) / k,
                org["y0"] + (dn * math.sin(h) - de * math.cos(h)) / k)

    def _gz_trail_add(self, trail, ll, min_d=0.5):
        if not trail or math.hypot(*self._ll_to_ne(trail[-1], ll[0], ll[1])) > min_d:
            trail.append(ll)
            if len(trail) > 4000:
                del trail[::2]

    def _gz_draw_trails(self):
        for attr, pts, color, width, dotted in (
                ("_gz_truth_path", self._gz_truth, "#6fa8ff", 2, True),
                ("_gz_trail_path", self._gz_trail, COL_TRAJECTORY, 3, False)):
            if len(pts) < 2:
                continue
            path = getattr(self, attr)
            if path is None or getattr(path, "deleted", False):
                path = self.map_widget.set_path(list(pts), color=color, width=width)
                if dotted:
                    try:
                        self.map_widget.canvas.itemconfig(path.canvas_line, dash=(2, 6))
                    except Exception:
                        pass
                setattr(self, attr, path)
            else:
                path.set_position_list(list(pts))

    def _gz_pump(self):
        self._ga_poll()
        br = self._odo_bridge
        snap = br.snapshot() if br is not None else None
        pose = snap["pose"] if snap else None
        now = time.time()
        live = bool(pose) and now - pose["t"] < 1.5 and not self._gz_mav_fresh() and not self._gz_real_link_up()
        self._gz_live = live

        # ---- SIM indicator: green while Gazebo is connected ----
        if live:
            self._update_source_indicators("SIM", True)
            self._gz_was_live = True
            if not self._gz_logged_connect:
                self._gz_logged_connect = True
                self._append_log(f">>> Gazebo connected  (model '{snap['model'] or '?'}', {snap['mode'] or '...'}) "
                                 f"- telemetry, gauges, map and camera are fed from the simulator")
        elif self._gz_was_live:
            self._gz_was_live = False
            self._gz_logged_connect = False
            if not getattr(self.mav, "connected", False):
                self._update_source_indicators("SIM", False)      # orange: Gazebo was connected, data stopped
            self._gz_org = None
            try:
                self.values["link"].config(text="GAZEBO LOST", fg=COL_WARN)
            except Exception:
                pass
        if not live:
            return

        v = self.values
        rate = snap["rate"]["pose"]
        est = snap.get("est")
        if self.gz_dr:
            cur = est if (est and now - est["t"] < 1.5) else None     # IMU mode: NEVER fall back to the truth pose
        else:
            cur = pose
        v["link"].config(text=f"GAZEBO LIVE  \u00b7  {rate:.0f} Hz", fg=COL_DOT_ON)
        if cur is None:
            v["mode"].config(text="GAZEBO  \u00b7  waiting for IMU data", fg=COL_WARN)
            return

        org = self._gz_origin(cur)
        lat, lon = self._gz_ll(org, cur["x"], cur["y"])
        hdg = int(round(cur["heading"])) % 360
        yaw = hdg if hdg <= 180 else hdg - 360
        sysid = self._odo_primary()
        q = self.data_queue
        # same messages a MAVLink vehicle would send -> map marker, gauges, compass and telemetry fields update
        q.put(("POSITION", {"lat": lat, "lon": lon, "alt": cur["z"], "sysid": sysid, "gz": True}))
        q.put(("VFR", {"airspeed": cur["gspeed"], "groundspeed": cur["gspeed"], "heading": hdg,
                       "climb": cur["climb"], "sysid": sysid, "gz": True}))
        q.put(("ATTITUDE", {"roll": cur["roll"], "pitch": cur["pitch"], "yaw": yaw, "sysid": sysid, "gz": True}))

        if self.gz_dr and not self.gps_enabled:
            # GPS is blocked on purpose (GPS-denied test): the IMU estimate is NOT a GPS value, so it still moves the drone
            self.last_lat, self.last_lon = lat, lon
            try:
                if self.drone_marker:
                    self.drone_marker.set_position(lat, lon)
            except Exception:
                pass
            v["lat"].config(text=f"{lat:.6f}  (IMU est.)", fg=COL_VALUE)
            v["lon"].config(text=f"{lon:.6f}  (IMU est.)", fg=COL_VALUE)

        # ---- yellow trail = where the drone thinks it went; light-blue dotted = where Gazebo says it went ----
        if self.gps_enabled or self.gz_dr:
            self._gz_trail_add(self._gz_trail, (lat, lon))
            if self.gz_dr:
                self._gz_trail_add(self._gz_truth, self._gz_ll(org, pose["x"], pose["y"]))
            self._gz_trail_n += 1
            if self._gz_trail_n % 5 == 0:
                self._gz_draw_trails()

        # fields that have no MAVLink message behind them
        v["mode"].config(text="GAZEBO  \u00b7  IMU dead-reckoning" if self.gz_dr else "GAZEBO  \u00b7  ground truth",
                         fg=COL_VALUE)
        v["armed"].config(text="n/a (simulator)", fg=COL_VALUE)
        v["prearm"].config(text="n/a (simulator)", fg=COL_VALUE)
        v["ekf"].config(text="n/a (IMU only)" if self.gz_dr else "OK  (Gazebo ground truth)", fg=COL_VALUE)
        v["voltage"].config(text="n/a (simulator)", fg=COL_VALUE)
        v["remaining"].config(text="n/a (simulator)", fg=COL_VALUE)
        v["localpos"].config(text=f"{cur['x'] - org['x0']:+.2f}  /  {cur['y'] - org['y0']:+.2f}")
        if self.gz_dr and est and est.get("err_h") is not None:
            col = COL_VALUE if est["err_h"] < 3.0 else (COL_WARN if est["err_h"] < 15.0 else COL_ARMED)
            v["dr_err"].config(text=f"{est['err_h']:.2f} horiz  /  {est['err_v']:.2f} vert", fg=col)
        else:
            v["dr_err"].config(text="-- (IMU mode off)", fg=COL_TEXT_DIM)
        imu = snap["imu"]
        if imu:
            v["accel"].config(text="%+.2f  %+.2f  %+.2f" % tuple(imu["accel"]))
            v["gyro"].config(text="%+.1f  %+.1f  %+.1f" % tuple(imu["gyro"]))
        if self.gps_enabled:
            self.gps_overlay_label.config(text="POS: IMU est." if self.gz_dr else "GPS: SIM (3D)", fg=COL_VALUE)

    # ------------------------------------------------------------------ slide (same as SWARM NAV)
    def _toggle_odo_panel(self):
        self._odo_slide(0 if self._odo_target else 1)

    def _odo_slide(self, target):
        self._odo_target = target
        if target:
            if getattr(self, "_nav_target", 0):
                self._nav_slide(0)                       # SWARM NAV and VISUAL ODO share the same spot
            self._odo_started = True
            self.odo_panel.show()
            self._odo_tick(reschedule=False)
        if self._odo_job is None:
            self._odo_job = self.root.after(0, self._odo_step)

    def _odo_step(self):
        self._odo_job = None
        tgt, cur = self._odo_target, self._odo_prog
        delta = tgt - cur
        if abs(delta) < 0.012:
            prog = float(tgt)
        else:
            step = delta * 0.25
            if abs(step) < 0.012:
                step = 0.012 if delta > 0 else -0.012
            prog = cur + step
        self._odo_prog = prog
        self.odo_panel.apply()
        if tgt:
            self.odo_panel.lift()
        if prog != tgt:
            self._odo_job = self.root.after(16, self._odo_step)
        elif tgt == 0:
            self.odo_panel.hide()

    def odo_shutdown(self):
        self._fl_shutdown()
        self._ga_stop()
        if self._gz_watch is not None:
            self._gz_watch.stop()
            self._gz_watch = None
        if self._odo_bridge is not None:
            self._odo_bridge.stop()
            self._odo_bridge = None
        if self._cam is not None:
            self._cam.stop()
            self._cam = None

    # ======================================================================================================
    # FLIGHT LOG  -  automatic telemetry recording + synchronized video / telemetry replay
    #
    # This is the last card of the VISUAL ODO panel.  Everything above is unchanged except four one-line hooks:
    #   _odo_init_state -> _fl_init_state        _build_odo_panel -> _build_odo_flight_card
    #   _odo_feed       -> _fl_rec_feed          odo_shutdown      -> _fl_shutdown
    #
    # RECORD  every telemetry message of the telemetry loop (IMU, attitude, GPS position, speed / heading,
    #         battery, GPS status, EKF, heartbeat / mode / armed, status texts, radio) is time-stamped from the
    #         moment of takeoff to landing and saved as   flight_logs/flight_YYYYmmdd_HHMMSS.json
    # REPLAY  the log and a (pre-trimmed) video are played from ONE clock: every 30 ms the clock position t is
    #         used for  (a) the telemetry that is shown / sent on,  (b) the video frame (the GeoAnchor node
    #         decodes frame t + offset and sends it back for the preview).  GeoAnchor matches the frames to the
    #         satellite map using the replayed heading / altitude / attitude and every fix is drawn as a dot.
    # ======================================================================================================
    _FL_FORMAT = "visual-nav-flightlog"
    _FL_TYPES = ("IMU", "ATTITUDE", "POSITION", "VFR", "BATTERY", "GPS_RAW", "EKF_STATUS", "HEARTBEAT",
                 "STATUSTEXT", "RADIO_STATUS", "MISSION_CURRENT")
    _FL_TAKEOFF_ALT = 0.5        # m above the launch point (and armed, when the heartbeat says so) = off the ground
    _FL_GROUND_ALT = 0.3         # m: at or below this the drone counts as standing on the ground
    _FL_LAND_ALT = 0.5           # m: landed = at or below this for _FL_LAND_HOLD_S seconds without climbing
    _FL_LAND_HOLD_S = 3.0
    _FL_MIN_FLIGHT_S = 8.0       # a landing is only accepted after at least this much flight
    _FL_PRE_S = 12.0             # recent telemetry kept before takeoff, so t = 0 is the moment of lift-off
    _FL_DIR = os.path.join(PROJECT_ROOT, "flight_logs")
    _RP_FIX_PORT = 15902         # node -> GCS (separate from the live GeoAnchor ports 15900 / 15901)
    _RP_STATE_PORT = 15903       # GCS -> node (replayed attitude / altitude + video clock)

    # ------------------------------------------------------------------ state
    def _fl_init_state(self):
        import collections
        self._fl_auto = True                       # automatic recording ON by default
        self._fl_gps_rec = True                    # GPS accepted into the log (Record GPS switch)
        self._fl = {"cache": {}, "pre": collections.deque(), "rec": None, "last": None,
                    "alt": None, "climb": None, "armed": None, "imu_feed_t": 0.0, "ground_t": time.time(),
                    "note": None, "err_logged": False}
        self._rp = {"state": "idle", "proc": None, "sock": None, "logf": None, "t0": None, "t_end": 0.0,
                    "idx": 0, "st": {}, "frame": None, "node": ("", ""), "fixes": 0, "off": 0.0,
                    "sent": 0.0, "ui": 0.0, "vid_dur": None, "err_logged": False, "t_prep": 0.0, "cam": None, "cam_n": 0}
        self._rp_log = None                        # the flight log that is loaded for the replay
        self._rp_ref_path = None                   # light-blue dotted line: the GPS track that was recorded

    # ------------------------------------------------------------------ recorder
    def _fl_rec_feed(self, msg_type, data):
        """Every message the telemetry loop receives passes here (called from _odo_feed)."""
        fl = self._fl
        if msg_type == "LINK_STATUS":
            if isinstance(data, dict) and not data.get("connected") and fl["rec"] is not None:
                self._fl_finish("telemetry link lost")       # keep what was flown so far
            return
        if msg_type not in self._FL_TYPES or not isinstance(data, dict):
            return
        sid = data.get("sysid")
        if sid is not None and sid != self._odo_primary():    # one drone: the primary one (same as the IMU feed)
            return
        now = time.time()
        data = dict(data)
        if not self._fl_gps_rec:                              # Record GPS = OFF: GPS never reaches the log
            if msg_type == "GPS_RAW":
                return
            if msg_type == "POSITION":
                data["lat"] = data["lon"] = None              # altitude stays (takeoff / landing + GeoAnchor need it)
        ev = (now, msg_type, data)
        fl["cache"][msg_type] = ev
        if msg_type == "POSITION":
            fl["alt"] = (now, float(data.get("alt") or 0.0))
        elif msg_type == "VFR":
            fl["climb"] = (now, float(data.get("climb") or 0.0))
        elif msg_type == "HEARTBEAT":
            fl["armed"] = (now, bool(data.get("armed")))
        elif msg_type == "IMU":
            fl["imu_feed_t"] = now
        if self._fl_auto:
            self._fl_push(ev)

    def _fl_push(self, ev):
        fl = self._fl
        if fl["rec"] is not None:
            fl["rec"]["events"].append(ev)
        else:                                      # before takeoff: only the last few seconds are kept
            pre = fl["pre"]
            pre.append(ev)
            cut = ev[0] - self._FL_PRE_S
            while pre and pre[0][0] < cut:
                pre.popleft()

    def _fl_on_auto(self, state):
        self._fl_auto = bool(state)
        self._fl["pre"].clear()
        if not self._fl_auto and self._fl["rec"] is not None:
            self._fl_finish("automatic recording switched off")
        self._append_log(">>> Flight log: automatic recording " + ("ON" if self._fl_auto else "OFF"))
        self._fl_refresh_ui()

    def _fl_on_gps(self, state):
        """Record GPS switch: ON = GPS position + satellites go into the log, OFF = they are filtered out."""
        self._fl_gps_rec = bool(state)
        self._fl["pre"].clear()
        self._fl["cache"].pop("GPS_RAW", None)
        if self._fl["rec"] is not None and not self._fl_gps_rec:
            self._fl["rec"]["gps_all"] = False               # this flight has GPS only for part of it
        self._append_log(">>> Flight log: GPS " + ("is recorded" if self._fl_gps_rec else "is filtered out of the log"))
        self._fl_refresh_ui()

    def _fl_tick(self):
        try:
            self._fl_rec_step()
            self._fl_refresh_ui()
        except tk.TclError:
            return
        except Exception as e:                                   # never let the loop die
            if not self._fl["err_logged"]:
                self._fl["err_logged"] = True
                try:
                    self._append_log(f"!!! Flight log recorder error: {e}")
                except Exception:
                    pass
        self.root.after(100, self._fl_tick)

    def _fl_rec_step(self):
        fl, now = self._fl, time.time()
        if not self._fl_auto:
            return
        # Gazebo has no IMU message: while the source is the simulator the IMU is sampled here (10 Hz)
        if self._odo_src() == "gazebo" and now - fl["imu_feed_t"] > 1.0 and self._rp["state"] != "running":
            s = self.odo_sample()
            if s["accel"] and s["gyro"] and s["fresh"]:
                ev = (now, "IMU", {"accel": list(s["accel"]), "gyro": list(s["gyro"]), "gz": True,
                                   "sysid": self._odo_primary()})
                fl["cache"]["IMU"] = ev
                self._fl_push(ev)
        alt = fl["alt"][1] if fl["alt"] and now - fl["alt"][0] < 2.5 else None
        climb = fl["climb"][1] if fl["climb"] and now - fl["climb"][0] < 2.5 else 0.0
        armed_known = bool(fl["armed"]) and now - fl["armed"][0] < 3.5
        armed = armed_known and fl["armed"][1]
        rec = fl["rec"]
        if rec is None:
            if alt is None:
                return
            if alt <= self._FL_GROUND_ALT:
                fl["ground_t"] = now                   # last moment on the ground = lift-off estimate
            if alt >= self._FL_TAKEOFF_ALT and (armed or not armed_known):
                self._fl_begin(now)
            return
        rec["max_alt"] = max(rec["max_alt"], alt or 0.0)
        dur = now - rec["t0"]
        if alt is not None and alt <= self._FL_LAND_ALT and abs(climb) < 0.6:
            if rec["ground_since"] is None:
                rec["ground_since"] = now
            elif now - rec["ground_since"] >= self._FL_LAND_HOLD_S and dur >= self._FL_MIN_FLIGHT_S:
                self._fl_finish("landed", t_end=rec["ground_since"] + 0.5)
                return
        else:
            rec["ground_since"] = None
        if armed_known and not armed and (alt is None or alt <= 3.0):
            self._fl_finish("disarmed")

    def _fl_begin(self, now):
        fl = self._fl
        t0 = max(fl["ground_t"], now - self._FL_PRE_S)
        rec = {"t0": t0, "events": [], "snap": [], "max_alt": 0.0, "ground_since": None, "gps_all": self._fl_gps_rec}
        for ty, ev in fl["cache"].items():
            if ev[0] < t0:
                rec["snap"].append((t0, ty, ev[2]))     # last known value from before lift-off -> t = 0
        for ev in fl["pre"]:
            if ev[0] >= t0:
                rec["events"].append(ev)
        fl["pre"].clear()
        fl["rec"] = rec
        fl["note"] = None
        self._append_log(">>> Flight log: takeoff detected - recording the telemetry")

    def _fl_finish(self, why, t_end=None):
        fl = self._fl
        rec, fl["rec"] = fl["rec"], None
        if rec is None:
            return
        t0 = rec["t0"]
        end = t_end if t_end else time.time()
        evs = [[0.0, ty, d] for (_w, ty, d) in rec["snap"]]
        for w, ty, d in rec["events"]:
            if w <= end:
                evs.append([round(max(0.0, w - t0), 3), ty, d])
        dur = max(0.0, end - t0)
        if dur < 3.0 or len(evs) < 20:
            fl["note"] = (f"Recording discarded: too short ({dur:.0f} s, {len(evs)} events)", self._NV_WARN)
            self._append_log(f"!!! Flight log: recording discarded - too short ({dur:.0f} s)")
            return
        counts = {}
        for e in evs:
            counts[e[1]] = counts.get(e[1], 0) + 1
        doc = {"format": self._FL_FORMAT, "version": 1,
               "created": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t0)), "takeoff_epoch": t0,
               "duration_s": round(dur, 3), "end_reason": why, "source": self._odo_src(),
               "sysid": self._odo_primary(), "gps_enabled": bool(self.gps_enabled),
               "gps_recorded": bool(self._fl_gps_rec and rec["gps_all"]),
               "home": list(self.odo_home) if self.odo_home else None, "max_alt_m": round(rec["max_alt"], 2),
               "counts": counts, "events": evs}
        name = time.strftime("flight_%Y%m%d_%H%M%S", time.localtime(t0)) + ".json"
        path = os.path.join(self._FL_DIR, name)
        try:
            os.makedirs(self._FL_DIR, exist_ok=True)
            with open(path, "w") as f:
                json.dump(doc, f, separators=(",", ":"))
        except Exception as e:
            fl["note"] = (f"Could not save the flight log: {e}", self._NV_BAD)
            self._append_log(f"!!! Flight log: could not save ({e})")
            return
        fl["last"] = {"path": path, "name": name, "dur": dur, "n": len(evs)}
        fl["note"] = (f"Saved {name}  -  {dur:.0f} s, {len(evs)} events  ({why})", self._NV_OK)
        self._append_log(f">>> Flight log saved: {path}  ({dur:.0f} s, {len(evs)} events, {why})")

    def _fl_refresh_ui(self):
        fl, rec = self._fl, self._fl["rec"]
        if not self._fl_auto:
            st, col = "OFF", COL_TEXT_DIM
        elif rec is not None:
            st, col = "REC  %02d:%02d" % divmod(int(time.time() - rec["t0"]), 60), self._NV_BAD
        else:
            st, col = "WAITING FOR TAKEOFF", self._NV_WARN
        self._fl_state_lbl.config(text=st, fg=col)
        if self._fl_gps_rec:
            self._fl_gps_lbl.config(text="ON  -  GPS position and satellites are recorded", fg=self._NV_OK)
        else:
            self._fl_gps_lbl.config(text="OFF  -  GPS is filtered out of the log", fg=self._NV_BAD)
        self._fl_badge.config(text=("\u25cf REC" if rec is not None else ""), fg=self._NV_BAD)
        lines = []
        if rec is not None:
            n = len(rec["events"]) + len(rec["snap"])
            lines.append(f"RECORDING   {n} events   max alt {rec['max_alt']:.1f} m   (ends at landing)")
        elif self._fl_auto:
            lines.append("Starts by itself the moment the drone lifts off the ground and stops at landing.")
        else:
            lines.append("Automatic recording is OFF.")
        self._fl_stat_lbl.config(text="\n".join(lines), fg=col if rec is not None else COL_TEXT_DIM)
        note = fl["note"]
        self._fl_note_lbl.config(text=note[0] if note else "", fg=note[1] if note else COL_TEXT_DIM)

    def _fl_download(self):
        last = self._fl["last"]
        if not last or not os.path.isfile(last["path"]):
            self._fl["note"] = ("Nothing recorded yet - fly first; the log is saved automatically at landing.",
                                self._NV_WARN)
            return
        from tkinter import filedialog
        import shutil
        dest = filedialog.asksaveasfilename(parent=self.root, title="Download flight log", initialfile=last["name"],
                                            defaultextension=".json",
                                            filetypes=[("Flight log", "*.json"), ("All files", "*.*")])
        if not dest:
            return
        try:
            shutil.copyfile(last["path"], dest)
            self._fl["note"] = (f"Downloaded to {dest}", self._NV_OK)
            self._append_log(f">>> Flight log downloaded to {dest}")
        except Exception as e:
            self._fl["note"] = (f"Download failed: {e}", self._NV_BAD)

    def _fl_use(self):
        """USE: the newest recording goes straight into the flight-log input of the replay section."""
        last = self._fl["last"]
        if not last or not os.path.isfile(last["path"]):
            self._fl["note"] = ("Nothing recorded yet - fly first; the log is saved automatically at landing.",
                                self._NV_WARN)
            return
        self._rp_log_e.delete(0, "end")
        self._rp_log_e.insert(0, last["path"])
        if self._fl_load_log(last["path"]):
            self._fl["note"] = (f"{last['name']} is loaded in the replay section below - choose the video and press START.",
                                self._NV_OK)

    def _fl_shutdown(self):
        try:
            if self._fl["rec"] is not None:
                self._fl_finish("GCS closed")
        except Exception:
            pass
        try:
            if self._rp["state"] != "idle":
                self._rp_stop("GCS closed")
        except Exception:
            pass

    # ------------------------------------------------------------------ replay: files
    def _rp_set_status(self, text, color=None):
        self._rp_status.config(text=text, fg=color or COL_TEXT_DIM)

    def _rp_browse_log(self):
        from tkinter import filedialog
        p = filedialog.askopenfilename(parent=self.root, title="Flight log",
                                       initialdir=self._FL_DIR if os.path.isdir(self._FL_DIR) else PROJECT_ROOT,
                                       filetypes=[("Flight log", "*.json"), ("All files", "*.*")])
        if p:
            self._rp_log_e.delete(0, "end")
            self._rp_log_e.insert(0, p)
            self._fl_load_log(p)

    def _rp_browse_video(self):
        from tkinter import filedialog
        p = filedialog.askopenfilename(parent=self.root, title="Flight video (trimmed: takeoff to landing)",
                                       filetypes=[("Video", "*.mp4 *.mov *.avi *.mkv *.m4v *.webm *.mpg *.mpeg"),
                                                  ("All files", "*.*")])
        if p:
            self._rp_vid_e.delete(0, "end")
            self._rp_vid_e.insert(0, p)
            self._rp_set_status("Video selected.", COL_TEXT_DIM)

    def _fl_load_log(self, path):
        """Read a flight-log file into self._rp_log.  True on success."""
        try:
            path = os.path.abspath(os.path.expanduser(path))
            with open(path, "r") as f:
                d = json.load(f)
            if not (isinstance(d, dict) and d.get("format") == self._FL_FORMAT and isinstance(d.get("events"), list)):
                raise ValueError("this is not a Visual Nav flight log")
            ev = [e for e in d["events"] if isinstance(e, list) and len(e) == 3 and isinstance(e[2], dict)]
            if not ev:
                raise ValueError("the log has no telemetry events")
            ev.sort(key=lambda e: e[0])
            dur = float(d.get("duration_s") or ev[-1][0])
            gps = [(e[2]["lat"], e[2]["lon"]) for e in ev
                   if e[1] == "POSITION" and not (e[2].get("lat") == 0.0 and e[2].get("lon") == 0.0)
                   and e[2].get("lat") is not None]
            alts = sorted(e[2]["alt"] for e in ev if e[1] == "POSITION" and e[2].get("alt") is not None)
            air = [a for a in alts if a > 2.0]
            cruise = air[len(air) // 2] if air else (max(alts) if alts else 50.0)
            types = {e[1] for e in ev}
            imu_hz = sum(1 for e in ev if e[1] == "IMU") / max(dur, 1.0)
        except Exception as e:
            self._rp_log = None
            self._rp_set_status(f"Flight log not loaded: {e}", self._NV_BAD)
            self._rp_log_info.config(text="", fg=COL_TEXT_DIM)
            return False
        self._rp_log = {"path": path, "ev": ev, "dur": dur, "gps": gps, "cruise": max(1.0, cruise),
                        "types": types, "meta": d, "imu_hz": imu_hz}
        self._rp_log_info.config(
            text=f"{os.path.basename(path)}  \u00b7  {dur:.0f} s  \u00b7  {len(ev)} events  \u00b7  "
                 f"GPS {'yes' if gps else 'no'}  \u00b7  IMU {'yes' if 'IMU' in types else 'no'}  \u00b7  "
                 f"cruise alt {max(1.0, cruise):.0f} m", fg=self._NV_VAL)
        self._rp_set_status("Flight log loaded.", COL_TEXT_DIM)
        return True

    # ------------------------------------------------------------------ replay: start / stop
    def _rp_start(self):
        rp = self._rp
        if rp["state"] != "idle":
            return
        bad = self._NV_BAD
        lp = self._rp_log_e.get().strip()
        if not lp:
            self._rp_set_status("Choose a flight-log file first (or press USE after a flight).", bad)
            return
        if self._rp_log is None or self._rp_log["path"] != os.path.abspath(os.path.expanduser(lp)):
            if not self._fl_load_log(lp):
                return
        log = self._rp_log
        vp = os.path.abspath(os.path.expanduser(self._rp_vid_e.get().strip()))
        if not self._rp_vid_e.get().strip() or not os.path.isfile(vp):
            self._rp_set_status("Choose the video file (already trimmed to takeoff - landing).", bad)
            return
        try:
            off, yaw, fov = (float(self._rp_off_e.get() or 0), float(self._rp_yaw_e.get() or 0),
                             float(self._rp_fov_e.get() or 60))
        except ValueError:
            self._rp_set_status("Sync offset, camera yaw and HFOV must be numbers.", bad)
            return
        if not 10.0 <= fov <= 170.0:
            self._rp_set_status("HFOV must be between 10 and 170 degrees.", bad)
            return
        if self._ga_proc is not None and self._ga_proc.poll() is None:
            self._rp_set_status("Switch the GEOANCHOR visual positioning card OFF first - the live node is running.", bad)
            return
        node = os.path.join(PROJECT_ROOT, "geoanchor_node.py")
        py = os.path.expanduser(self._ga_cfg["python"].get().strip())
        if not os.path.isfile(node):
            self._rp_set_status(f"geoanchor_node.py not found next to this file ({node})", bad)
            return
        if not os.path.isfile(py):
            self._rp_set_status(f"GeoAnchor python not found: {py}  (see the GEOANCHOR card)", bad)
            return
        # map area: the recorded GPS track (if any) + HOME (if set) + margin
        gps = log["gps"]
        pts = [(min(p[0] for p in gps), min(p[1] for p in gps)), (max(p[0] for p in gps), max(p[1] for p in gps))] if gps else []
        home = tuple(gps[0]) if gps else None
        if self.odo_home:
            home = (float(self.odo_home[0]), float(self.odo_home[1]))
            pts.append(home)
        if not pts or home is None:
            self._rp_set_status("The log has no GPS position and no HOME is set (HOME card above) - the map area "
                                "is unknown.", bad)
            return
        try:
            margin = float(self._ga_cfg["radius"].get() or 150)
        except ValueError:
            margin = 150.0
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            except OSError:
                pass
            sock.bind(("127.0.0.1", self._RP_FIX_PORT))
            sock.setblocking(False)
        except OSError as e:
            self._rp_set_status(f"UDP port {self._RP_FIX_PORT} is busy ({e}) - is another replay running?", bad)
            return
        cruise = log["cruise"]
        cmd = [py, "-u", node, "--map", "auto", "--replay", "--video", vp,
               "--alt", f"{cruise:g}", f"--min-alt-m={max(1.0, 0.5 * cruise):g}",
               "--port", str(self._RP_FIX_PORT), "--state-port", str(self._RP_STATE_PORT),
               "--home", "%.7f,%.7f" % home, "--points", ";".join("%.7f,%.7f" % p for p in pts),
               "--margin-m", f"{margin:g}", f"--hfov={math.radians(fov):.5f}", f"--cam-yaw={yaw:g}"]
        try:
            logf = open(os.path.join(PROJECT_ROOT, "geoanchor_replay.log"), "w")
            proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT, cwd=PROJECT_ROOT)
        except Exception as e:
            sock.close()
            self._rp_set_status(f"Could not start the GeoAnchor node: {e}", bad)
            return
        self._rp_clear_map()
        if len(gps) >= 2:                                    # light-blue dotted line = where the GPS says it flew
            step = max(1, len(gps) // 800)
            try:
                self._rp_ref_path = self.map_widget.set_path([tuple(p) for p in gps[::step]], color="#6fa8ff", width=2)
                self.map_widget.canvas.itemconfig(self._rp_ref_path.canvas_line, dash=(2, 6))
            except Exception:
                self._rp_ref_path = None
        rp.update(state="preparing", proc=proc, sock=sock, logf=logf, t0=None, idx=0, st={}, frame=None, cam=None,
                  node=("loading", "starting GeoAnchor ..."), fixes=0, off=off, sent=0.0, ui=0.0, vid_dur=None,
                  t_end=log["dur"], t_prep=time.time())
        self._rp_set_status("Preparing: loading the satellite map and the video (first run downloads the map "
                            "tiles - internet needed once) ...", self._NV_WARN)
        self._append_log(f">>> Flight replay: preparing {os.path.basename(log['path'])} + {os.path.basename(vp)} "
                         f"(log: geoanchor_replay.log)")

    def _rp_stop(self, msg="Replay stopped.", error=False):
        rp = self._rp
        p, rp["proc"] = rp["proc"], None
        if p is not None and p.poll() is None:
            try:
                p.terminate()
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    p.kill()
            except Exception:
                pass
        for key in ("sock", "logf"):
            obj, rp[key] = rp[key], None
            if obj is not None:
                try:
                    obj.close()
                except Exception:
                    pass
        was = rp["state"]
        rp["state"] = "idle"
        rp["cam"] = None                                    # the camera panel goes back to its normal source
        self._rp_set_status(msg, self._NV_BAD if error else self._NV_OK)
        if was != "idle":
            self._append_log((f"!!! Flight replay: {msg}" if error else f">>> Flight replay: {msg}"))

    def _rp_clear_map(self):
        self._ga_clear_dots()
        if self._rp_ref_path is not None:
            try:
                self._rp_ref_path.delete()
            except Exception:
                pass
            self._rp_ref_path = None

    # ------------------------------------------------------------------ replay: loop
    def _rp_tick(self):
        try:
            self._rp_step()
        except tk.TclError:
            return
        except Exception as e:                                   # never let the loop die
            if not self._rp["err_logged"]:
                self._rp["err_logged"] = True
                try:
                    self._append_log(f"!!! Flight replay error: {e}")
                except Exception:
                    pass
        self.root.after(30, self._rp_tick)

    def _rp_poll_sock(self):
        rp = self._rp
        sock = rp["sock"]
        for _ in range(400):
            if sock is None:
                return
            try:
                data, _a = sock.recvfrom(262144)
            except (BlockingIOError, InterruptedError, OSError):
                return
            if data[:4] == b"GAFR":                              # preview frame (jpeg) - only the newest is kept
                rp["frame"] = data[4:]
                continue
            try:
                m = json.loads(data.decode())
            except ValueError:
                continue
            kind = m.get("k") if isinstance(m, dict) else None
            if kind == "ga_status":
                rp["node"] = (m.get("state", ""), m.get("text", ""))
            elif kind == "ga_replay" and rp["state"] == "preparing":
                self._rp_begin_clock(m)
            elif kind == "ga_fix" and rp["state"] == "running":
                self._rp_on_fix(m)

    def _rp_begin_clock(self, m):
        """The node has the map and the video ready: from here ONE clock drives video and telemetry."""
        rp = self._rp
        vd = m.get("dur")
        rp["vid_dur"] = float(vd) if vd else None
        t_end = rp["t_end"]
        if rp["vid_dur"]:
            t_end = min(t_end, rp["vid_dur"] - rp["off"])
        if t_end < 1.0:
            self._rp_stop("The video and the log do not overlap - check the sync offset.", error=True)
            return
        rp["t_end"] = t_end
        rp["t0"] = time.monotonic()
        rp["state"] = "running"
        warn = ""
        if rp["vid_dur"] and rp["vid_dur"] - rp["off"] < self._rp_log["dur"] - 1.0:
            warn = f"  (the video is shorter than the log: replaying {t_end:.0f} of {self._rp_log['dur']:.0f} s)"
        self._rp_set_status("Replaying - the video plays in the live camera panel (bottom left)." + warn, self._NV_OK)
        self._append_log(">>> Flight replay started: video and telemetry run from the same clock")

    def _rp_on_fix(self, m):
        if m.get("mode") in self._GA_MODES_NO_FIX:
            return
        lat, lon = float(m["lat"]), float(m["lon"])
        self._ga_mark_fix(lat, lon, True, m.get("inl"), None)    # the same dots / red path as the live GeoAnchor
        self._rp["fixes"] += 1
        if self._rp["fixes"] == 1:
            try:
                self.map_widget.set_position(lat, lon)
            except Exception:
                pass

    def _rp_step(self):
        rp = self._rp
        if rp["state"] == "idle":
            return
        self._rp_poll_sock()
        p = rp["proc"]
        if p is not None and p.poll() is not None:
            why = rp["node"][1] if rp["node"][0] == "error" and rp["node"][1] else f"exit code {p.returncode}"
            self._rp_stop(f"The GeoAnchor node stopped ({why}) - see geoanchor_replay.log", error=True)
            return
        now = time.monotonic()
        if rp["state"] == "preparing":
            self._rp_render_frame()
            ns, nt = rp["node"]
            self._rp_set_status(f"Preparing ({time.time() - rp['t_prep']:.0f} s): {nt or 'starting ...'}", self._NV_WARN)
            return
        t = now - rp["t0"]
        if t >= rp["t_end"]:
            self._rp_stop(f"Replay finished ({rp['fixes']} visual fixes drawn on the map).")
            return
        # --- telemetry up to the clock position
        ev, st, i = self._rp_log["ev"], rp["st"], rp["idx"]
        while i < len(ev) and ev[i][0] <= t:
            st[ev[i][1]] = ev[i][2]
            i += 1
        rp["idx"] = i
        # --- clock + replayed attitude / altitude to the node (20 Hz); the node picks the video frame at vt
        if now - rp["sent"] >= 0.05:
            rp["sent"] = now
            att, vfr, pos = st.get("ATTITUDE"), st.get("VFR"), st.get("POSITION")
            hdg = vfr.get("heading") if vfr else (att["yaw"] % 360.0 if att and att.get("yaw") is not None else None)
            msg = {"k": "ga_state", "t": time.time(), "vt": t + rp["off"],
                   "roll": att.get("roll") if att else None, "pitch": att.get("pitch") if att else None,
                   "heading": hdg, "alt": pos.get("alt") if pos else None, "fresh": True}
            try:
                rp["sock"].sendto(json.dumps(msg).encode(), ("127.0.0.1", self._RP_STATE_PORT))
            except Exception:
                pass
        self._rp_render_frame()
        if now - rp["ui"] >= 0.1:
            rp["ui"] = now
            self._rp_readout.config(text=self._rp_readout_text(t), fg=self._NV_VAL)

    def _rp_render_frame(self):
        """Newest preview frame from the node -> kept for the live camera panel (see _rp_cam_frame)."""
        rp = self._rp
        b, rp["frame"] = rp["frame"], None
        if not b:
            return
        import io
        im = Image.open(io.BytesIO(b)).convert("RGB")
        rp["cam_n"] += 1
        rp["cam"] = {"fmt": "RGB_INT8", "w": im.width, "h": im.height, "data": im.tobytes(), "t": rp["cam_n"]}

    def _rp_sample(self):
        """odo_sample()-shaped dict built from the replayed log (None unless a replay is running)."""
        rp = self._rp
        if rp["state"] != "running" or self._rp_log is None:
            return None
        st = rp["st"]
        imu, att, vfr, pos = st.get("IMU"), st.get("ATTITUDE"), st.get("VFR"), st.get("POSITION")
        hdg = None
        if att and att.get("yaw") is not None:
            hdg = att["yaw"] % 360.0
        elif vfr and vfr.get("heading") is not None:
            hdg = float(vfr["heading"])
        return {"source": "telemetry", "t": time.time(), "age": 0.0, "fresh": True,
                "accel": list(imu["accel"]) if imu else None, "gyro": list(imu["gyro"]) if imu else None,
                "roll": att.get("roll") if att else None, "pitch": att.get("pitch") if att else None,
                "heading": hdg, "alt": pos.get("alt") if pos else None, "climb": vfr.get("climb") if vfr else None,
                "rate_hz": self._rp_log.get("imu_hz", 0.0), "gps_enabled": self.gps_enabled, "home": self.odo_home,
                "enabled": self.odo_enabled}

    def _rp_cam_frame(self):
        """(frame, led colour, text) for the live camera panel while a replay runs, else None."""
        rp = self._rp
        if rp["state"] == "idle" or rp["cam"] is None:
            return None
        if rp["state"] == "running":
            return rp["cam"], self._NV_OK, f"FLIGHT REPLAY  \u00b7  {time.monotonic() - rp['t0']:.0f} / {rp['t_end']:.0f} s"
        return rp["cam"], self._NV_WARN, "FLIGHT REPLAY  \u00b7  preparing"

    def _rp_readout_text(self, t):
        rp = self._rp
        st = rp["st"]
        g = lambda ty, key, fmt, dflt="--": (fmt % st[ty][key]) if ty in st and st[ty].get(key) is not None else dflt
        lines = [f"t {t:6.1f} / {rp['t_end']:.1f} s     video {t + rp['off']:6.1f} s     "
                 f"fixes {rp['fixes']}"]
        pos = st.get("POSITION")
        gpsraw = st.get("GPS_RAW")
        lat = f"{pos['lat']:.6f} {pos['lon']:.6f}" if pos and pos.get("lat") is not None else "--"
        sats = f"{gpsraw.get('satellites_visible', '--')} sat  fix {gpsraw.get('fix_type', '--')}" if gpsraw else "-- sat"
        lines.append(f"POS {lat}   ALT {g('POSITION', 'alt', '%.1f')} m   GPS {sats}")
        lines.append(f"SPD gs {g('VFR', 'groundspeed', '%.1f')} m/s  air {g('VFR', 'airspeed', '%.1f')}   "
                     f"climb {g('VFR', 'climb', '%+.1f')}   HDG {g('VFR', 'heading', '%s')}\u00b0")
        lines.append(f"ATT roll {g('ATTITUDE', 'roll', '%+.1f')}  pitch {g('ATTITUDE', 'pitch', '%+.1f')}  "
                     f"yaw {g('ATTITUDE', 'yaw', '%+.1f')}")
        imu = st.get("IMU")
        if imu:
            lines.append("IMU a %+.2f %+.2f %+.2f  g %+.1f %+.1f %+.1f" % (tuple(imu["accel"]) + tuple(imu["gyro"])))
        else:
            lines.append("IMU --")
        bat = st.get("BATTERY")
        hb = st.get("HEARTBEAT")
        lines.append(f"BAT {g('BATTERY', 'voltage', '%.2f')} V {g('BATTERY', 'remaining', '%s')} %   "
                     f"MODE {hb.get('mode', '--') if hb else '--'}   {'ARMED' if hb and hb.get('armed') else 'disarmed' if hb else ''}")
        return "\n".join(lines)

    # ------------------------------------------------------------------ build the card
    def _build_odo_flight_card(self):
        CARD, TILE = self._NAV_CARD, self._NAV_TILE
        body, self._fl_badge = self._odo_card(
            "FLIGHT LOG  -  record & synchronized replay",
            "Fly manually with the GCS connected: the telemetry is recorded by itself from takeoff to landing "
            "(the Record GPS switch here filters GPS in or out of the log; the GPS switch at the top of the panel "
            "only blocks GPS on the dashboard). "
            "Then replay it together with your own pre-trimmed downward-camera video: the video plays in the live "
            "camera panel (bottom left) from the same clock as the telemetry, GeoAnchor locates every frame on the "
            "map and each fix is drawn as a dot.")
        # ---------------- 1. automatic recording ----------------
        row = tk.Frame(body, bg=CARD)
        row.pack(fill="x")
        tk.Label(row, text="Automatic recording", font=("Segoe UI", 12, "bold"), fg=COL_TEXT, bg=CARD).pack(side="left")
        self._fl_switch = ToggleSwitch(row, value=True, command=self._fl_on_auto, bg=CARD, width=38, height=20)
        self._fl_switch.pack(side="left", padx=14)
        self._fl_state_lbl = tk.Label(row, text="", font=("Segoe UI", 10, "bold"), fg=COL_TEXT_DIM, bg=CARD)
        self._fl_state_lbl.pack(side="left")
        grow = tk.Frame(body, bg=CARD)
        grow.pack(fill="x", pady=(8, 0))
        tk.Label(grow, text="Record GPS", font=("Segoe UI", 12, "bold"), fg=COL_TEXT, bg=CARD).pack(side="left")
        self._fl_gps_switch = ToggleSwitch(grow, value=True, command=self._fl_on_gps, bg=CARD, width=38, height=20)
        self._fl_gps_switch.pack(side="left", padx=14)
        self._fl_gps_lbl = tk.Label(grow, text="", font=("Segoe UI", 10, "bold"), fg=COL_TEXT_DIM, bg=CARD)
        self._fl_gps_lbl.pack(side="left")
        box = tk.Frame(body, bg=TILE, highlightthickness=1, highlightbackground=COL_BORDER)
        box.pack(fill="x", pady=(10, 0))
        self._fl_stat_lbl = tk.Label(box, text="", font=("DejaVu Sans Mono", 9), fg=COL_TEXT_DIM, bg=TILE,
                                     anchor="w", justify="left", wraplength=560)
        self._fl_stat_lbl.pack(fill="x", padx=10, pady=(8, 0))
        self._fl_note_lbl = tk.Label(box, text="", font=("Segoe UI", 9), fg=COL_TEXT_DIM, bg=TILE,
                                     anchor="w", justify="left", wraplength=560)
        self._fl_note_lbl.pack(fill="x", padx=10, pady=(2, 8))
        self._odo_wraps.extend([self._fl_stat_lbl, self._fl_note_lbl])
        brow = tk.Frame(body, bg=CARD)
        brow.pack(anchor="w", pady=(8, 0))
        self._odo_button(brow, "DOWNLOAD", self._fl_download, width=14).pack(side="left", ipady=2)
        self._odo_button(brow, "USE", self._fl_use, kind="primary", width=10).pack(side="left", padx=(8, 0), ipady=2)
        tk.Label(brow, text="Download = save a copy of the newest log.   Use = load it into the replay below.",
                 font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=CARD).pack(side="left", padx=10)

        # ---------------- 2. synchronized replay ----------------
        tk.Frame(body, bg=COL_BORDER, height=1).pack(fill="x", pady=(14, 8))
        tk.Label(body, text="Synchronized replay", font=("Segoe UI", 12, "bold"), fg=COL_TEXT, bg=CARD,
                 anchor="w").pack(fill="x")
        grid = tk.Frame(body, bg=CARD)
        grid.pack(fill="x", pady=(8, 0))
        grid.grid_columnconfigure(1, weight=1)
        for r, (label, attr, cmd) in enumerate((("Flight log", "_rp_log_e", self._rp_browse_log),
                                                ("Video", "_rp_vid_e", self._rp_browse_video))):
            tk.Label(grid, text=label, font=("Segoe UI", 9), fg=COL_TEXT_DIM, bg=CARD,
                     anchor="w").grid(row=r, column=0, sticky="w", pady=3, padx=(0, 10))
            e = self._odo_entry(grid, 40)
            e.grid(row=r, column=1, sticky="ew", pady=3, ipady=3)
            setattr(self, attr, e)
            self._odo_button(grid, "BROWSE", cmd, width=9).grid(row=r, column=2, padx=(8, 0), pady=3)
        self._rp_log_info = tk.Label(body, text="", font=("DejaVu Sans Mono", 8), fg=COL_TEXT_DIM, bg=CARD,
                                     anchor="w", justify="left", wraplength=560)
        self._rp_log_info.pack(fill="x", pady=(2, 0))
        self._odo_wraps.append(self._rp_log_info)
        prow = tk.Frame(body, bg=CARD)
        prow.pack(fill="x", pady=(8, 0))
        for label, attr, default in (("Sync offset (s)", "_rp_off_e", "0"), ("Camera yaw (\u00b0)", "_rp_yaw_e", "0"),
                                     ("HFOV (\u00b0)", "_rp_fov_e", "60")):
            tk.Label(prow, text=label, font=("Segoe UI", 9), fg=COL_TEXT_DIM, bg=CARD).pack(side="left")
            e = self._odo_entry(prow, 6, default)
            e.pack(side="left", padx=(6, 16), ipady=3)
            setattr(self, attr, e)
        hint = tk.Label(body, text="Sync offset: video time = log time + offset (+ if your video starts before the "
                                   "takeoff, - if it starts after).  Camera yaw: bearing of the image-up direction "
                                   "relative to the drone's nose.  HFOV: horizontal field of view of the camera.",
                        font=("Segoe UI", 8), fg=COL_TEXT_DIM, bg=CARD, anchor="w", justify="left", wraplength=560)
        hint.pack(fill="x", pady=(4, 0))
        self._odo_wraps.append(hint)
        srow = tk.Frame(body, bg=CARD)
        srow.pack(anchor="w", pady=(10, 0))
        self._odo_button(srow, "START", self._rp_start, kind="primary", width=14).pack(side="left", ipady=2)
        self._odo_button(srow, "STOP", lambda: self._rp_stop("Replay stopped.") if self._rp["state"] != "idle" else None,
                         kind="danger", width=10).pack(side="left", padx=(8, 0), ipady=2)
        self._odo_button(srow, "CLEAR MAP", self._rp_clear_map, width=12).pack(side="left", padx=(8, 0), ipady=2)
        self._rp_status = tk.Label(body, text="Choose a flight log and the trimmed video, then press START.",
                                   font=("Segoe UI", 9), fg=COL_TEXT_DIM, bg=CARD, anchor="w", justify="left",
                                   wraplength=560)
        self._rp_status.pack(fill="x", pady=(8, 0))
        self._odo_wraps.append(self._rp_status)
        self._rp_readout = tk.Label(body, text="", font=("DejaVu Sans Mono", 9), fg=COL_TEXT_DIM, bg=CARD,
                                    anchor="w", justify="left")
        self._rp_readout.pack(fill="x", pady=(8, 0))
        self._fl_refresh_ui()
        self.root.after(500, self._fl_tick)
        self.root.after(500, self._rp_tick)

##############################################################################
# SECTION: dashboard.py   (was gcs/app/dashboard.py)
##############################################################################
class DashboardApp(
        LayoutMixin,
        SidePanelsMixin,
        UiHelpersMixin,
        StatusBarMixin,
        ConnectionMixin,
        MapPanelMixin,
        MapOverlaysMixin,
        MapStyleMixin,
        Map3DMixin,
        WaypointsMixin,
        TelemetryPanelMixin,
        ControlPanelMixin,
        ParametersMixin,
        GaugesPanelMixin,
        FailsafePanelMixin,
        LogPanelMixin,
        SwarmViewMixin,
        SwarmCommonMixin,
        SwarmMathMixin,
        FormationBarMixin,
        SwarmBusMixin,
        AgentProcessMixin,
        SwarmMissionMixin,
        SwarmRtlMixin,
        SwarmSupervisorMixin,
        NavPanelMixin,
        NavRebuildMixin,
        NavUpdateMixin,
        NavStoryMixin,
        NavDrawMixin,
        OdoPanelMixin,
        PollLoopMixin,
):
    """The whole GCS window. All behaviour lives in the mixins (see gcs/app/*); 
    this class only builds the shared state in __init__."""

    def __init__(self, root, mav, data_queue):
        self.root = root
        self.mav = mav
        self.data_queue = data_queue
        self.root.title("Drone Ground Control Dashboard")
        self.root.geometry(f"{min(1680, self.root.winfo_screenwidth())}x1460")   # wide enough for the whole bottom bar
        self.root.configure(bg=COL_BG)

        self.values = {}
        self.drone_marker = None
        self.swarm_markers = {}   # sysid -> map marker, for every drone that isn't sysid 1

        self.mission_markers = []
        self.mission_path = None
        self.rtl_path = None

        self.marking_mode = False
        # Waypoint marking now always acts on whichever drones are ticked
        # in the DRONES/Telemetry panel (self.swarm_selected) -- there's
        # no separate "which drone am I marking for" selector anymore, so
        # there's nothing that can disagree with it. See
        # _selected_sysids() / _update_marking_indicator().
        self.manual_waypoints = {}             # sysid -> [(lat, lon), ...] (per-drone, offset-adjusted)
        self.manual_markers = {}               # sysid -> [map marker, ...] (only ever populated for
                                                #   the first ticked drone -- see _add_manual_waypoint)
        self.manual_path = {}                  # sysid -> map path or None
        self.drone_colors = {1: DEFAULT_WP_COLORS[1]}  # sysid -> waypoint/path color
        self.wp_pin_icons = {}                 # label -> cached pin PhotoImage
        self.drone_icon_cache = {}             # rounded heading -> cached drone PhotoImage
        self.marker_heading = {}               # sysid -> last heading applied to its map icon
        self.swarm_positions = {}              # sysid -> (lat, lon), latest known
        # sysid -> (dlat, dlon) offset from the first ticked drone's live
        # position, captured when marking mode is turned on (or the tick
        # selection changes while marking) -- keeps drones the same
        # distance apart while flying a marked route together.
        self.marking_formation_offsets = {}

        self.last_lat = -35.363262
        self.last_lon = 149.165237
        self.current_tile_server = "satellite"

        # 3D Map subprocess tracking (indicator pulse + poll timers)
        self.map3d_process = None
        self.map3d_pulse_after_id = None
        self.map3d_poll_after_id = None
        self.map3d_pulse_on = False

        # Waypoint progress state (for the map overlay)
        self.wp_total = 0
        self.wp_current = 0

        # Telemetry radio / mode switching state
        self.telemetry_port = None
        self.mode_override = "AUTO"   # "AUTO", "SIM", or "REAL"
        self._current_target = None   # (mode, connection_string) currently active

        # Manual Connect/Disconnect (Mission-Planner-style, top-right of the
        # status bar) -- while True, the existing AUTO/SIM/REAL logic above
        # steps aside (see _apply_mode_logic) so it can't fight with a
        # connection the user started/stopped by hand.
        self.manual_conn_active = False

        # Parameters now lives in its own popup window instead of inline
        # under Flight Controls -- built lazily on first open, then just
        # shown/hidden after that (see _open_parameters_popup) so the
        # Entry/Label widgets inside stay alive for background param
        # reads/writes even while the window is closed.
        self._params_popup = None

        # Tracks which parameter READ is currently outstanding, so stray
        # PARAM_VALUE messages from other traffic on the link (e.g. MAVProxy
        # auto-fetching its full parameter list) don't overwrite the readout.
        self.pending_param_read = None

        # ---- Swarm (multi-drone) state ----
        # All drones share the SAME connection (self.mav) -- they're
        # distinguished purely by MAVLink system ID (sysid), matching how
        # `sim_vehicle.py --count N --auto-sysid` actually works: every
        # vehicle's telemetry arrives multiplexed over one socket.
        #
        # Drones are discovered dynamically as new sysids are seen (via
        # HEARTBEAT) -- spawn a 4th vehicle and "Drone D" appears on its
        # own, no code change needed. sysid 1 is always treated as the
        # classic single "Drone A" shown in the main Telemetry panel.
        self.swarm_order = []          # sysids in first-seen order -> drone numbers 1,2,3,4...
        self.prearm_status = {}        # sysid -> (status_text, color) for Pre-Arm indicator
        self.swarm_data = {}           # sysid -> dict of telemetry values
        self.swarm_widgets = {}        # sysid -> dict of Label/Checkbutton widgets
        self.nav_fields = {}           # sysid -> {field: text} mirror used by the SWARM NAV panel
        self.swarm_selected = {}       # sysid -> tk.BooleanVar
        self.swarm_view_active = False  # False = showing single Telemetry, True = showing DRONES

        # ATTITUDE/HEADING gauge box: which drone's attitude/heading is
        # currently shown, plus the latest known raw values for every
        # drone so switching the dropdown shows that drone's own last
        # reading immediately, without waiting on new telemetry.
        self.gauge_sysid = 1
        self.swarm_attitude_val = {}   # sysid -> (roll_deg, pitch_deg)
        self.swarm_heading_val = {}    # sysid -> heading_deg

        self.drone_icon = create_drone_icon()

        self.panels = []              # every floating GlassPanel (see _run_layout)
        self.app_active = True        # False while another application has focus
        self._unfocused_polls = 0
        self._layout_job = None
        self._map_stack = []          # WP / battery / GPS / locate overlays on the map
        self._popup_seq = 0           # bumps every time a popup panel is shown (stacking order)
        self._raise_jobs = []

        # ---- Swarm formation auto-flight (Swarm Nav) ----
        self.swarm_form_shape = "polygon"   # formation shape chosen in Formation Control
        self.swarm_form_d = None      # "Distance between" in metres once SET (None = formation off)
        self._sw = None               # running swarm-auto state dict, or None
        self._last_sw = None          # last completed/active swarm mission dict, or None
        self._swarm_job = None
        # -- swarm bus: the GCS only hands the mission to the per-drone agents and
        #    listens to what they report (see swarm_agent.py). It never flies them.
        self._bus = None              # UDP socket on SWARM_BUS_BASE
        self._bus_states = {}         # sysid -> last "state" message from that drone's agent
        self._agent_procs = {}        # sysid -> (Popen, logfile) for agents this GCS started
        self._swarm_poll_job = None
        self._bus_links = {}          # sysid -> last "link" report (what that agent REALLY sends / receives)
        self._feed = []               # swarm-bus traffic lines [(seq, time, text)]
        self._feed_seq = 0
        self._feed_last_tx = ("", 0.0)
        self._cmd_log = {}            # sysid -> [(seq, time, text)] commands that reached that autopilot
        self._cmd_seq = 0
        self._nav_feed_text = None
        self._nav_feed_shown = 0
        self._nav_bus_hdr = None
        self._nav_err_shown = False
        self._nav_wraps = []
        self._nav_raw_open = False
        self._nav_v = {"order": []}
        self._nav_slots = {}          # sysid -> formation slot index (from the agents' 'formation agreed' event)
        self._gcs_rx = {}             # sysid -> packets the DASHBOARD itself received from that agent
        self._tri_ids = None          # sysids whose triangle is drawn on the map
        self._tri_visible = False
        self._tri_lines = {}          # (sid_a, sid_b) -> map path (green)
        self._tri_labels = {}         # (sid_a, sid_b) -> map marker carrying the distance text
        self._tri_last = 0.0
        self._tri_blank = None
        self.swarm_trail_paths = {}   # sysid -> solid map path of the ground actually covered
        self.swarm_trail_pts = {}     # sysid -> [(lat, lon), ...]
        self._home_markers = {}       # sysid -> "H<n>" map marker, placed when the drone reaches its altitude
        self._home_airborne = set()   # sysids seen flying (so a drone sitting on the ground never clears its marker)

        self._odo_init_state()        # VISUAL ODO: GPS switch, manual home, Gazebo bridge state
        self._build_layout()
        self._apply_mode_logic()  # starts A in SIM until a radio is detected
        self._poll_queue()
        self._check_link_health()
        self._tri_blank = tk.PhotoImage(width=1, height=1)   # invisible icon -> marker shows text only
        self._swarm_bus_open()


##############################################################################
# SECTION: main.py   (was gcs/main.py)
##############################################################################
def main():
    data_queue = queue.Queue()
    mav = MAVLink(data_queue)

    serial_watcher = SerialWatcher(data_queue)
    serial_watcher.start()

    root = tk.Tk()
    app = DashboardApp(root, mav, data_queue)

    def on_close():
        try:
            app.shutdown_swarm()
        except Exception:
            pass
        try:
            app.odo_shutdown()
        except Exception:
            pass
        serial_watcher.stop()
        mav.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
