"""gcs_ui.py -- EVERYTHING that looks like something.

Colours, sizes, map-style list, timings (the old config.py), the glass panel, the icons,
every widget (EdgeButton, gauges, RSSI graph, battery, check box) and the shared
button / section-header / instrument-card helpers (UiHelpersMixin).
Change a colour or a button style here and it changes everywhere.
"""
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
# Folder that holds gcs5.py / gcs_ui.py / gcs_3d.py / swarm_agent.py (the project root). The GCS starts
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
# SECTION: compass_gauge.py   (was gcs/ui/widgets/compass_gauge.py)
##############################################################################
class CompassGauge(tk.Canvas):
    """Heading indicator styled after a real directional gyro: a
    numbered compass card (N/E/S/W plus the 3/6/12/15/21/24/30/33
    minor markers) that rotates so the current heading always sits at
    the top lubber line, beneath a FIXED white aircraft silhouette
    (matching the reference gauge photo)."""

    def __init__(self, parent, size=84, **kwargs):
        super().__init__(parent, width=size, height=size, bg=COL_BG,
                          highlightthickness=0, **kwargs)
        self.size = size
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
class AttitudeGauge(tk.Canvas):
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
        r = s / 2 - 16   # leaves room outside the disk for the fixed bezel scale

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
