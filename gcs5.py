#!/usr/bin/env python3
"""gcs5.py -- the WHOLE ground station in one file.  Run:  python3 gcs5.py

  gcs_ui.py       all colours / sizes / buttons / widgets / panel look   (imported)
  gcs_3d.py       the 3D Map button                                       (imported)
  swarm_agent.py  all swarm algorithms; the GCS starts one copy per drone (imported for
                  formation_slots so there is ONE single copy of the formation geometry)

Sections of this file, top to bottom:
  1 MAVLink link      mav_commands / mav_mission / mavlink / serial_watcher
  2 Connection + top status bar
  3 Window layout + side tabs
  4 Map (panel, style menu, overlays, waypoints)
  5 Panels (control, gauges, failsafe, telemetry, parameters, log)
  6 poll_loop         incoming telemetry -> UI
  7 Swarm             bus, agent processes, view, formation bar, mission, RTL, supervisor, navigation panel
  8 DashboardApp      glues every mixin together (all shared state lives in its __init__)
  9 main()
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

from gcs_ui import (
    AttitudeGauge,
    BatteryIndicator,
    COL_ARMED,
    COL_BG,
    COL_BORDER,
    COL_BTN,
    COL_BTN_ACCENT,
    COL_BTN_HOVER,
    COL_CARD,
    COL_DOT_OFF,
    COL_DOT_ON,
    COL_GLASS,
    COL_GLASS_BORDER,
    COL_MISSION_PATH,
    COL_RTL_PATH,
    COL_TEXT,
    COL_TEXT_DIM,
    COL_TRAJECTORY,
    COL_VALUE,
    COL_WARN,
    CompassGauge,
    DEFAULT_MAP_ZOOM,
    DEFAULT_WP_COLORS,
    EdgeButton,
    GlassPanel,
    LINK_CHECK_MS,
    LINK_TIMEOUT_S,
    MAP_STYLES,
    MAP_STYLE_BY_ID,
    MAX_LOG_LINES,
    PROJECT_ROOT,
    REAL_BAUD_RATE,
    ReadoutRow,
    SATELLITE_TILE_SERVER,
    SERIAL_POLL_INTERVAL_S,
    SIM_CONNECTION_STRING,
    SmoothCheck,
    TELEMETRY_PORT_HINTS,
    TELEM_PANEL_W,
    UPDATE_INTERVAL_MS,
    UiHelpersMixin,
    WP_COLOR_POOL,
    _round_window,
    cardinal_for,
    create_drone_icon,
    create_wp_pin_icon,
    display_mode,
)
from swarm_agent import (
    FORMATION_SHAPES,
    formation_slots,
)
from gcs_3d import Map3DMixin


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
            "EKF_STATUS_REPORT": 2,
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
            dy=lambda W, H: 8 + self.toolbar_panel.frame.winfo_reqheight() + 10)
        self._build_gauges_panel(self.gauges_panel.body)

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

        self._section_header(toolbar, "MAP", first=True)
        self._build_map_style_dropdown(toolbar)

        self._build_3d_map_button(toolbar)

        self._section_header(toolbar, "MISSION")
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
        if not self.marking_mode:
            return
        lat, lon = coords
        self._add_manual_waypoint(lat, lon)

    def _on_canvas_click_fallback(self, event):
        if not self.marking_mode:
            return
        try:
            lat, lon = self.map_widget.convert_canvas_coords_to_decimal_coords(event.x, event.y)
        except Exception:
            return
        self._add_manual_waypoint(lat, lon)


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
        btn("SWARM NAV", self._toggle_swarm_nav, bold=True, width=12)
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
        """Vertical strip under the map buttons: a drone selector on top and
        two instrument cards (attitude, heading) with captioned readouts.
        Switching the dropdown swaps both gauges to that drone's own latest
        attitude/heading -- see _on_gauge_drone_change()."""
        box = tk.Frame(parent, bg=COL_GLASS)
        box.pack(padx=10, pady=8, fill="x")

        self._build_gauge_selector(box)

        card = self._instrument_card(box, "ATTITUDE")
        self.attitude_gauge = AttitudeGauge(card, size=124)
        self.attitude_gauge.config(bg=COL_CARD)
        self.attitude_gauge.pack(pady=(6, 4), padx=6)
        self.attitude_readout = ReadoutRow(card, ("ROLL", "PITCH"))
        self.attitude_readout.pack(fill="x", padx=6, pady=(0, 8))

        card = self._instrument_card(box, "HEADING")
        self.compass_gauge = CompassGauge(card, size=124)
        self.compass_gauge.config(bg=COL_CARD)
        self.compass_gauge.pack(pady=(6, 4), padx=6)
        self.compass_readout = ReadoutRow(card, ("HDG", "DIR"))
        self.compass_readout.pack(fill="x", padx=6, pady=(0, 8))

    def _build_gauge_selector(self, box):
        """Flat dropdown ('Drone 1 v') -- rebuilt any time a new drone is
        discovered (see _rebuild_gauge_menu(), called from _ensure_drone_ui)."""
        self._section_header(box, "INSTRUMENTS", first=True)
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
                    has_fix = not (data["lat"] == 0.0 and data["lon"] == 0.0)
                    if has_fix:
                        self.swarm_positions[sysid] = (data["lat"], data["lon"])
                    self._update_swarm_field(sysid, "alt", f"{data['alt']:.2f}")
                    self._update_swarm_field(sysid, "lat", f"{data['lat']:.6f}")
                    self._update_swarm_field(sysid, "lon", f"{data['lon']:.6f}")
                    if sysid == 1:
                        self.values["alt"].config(text=f"{data['alt']:.2f}")
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
            if now - st["rx"] < 1.0 and (not pts or math.hypot(*self._ll_to_ne(pts[-1], st["lat"], st["lon"])) > 0.4):
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
        PollLoopMixin,
):
    """The whole GCS window. All behaviour lives in the mixins (see gcs/app/*); 
    this class only builds the shared state in __init__."""

    def __init__(self, root, mav, data_queue):
        self.root = root
        self.mav = mav
        self.data_queue = data_queue
        self.root.title("Drone Ground Control Dashboard")
        self.root.geometry("1550x1460")
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
        serial_watcher.stop()
        mav.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
