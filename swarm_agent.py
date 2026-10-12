"""
swarm_agent.py  --  one copy runs for EACH drone (think: the companion computer
on that drone).  There is no central swarm controller:

  - every agent talks to ITS OWN ArduPilot over MAVLink,
  - every agent talks to the OTHER agents over a small UDP "swarm bus"
    (position / velocity / phase / formation reference, ~10 Hz),
  - every agent runs the same code and decides from its own state, its
    neighbours' states and the common mission goal.

The GCS only (1) hands the mission goal to the agents once and (2) listens to
what the agents report.  It sends nothing to the autopilots during the run
(except the operator's ABORT MISSION / RTL).

Mission goal = destination route + spacing "d" + altitude + speed.  The agents
then, by themselves:   GUIDED -> arm -> take off -> wait for each other ->
form the formation (side d) -> fly the route keeping the shape -> settle at
the destination.

Formation control (all distributed):

  - slots     : the formation is a polygon / wedge / line / column centred on
                the reference point.
  - reference : a shared formation reference point R + heading th. Every agent
                advances its own copy and averages it with the neighbours'
                copies (consensus), so the copies stay together without a leader.
  - cohesion  : R only advances as fast as the WORST slot error among all
                members allows, so a lagging drone makes everybody wait for it.
  - corners   : R brakes smoothly into every corner / the destination, pivots on
                the spot at a rate the outer drones can actually fly, then
                accelerates away again -> the shape is kept in turns.
  - control   : pure velocity commands
                v = vR + (turn feed-forward) + Kp*(slot - pos) + Ki*int + repulsion,
                acceleration-limited (no position set-points -> no overshoot).

RTL (return to launch) -- also fully distributed:

  1. each drone remembers its OWN home (where it took off from) and shares it,
  2. on RTL the swarm climbs +5 m IN FORMATION (braking smoothly if it was
     flying), pivots, and flies back in formation (collision avoidance active
     the whole time) to the centre of all the home points,
  3. at the very end the formation is BROKEN one drone at a time:
       - every drone moves to its own altitude layer (3 m apart) -> from now on
         no two drones are ever at the same height,
       - drone #1 (closest to its home) flies above its home and lands,
       - when it has landed, drone #2 breaks away, flies above ITS home, lands, ...
     so every drone ends up on its own home point and nobody can collide.

Run by hand (SITL, instance i of `sim_vehicle.py --count N --auto-sysid`):
    python3 swarm_agent.py --master tcp:127.0.0.1:5762 --sysid 1
    python3 swarm_agent.py --master tcp:127.0.0.1:5772 --sysid 2
    python3 swarm_agent.py --master tcp:127.0.0.1:5782 --sysid 3
(gcs2.py starts them for you automatically when you press AUTO MODE.)

----------------------------------------------------------------------
FILE MAP (everything algorithmic is in THIS one file, top to bottom):
  config     gains / limits / ports / phase names
  util       link-rate counters
  geometry   lat/lon<->north/east, formation slots (the GCS imports these too -> one single copy)
  bus        UdpBus: UDP swarm bus between agents and the GCS
  vehicle    MavVehicle: this drone's MAVLink + low level commands
  formation  shared formation reference
  control    formation / transit state machine, holds, velocity control
  rtl        return-to-launch + one-by-one landing
  handlers   incoming bus messages (peers, mission, rtl, stop)
  agent      Agent class (tick loop, phases, broadcast)
  main       entry point
"""
import math
import json
import socket
import time
import itertools
import argparse


##############################################################################
# SECTION: config.py   (was agent/config.py)
##############################################################################
BUS_HOST = "127.0.0.1"
BUS_BASE = 15000          # GCS listens on BUS_BASE, agent <sysid> on BUS_BASE + sysid
CTRL_HZ = 10.0

PHASES = ["IDLE", "PREP", "WAIT", "FORM", "TRANSIT", "SETTLE", "DONE", "STOPPED"]
PH = {p: i for i, p in enumerate(PHASES)}

KP = 0.8                  # slot-error -> velocity gain (1/s)
KI = 0.04                 # slot-error integral gain (1/s^2): removes steady offsets (wind, drift)
I_MAX = 1.5               # m/s clamp on the integral velocity (anti-windup)
ACC_LIMIT = 1.5           # m/s^2 slew limit on the commanded velocity
M_PER_DEG = 111320.0
TURN_MAX = math.radians(25.0)   # absolute ceiling on the formation turn rate (rad/s)
A_REF = 0.8               # m/s^2  reference speeds up at most this fast (< ACC_LIMIT so followers keep up)
D_REF = 1.2               # m/s^2  ... and slows down at most this fast
A_TURN = 0.4              # rad/s^2 turn-rate ramp

# ---- RTL landing (breaking the formation at the very end) ----
LAYER_STEP = 3.0          # m  vertical gap between the drones' private altitude layers
HOME_TOL = 0.8            # m  "I am above my home point"
BREAK_SPEED = 2.5         # m/s max horizontal speed while a drone leaves the formation for its home
DESCEND_V = 2.0           # m/s controlled descent over the home point, down to LAND_HANDOVER
LAND_HANDOVER = 3.0       # m  above ground: hand over to the autopilot's LAND mode
VCLEAR = 2.5              # m  drones closer than this vertically are not separated by altitude
RTL_CLIMB_TIMEOUT = 12.0  # s  safety timeout of the "climb first" step
LAND_TIMEOUT = 150.0      # s  give up waiting for LAND mode to touch down


##############################################################################
# SECTION: util.py   (was agent/util.py)
##############################################################################
def _bump(w, now):
    """Tiny rate counter. w = [window_start, count_in_window, last_rate_hz]."""
    if w[0] is None:
        w[0] = now
    w[1] += 1
    if now - w[0] >= 1.0:
        w[2] = w[1] / (now - w[0])
        w[0] = now
        w[1] = 0


def _hz(w, now):
    return w[2] if w[0] is not None and now - w[0] < 2.5 else 0.0


##############################################################################
# SECTION: geometry.py   (was agent/geometry.py)
##############################################################################
# ----------------------------------------------------------------- geometry

def ll_to_ne(o, lat, lon):
    return ((lat - o[0]) * M_PER_DEG,
            (lon - o[1]) * M_PER_DEG * math.cos(math.radians(o[0])))


def ne_to_ll(o, n, e):
    return (o[0] + n / M_PER_DEG,
            o[1] + e / (M_PER_DEG * math.cos(math.radians(o[0]))))


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


FORMATION_SHAPES = (("polygon", "Polygon"), ("wedge", "Wedge"), ("line", "Line"), ("column", "Column"))


def formation_slots(count, d, shape="polygon"):
    """(forward, right) offsets from the formation CENTRE.  Adjacent drones are d apart.
    polygon: regular n-gon (3 = triangle, 4 = square, 5 = pentagon ...)
    wedge  : V / arrow, leader at the apex     line  : side by side     column: single file"""
    n = count
    if n <= 1:
        return [(0.0, 0.0)]
    if shape == "line":
        return [(0.0, (i - (n - 1) / 2.0) * d) for i in range(n)]
    if shape == "column":
        return [(((n - 1) / 2.0 - i) * d, 0.0) for i in range(n)]
    if shape == "polygon" and n >= 3:
        R = d / (2.0 * math.sin(math.pi / n))
        off = math.pi / n if n % 2 == 0 else 0.0     # even n: flat front edge (square), odd n: apex first
        pts = []
        for i in range(n):
            a = off + 2.0 * math.pi * i / n
            pts.append((R * math.cos(a), R * math.sin(a)))
        return pts
    raw = [(0.0, 0.0)]                                # wedge (also 2-drone fallback)
    for i in range(1, n):
        k = (i + 1) // 2
        side = -1.0 if i % 2 == 1 else 1.0
        raw.append((-0.8660254 * d * k, side * 0.5 * d * k))
    mf = sum(s[0] for s in raw) / n
    mr = sum(s[1] for s in raw) / n
    return [(f - mf, r - mr) for f, r in raw]


def slot_point(R, th, slot):
    f, r = slot
    return (R[0] + f * math.cos(th) - r * math.sin(th),
            R[1] + f * math.sin(th) + r * math.cos(th))


##############################################################################
# SECTION: bus.py   (was agent/bus.py)
##############################################################################
# ----------------------------------------------------------------- UDP bus

class UdpBus:
    def __init__(self, sysid):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # no SO_REUSEADDR on purpose: a 2nd agent for the same drone must fail to bind
        self.sock.bind((BUS_HOST, BUS_BASE + sysid))
        self.sock.setblocking(False)

    def send(self, msg, targets):
        data = json.dumps(msg).encode()
        for t in targets:
            try:
                self.sock.sendto(data, (BUS_HOST, BUS_BASE + t))
            except OSError:
                pass

    def recv_all(self):
        out = []
        while True:
            try:
                data, addr = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            try:
                msg = json.loads(data.decode())
                if isinstance(msg, dict):
                    msg["_src"] = addr[1] - BUS_BASE      # sysid of the sender (0 = GCS)
                    out.append(msg)
            except ValueError:
                pass
        return out


##############################################################################
# SECTION: vehicle.py   (was agent/vehicle.py)
##############################################################################
# ----------------------------------------------------------------- ArduPilot link

class MavVehicle:
    FRAME = 6                                    # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

    def __init__(self, conn):
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.m = mavutil.mavlink_connection(conn, source_system=250, source_component=191)
        self.sysid = None
        t0 = time.time()
        while self.sysid is None and time.time() - t0 < 60:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb is not None and hb.type != 6 and hb.autopilot != 8:
                self.sysid = hb.get_srcSystem()
                self.comp = hb.get_srcComponent()
        if self.sysid is None:
            raise SystemExit(f"no autopilot heartbeat on {conn}")
        self.s = None
        self._mode, self._armed = "?", False
        self._pos = None
        self._modes = self.m.mode_mapping() or {}
        mav = self.m.mav
        mav.request_data_stream_send(self.sysid, self.comp, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
        mav.command_long_send(self.sysid, self.comp, 511, 0, 33, 100000, 0, 0, 0, 0, 0)

    def poll(self):
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if msg.get_srcSystem() != self.sysid:
                continue
            if t == "HEARTBEAT" and msg.type != 6:
                self._mode = self.mavutil.mode_string_v10(msg)
                self._armed = bool(msg.base_mode & 128)
            elif t == "GLOBAL_POSITION_INT":
                self._pos = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0,
                             msg.vx / 100.0, msg.vy / 100.0,
                             None if msg.hdg == 65535 else msg.hdg / 100.0)

    def state(self):
        if not self._pos or (self._pos[0] == 0.0 and self._pos[1] == 0.0):
            return None
        lat, lon, alt, vn, ve, hdg = self._pos
        return {"lat": lat, "lon": lon, "alt": alt, "vn": vn, "ve": ve, "hdg": hdg,
                "mode": self._mode, "armed": self._armed}

    def _set_mode(self, name):
        mid = self._modes.get(name)
        if mid is not None:
            self.m.mav.set_mode_send(self.sysid, 1, mid)

    def set_guided(self):
        self._set_mode("GUIDED")

    def brake(self):
        self._set_mode("BRAKE")

    def land(self):
        self._set_mode("LAND")

    def arm(self):
        self.m.mav.command_long_send(self.sysid, self.comp, 400, 0, 1, 0, 0, 0, 0, 0, 0)

    def takeoff(self, alt):
        self.m.mav.command_long_send(self.sysid, self.comp, 22, 0, 0, 0, 0, 0, 0, 0, float(alt))

    def set_speed(self, ms):
        self.m.mav.command_long_send(self.sysid, self.comp, 178, 0, 1, float(ms), -1, 0, 0, 0, 0)

    def send_vel(self, vn, ve, vd, yaw):
        mask = 7 | 448 | 2048                    # velocity (+ yaw) only
        if yaw is None:
            mask |= 1024
        self.m.mav.set_position_target_global_int_send(
            int(time.time() * 1000) & 0xFFFFFFFF, self.sysid, self.comp, self.FRAME, mask,
            0, 0, 0.0, float(vn), float(ve), float(vd), 0, 0, 0,
            float(yaw if yaw is not None else 0.0), 0)

    def send_pos(self, lat, lon, alt, yaw):
        mask = 56 | 448 | 2048                   # position (+ yaw) only
        self.m.mav.set_position_target_global_int_send(
            int(time.time() * 1000) & 0xFFFFFFFF, self.sysid, self.comp, self.FRAME, mask,
            int(lat * 1e7), int(lon * 1e7), float(alt), 0, 0, 0, 0, 0, 0, float(yaw), 0)


##############################################################################
# SECTION: formation_ctl.py   (was agent/formation_ctl.py)
##############################################################################
class FormationMixin:
    """Shared formation reference: create / adopt."""

    # ---- formation agreement ----
    def init_formation(self, st, now, route_ne, d):
        ids = self.m["ids"]
        n = len(ids)
        pos = self.positions(st, now)
        if any(s not in pos for s in ids):
            return
        cn = sum(pos[s][0] for s in ids) / n
        ce = sum(pos[s][1] for s in ids) / n
        w0 = route_ne[0]
        th = math.atan2(w0[1] - ce, w0[0] - cn) if math.hypot(w0[0] - cn, w0[1] - ce) > 1.0 else 0.0
        slots = formation_slots(n, d, self.m.get("shape", "polygon"))
        R = [cn, ce]                       # slots are centre-based -> least movement
        best, best_cost = None, 1e18
        for perm in itertools.permutations(range(n)):
            cost = 0.0
            for s, k in zip(ids, perm):
                pn, pe = slot_point(R, th, slots[k])
                cost += (pn - pos[s][0]) ** 2 + (pe - pos[s][1]) ** 2
            if cost < best_cost:
                best, best_cost = perm, cost
        self.sm = {s: k for s, k in zip(ids, best)}
        self.R, self.th, self.wi = R, th, 0
        self.set_phase("FORM", now)
        self.evt("form", sm=self.sm, to=[s for s in ids if s != self.id])

    def adopt_formation(self, fm):
        self.sm = {int(k): v for k, v in fm["sm"].items()}
        self.R, self.th, self.wi = list(fm["R"]), fm["th"], 0
        self.set_phase("FORM", self.now())
        self.evt("form", sm=self.sm, frm=self.m["ids"][0], adopt=True)


##############################################################################
# SECTION: control.py   (was agent/control.py)
##############################################################################
class ControlMixin:
    """Flight control: the formation / transit state machine and holds."""

    # ---- the behaviour ----
    def control(self, st, now, dt):
        m = self.m
        ids = m["ids"]
        n = len(ids)

        # ---- remember HOME = the spot this drone takes off from ----
        if self.home is None:
            self.home = (st["lat"], st["lon"])

        # ---- PREP: GUIDED -> arm -> take off (this drone only) ----
        if self.phase == "PREP":
            if now - self.t_phase > 90:
                self.stop("take-off timed out (90 s)", brake=False)
                return
            p = self.prep
            if st["mode"] != "GUIDED":
                if now - p["mode"] > 2.0:
                    self.veh.set_guided()
                    self.cmd("SET_MODE", "GUIDED")
                    p["mode"] = now
            elif not st["armed"]:
                if now - p["arm"] > 3.0:
                    self.veh.arm()
                    self.cmd("ARM")
                    p["arm"] = now
            else:
                if not p["speed"]:
                    self.veh.set_speed(m["speed"] * 1.5 + 1.0)   # headroom: a lagging drone must be able to out-run the reference
                    self.cmd("SET_SPEED", f"{m['speed'] * 1.5 + 1.0:.1f} m/s")
                    p["speed"] = True
                if st["alt"] < 0.6:
                    if now - p["to"] > 5.0 and p["tries"] < 5:
                        self.veh.takeoff(m["alt"])
                        p["to"] = now
                        p["tries"] += 1
                        self.cmd("TAKEOFF", f"{m['alt']:g} m (try {p['tries']})")
                elif st["alt"] < m["alt"] - 1.0:
                    self.send_vel(0.0, 0.0, st, None)
                if st["alt"] >= m["alt"] - 1.0:
                    self.set_phase("WAIT", now)
                    self.log("at altitude -- waiting for the other drones")
            return

        # ---- WAIT: hover until every member is airborne ----
        if self.phase == "WAIT":
            if now - self.t_phase > 90:
                self.stop("other drones never reached altitude (90 s)", brake=True)
                return
            self.send_vel(0.0, 0.0, st, None)
            fp = self.fresh_peers(now)
            if len(fp) == n - 1 and all(PH.get(p.get("phase"), 0) >= PH["WAIT"] for p in fp.values()):
                self.set_phase("FORM", now)
                self.fm_wait = now
                self.log("all drones airborne -- agreeing the formation")
            return

        # ---- from here on: FORM / TRANSIT / SETTLE ----
        if now - self.t_phase > 3.0 or self.phase != "FORM":
            if st["mode"] != "GUIDED" or not st["armed"]:
                why = "disarmed" if st["mode"] == "GUIDED" else f"flight mode changed to {st['mode']}"
                self.stop(why, brake=False)
                return
        fp = self.fresh_peers(now, 2.5)
        for s in ids:
            if s == self.id:
                continue
            p = self.peers.get(s)
            if p is not None and p.get("phase") == "STOPPED" and not self.peer_landed(p):
                # a neighbour stopped: give a possible RTL/BRAKE command time to arrive first
                if self.stop_wait is None:
                    self.stop_wait = now
                if now - self.stop_wait > 1.5:
                    self.stop(f"Drone {s} stopped ({p.get('why', '')})", brake=True)
                    return
            elif s not in fp and now - self.t_phase > 3.0 and s not in self.landed:
                self.stop(f"lost contact with Drone {s}", brake=True)
                return

        d = float(m["d"])
        o = m["o"]
        route_ne = [ll_to_ne(o, la, lo) for la, lo in m["route"]]
        speed = float(m["speed"])
        tol = max(0.7, 0.12 * d)

        # ---- FORM start: agree on reference, heading and slot assignment ----
        if not self.sm:
            lowest = ids[0]
            if self.id == lowest:
                self.init_formation(st, now, route_ne, d)
            else:
                p = self.peers.get(lowest)
                if p and p.get("fm"):
                    self.adopt_formation(p["fm"])
                elif now - (self.fm_wait or now) > 4.0 and lowest not in fp:
                    self.init_formation(st, now, route_ne, d)
                else:
                    self.send_vel(0.0, 0.0, st, None)
                    return
            if not self.sm or self.id not in self.sm:
                self.send_vel(0.0, 0.0, st, None)
                return
            self.log(f"formation agreed: slot {self.sm[self.id]} of {n}, spacing {d:g} m")

        # only neighbours on the same mission (same mid: an RTL has its own mid) for phase follow and consensus
        same_m = [p for p in fp.values() if p.get("mid") == self.m["mid"]]

        # ---- follow neighbours' phase progress (monotone) ----
        for p in same_m:
            ph = p.get("phase")
            if PH.get(ph, 0) > PH[self.phase] and ph in ("TRANSIT", "SETTLE", "DONE"):
                if ph == "DONE":
                    continue
                self.R, self.th, self.wi = list(p["R"]), p["th"], self.clamp_wi(p["wi"], route_ne)
                self.vref = p.get("vr", 0.0)
                self.set_phase(ph, now)
        # ---- consensus on the moving reference ----
        if self.phase in ("TRANSIT", "SETTLE"):
            same = [p for p in same_m if p.get("wi") == self.wi and p.get("phase") in ("TRANSIT", "SETTLE")]
            ahead = [p for p in same_m if p.get("wi", 0) > self.wi and p.get("phase") in ("TRANSIT", "SETTLE")]
            if ahead:
                p = ahead[0]
                self.R, self.th, self.wi = list(p["R"]), p["th"], self.clamp_wi(p["wi"], route_ne)
                self.vref = p.get("vr", self.vref)
            elif same:
                mr = [sum(p["R"][i] for p in same) / len(same) for i in (0, 1)]
                self.R = [self.R[i] + 0.3 * (mr[i] - self.R[i]) for i in (0, 1)]
                sx = sum(math.cos(p["th"]) for p in same) + math.cos(self.th)
                sy = sum(math.sin(p["th"]) for p in same) + math.sin(self.th)
                self.th = self.th + 0.3 * wrap_pi(math.atan2(sy, sx) - self.th)
        self.wi = self.clamp_wi(self.wi, route_ne)

        # ---- advance the reference (TRANSIT) ----
        refv = (0.0, 0.0)
        w_prev = self.omega
        self.omega = 0.0
        if self.phase == "TRANSIT":
            # ---- RTL climb phase: climb IN FORMATION until every drone is at the RTL altitude ----
            if self.climbing_rtl:
                mine_ok = st["alt"] >= m["alt"] - 0.8
                others_ok = all(p.get("alt", 0.0) >= m["alt"] - 0.8 for p in same_m)
                timed_out = (now - self.t_rtl_start) >= RTL_CLIMB_TIMEOUT
                if (mine_ok and others_ok) or timed_out:
                    self.climbing_rtl = False
                    reason = "altitude reached" if (mine_ok and others_ok) else "climb timeout"
                    self.log(f"RTL: {reason} ({st['alt']:.1f} m / {m['alt']:g} m) -- flying back in formation")
                else:
                    # brake the reference smoothly (it may have been flying) and stop its rotation
                    self.vref = max(0.0, self.vref - D_REF * dt)
                    if self.vref > 0.0:
                        ux, uy = math.cos(self.th), math.sin(self.th)
                        self.R = [self.R[0] + ux * self.vref * dt, self.R[1] + uy * self.vref * dt]
                        refv = (ux * self.vref, uy * self.vref)
                    w = w_prev - max(-A_TURN * dt, min(A_TURN * dt, w_prev))
                    self.th += w * dt
                    self.omega = w
            if not self.climbing_rtl:
                scale = min(1.0, max(0.0, 1.0 - (self.emax - tol) / (0.3 * d + 1.0)))
                tgt = route_ne[self.wi]
                dn, de = tgt[0] - self.R[0], tgt[1] - self.R[1]
                dist = math.hypot(dn, de)
                last = self.wi >= len(route_ne) - 1
                # ---- turning: rate capped by what the OUTER drone can fly, throttled when anyone lags
                align = 1.0
                if dist > 0.5:
                    herr = wrap_pi(math.atan2(de, dn) - self.th)
                    rmax = max(math.hypot(f, r) for f, r in formation_slots(n, d, self.m.get("shape", "polygon")))
                    rim = min(2.0, max(0.6, 0.4 * speed))
                    wlim = min(TURN_MAX, rim / max(rmax, 1.0))
                    w_t = max(-wlim, min(wlim, 2.0 * herr)) * scale
                    w = w_prev + max(-A_TURN * dt, min(A_TURN * dt, w_t - w_prev))
                    self.th += w * dt
                    self.omega = w
                    # pivot on the spot until we roughly face the next waypoint
                    align = min(1.0, max(0.0, 1.0 - (abs(herr) - math.radians(8.0)) / math.radians(25.0)))
                # ---- speed: smooth ramp; must be ~0 at a real corner and at the destination
                need_stop = last
                if not last:
                    nxt = route_ne[self.wi + 1]
                    a1 = math.atan2(de, dn)
                    a2 = math.atan2(nxt[1] - tgt[1], nxt[0] - tgt[0])
                    need_stop = abs(wrap_pi(a2 - a1)) > math.radians(10.0)
                vwant = speed * scale * align
                if need_stop:
                    vwant = min(vwant, max(0.15, math.sqrt(2.0 * D_REF * 0.8 * max(0.0, dist - 0.4))))
                if self.vref < vwant:
                    self.vref = min(vwant, self.vref + A_REF * dt)
                else:
                    self.vref = max(vwant, self.vref - D_REF * dt)
                step = self.vref * dt
                if dist <= max(step, 0.4):
                    self.R = [tgt[0], tgt[1]]
                    self.wi += 1
                    if self.wi >= len(route_ne):
                        self.wi = len(route_ne) - 1
                        self.vref = 0.0
                        self.set_phase("SETTLE", now)
                        self.log("reference reached the destination -- settling")
                elif dist > 0:
                    ux, uy = dn / dist, de / dist
                    self.R = [self.R[0] + ux * step, self.R[1] + uy * step]
                    refv = (ux * self.vref, uy * self.vref)

        # ---- slots and everybody's error (every agent can compute this) ----
        slots = formation_slots(n, d, self.m.get("shape", "polygon"))
        pos = self.positions(st, now)
        slot_ne = {s: slot_point(self.R, self.th, slots[self.sm[s]]) for s in ids if s in self.sm}
        errs = {s: math.hypot(slot_ne[s][0] - pos[s][0], slot_ne[s][1] - pos[s][1])
                for s in ids if s in pos and s in slot_ne}
        e_now = max(errs.values()) if errs else 0.0
        self.emax += (e_now - self.emax) * min(1.0, dt / 0.8)     # ~0.8 s low-pass: GPS jitter must not stall the swarm
        self.inpos = sum(1 for e in errs.values() if e < 1.5 * tol)
        pr, tg = self.R, 0.0
        for w in route_ne[self.wi:]:
            tg += math.hypot(w[0] - pr[0], w[1] - pr[1])
            pr = w
        self.togo = tg

        # ---- my velocity command ----
        pn, pe = pos[self.id]
        tn, te = slot_ne[self.id]
        # v = reference velocity + velocity of my slot due to the formation turning + P on the error
        en, ee = tn - pn, te - pe
        if math.hypot(en, ee) < 1.5:                     # integrate only when close to the slot (anti-windup)
            self.ivel[0] = max(-I_MAX, min(I_MAX, self.ivel[0] + KI * en * dt))
            self.ivel[1] = max(-I_MAX, min(I_MAX, self.ivel[1] + KI * ee * dt))
        vn = refv[0] - self.omega * (te - self.R[1]) + KP * en + self.ivel[0]
        ve = refv[1] + self.omega * (tn - self.R[0]) + KP * ee + self.ivel[1]
        dsafe = max(2.0, 0.45 * d)
        for s, (qn, qe) in pos.items():                 # local collision avoidance
            if s == self.id or s in self.landed:
                continue
            dx, dy = pn - qn, pe - qe
            dist = math.hypot(dx, dy)
            if 0.05 < dist < dsafe:
                push = (dsafe - dist) * 0.8
                vn += dx / dist * push
                ve += dy / dist * push
        if self.phase == "TRANSIT":
            vcap = speed * 1.5 + 0.5
        else:
            vcap = min(max(speed * 1.5, 1.5), 3.0)
        mag = math.hypot(vn, ve)
        if mag > vcap:
            vn, ve = vn * vcap / mag, ve * vcap / mag
        vn, ve = self.slew(vn, ve, dt)
        self.send_vel(vn, ve, st, wrap_pi(self.th))

        # ---- phase logic ----
        if self.phase == "FORM":
            if self.emax < tol:
                self.ok_since = self.ok_since or now
                if now - self.ok_since >= 2.0:
                    self.set_phase("TRANSIT", now)
                    if self.is_rtl:
                        self.t_rtl_start = now
                    self.log(f"formation built ({d:g} m) -- heading to the destination")
            else:
                self.ok_since = None
            if now - self.t_phase > 120:
                self.stop("formation did not settle in 120 s", brake=True)
        elif self.phase == "SETTLE":
            if self.emax < tol:
                self.ok_since = self.ok_since or now
            else:
                self.ok_since = None
            if (self.ok_since and now - self.ok_since >= 1.5) or now - self.t_phase > 20:
                la, lo = ne_to_ll(o, *slot_ne[self.id])
                for _ in range(3):
                    self.veh.send_pos(la, lo, m["alt"], wrap_pi(self.th))
                self.cmd("GOTO", f"{la:.6f} {lo:.6f} alt {m['alt']:g} m (hold slot)")
                self.evt("phase", frm=self.phase, to="DONE")
                self.phase = "DONE"
                self.vcmd = [0.0, 0.0]
                if self.is_rtl:
                    self.land_state = ""         # rtl_land() takes over from the next tick
                    self.log(f"RTL: formation is above the launch area ({d:g} m spacing) "
                             f"-- breaking formation to land one by one")
                else:
                    self.log(f"destination reached -- holding the formation ({d:g} m)")

    def clamp_wi(self, wi, route_ne):
        try:
            wi = int(wi)
        except (TypeError, ValueError):
            wi = 0
        return max(0, min(len(route_ne) - 1, wi))

    # ---- RTL: break the formation at the very end and land on the home points, one by one ----
    def _enter(self, state, now):
        self.land_state = state
        self.t_land_state = now
        self.over_since = None

    def _hold(self, st, now, dt, target_ne, alt, vmax=BREAK_SPEED, repel=True, vz_max=1.0):
        """Velocity command toward target_ne (horizontal P, capped), altitude held at `alt`,
        with repulsion from every airborne neighbour that is at a similar height."""
        pos = self.positions(st, now)
        pn, pe = pos[self.id]
        en, ee = target_ne[0] - pn, target_ne[1] - pe
        vn, ve = KP * en, KP * ee
        mag = math.hypot(vn, ve)
        if mag > vmax:
            vn, ve = vn * vmax / mag, ve * vmax / mag
        if repel:
            dsafe = max(2.0, 0.45 * float(self.m["d"]))
            for s, (qn, qe) in pos.items():
                if s == self.id or s in self.landed:
                    continue
                p = self.peers.get(s)
                if p is not None and abs(p.get("alt", 0.0) - st["alt"]) >= VCLEAR:
                    continue                      # separated by altitude: no horizontal push needed
                dx, dy = pn - qn, pe - qe
                dist = math.hypot(dx, dy)
                if 0.05 < dist < dsafe:
                    push = (dsafe - dist) * 0.8
                    vn += dx / dist * push
                    ve += dy / dist * push
        vn, ve = self.slew(vn, ve, dt)
        self.send_vel(vn, ve, st, wrap_pi(self.th), alt=alt, vz_max=vz_max)
        return math.hypot(en, ee)

    def _compute_order(self, st, now):
        """Landing order: the drone closest to its own home lands first (ties: lower sysid)."""
        o = self.m["o"]
        pos = self.positions(st, now)
        keyed = []
        for s in self.m["ids"]:
            if s not in pos:
                continue
            hm = self.home if s == self.id else (self.peers.get(s) or {}).get("home")
            if hm:
                hn, he = ll_to_ne(o, hm[0], hm[1])
                dist = math.hypot(hn - pos[s][0], he - pos[s][1])
            else:
                dist = 0.0
            keyed.append((round(dist, 1), s))
        return [s for _, s in sorted(keyed)]


##############################################################################
# SECTION: rtl.py   (was agent/rtl.py)
##############################################################################
class RtlMixin:
    """Return-to-launch: break formation, per-drone landing sequence."""

    def rtl_land(self, st, now, dt):
        m = self.m
        ids = m["ids"]
        o = m["o"]

        if self.land_state == "":
            self.land_state = "HOLD"
            self.t_land = self.t_land_state = now
            pn, pe = ll_to_ne(o, st["lat"], st["lon"])
            self.land_hold = (pn, pe)
            self.land_order = None
            self.land_layer = None
            if self.home is None:                # should not happen: land where we are
                self.home = (st["lat"], st["lon"])
        ls = self.land_state
        hn, he = ll_to_ne(o, self.home[0], self.home[1])

        # ---------- DOWN: LAND mode is flying, wait for touch-down ----------
        if ls == "DOWN":
            on_ground = st["alt"] < 0.35 or (not st["armed"] and st["alt"] < 1.0)
            if on_ground or now - self.t_land_state > LAND_TIMEOUT:
                self.land_state = "LANDED"
                self.evt("phase", frm=self.phase, to="STOPPED")
                self.phase = "STOPPED"
                self.why = "RTL landed" if on_ground else "RTL landing timeout"
                self.landed.add(self.id)
                self.log(f"landed on its home point ({st['lat']:.6f}, {st['lon']:.6f})"
                         if on_ground else "landing timeout")
            return

        # ---------- everything before DOWN is flown in GUIDED: watch for a manual mode change ----------
        if st["mode"] != "GUIDED" or not st["armed"]:
            self.land_state = "ABORT"
            self.stop_from_done("disarmed" if st["mode"] == "GUIDED" else f"flight mode changed to {st['mode']}")
            return

        fp = self.fresh_peers(now, 2.5)
        my_rank = self.land_order.index(self.id) if self.land_order and self.id in self.land_order else 0

        # ---------- HOLD: wait until everyone is above the launch area, agree the landing order ----------
        if ls == "HOLD":
            self._hold(st, now, dt, self.land_hold, m["alt"], vmax=2.0)
            waiting = []
            for s in ids:
                if s == self.id or s in self.landed:
                    continue
                p = fp.get(s)
                if p is None or p.get("phase") not in ("DONE",):
                    waiting.append(s)
            if waiting and now - self.t_land < 45.0:
                return
            lead = ids[0]
            if self.id == lead or lead in self.landed:
                order = self._compute_order(st, now)
            else:
                lp = fp.get(lead)
                if lp is not None and lp.get("lo"):
                    order = [int(s) for s in lp["lo"]]
                elif now - self.t_land > 8.0:
                    order = self._compute_order(st, now)      # lead silent: decide locally
                else:
                    return
            if self.id not in order:
                order.append(self.id)
            self.land_order = order
            rank = order.index(self.id)
            self.land_layer = float(m["alt"]) + LAYER_STEP * rank
            self.log(f"landing order {order} -- I am #{rank + 1}, private altitude {self.land_layer:g} m")
            self._enter("LAYER", now)
            return

        # ---------- LAYER: climb straight up to my private altitude (nobody shares a height from now on) ----------
        if ls == "LAYER":
            self._hold(st, now, dt, self.land_hold, self.land_layer, vmax=1.0)
            if abs(st["alt"] - self.land_layer) < 0.8 or now - self.t_land_state > 25.0:
                self._enter("READY", now)
            return

        # ---------- READY: all on their layers; I break away when the drone before me has started landing ----------
        if ls == "READY":
            self._hold(st, now, dt, self.land_hold, self.land_layer, vmax=1.0)
            all_ready = True
            for s in ids:
                if s == self.id or s in self.landed:
                    continue
                p = fp.get(s)
                if p is not None and p.get("ls") in ("", "HOLD", "LAYER") and now - self.t_land_state < 30.0:
                    all_ready = False
            if not all_ready:
                return
            before = self.land_order[:my_rank]
            if all((s in self.landed) or ((self.peers.get(s) or {}).get("ls") in ("DESCEND", "DOWN", "LANDED"))
                   for s in before):
                self.log("breaking away from the formation -- flying above my home point")
                self.cmd("BREAK", f"to home {self.home[0]:.6f} {self.home[1]:.6f}")
                self._enter("BREAK", now)
            return

        # ---------- BREAK / OVER: fly (at my own layer) above my home point and wait for my turn ----------
        if ls in ("BREAK", "OVER"):
            dist = self._hold(st, now, dt, (hn, he), self.land_layer)
            before = self.land_order[:my_rank]
            clear = all(s in self.landed for s in before)
            if dist < HOME_TOL:
                self.over_since = self.over_since or now
            else:
                self.over_since = None
            if self.over_since and ls == "BREAK":
                self._enter("OVER", now)
                self.over_since = now
                self.log("above my home point -- waiting for the drones before me to land")
            if ls == "OVER" and clear and dist < 2.0 * HOME_TOL and now - self.t_land_state > 1.0:
                self.log("my turn -- descending over my home point")
                self._enter("DESCEND", now)
            return

        # ---------- DESCEND: straight down over the home point (everybody else is above me or on the ground) ----------
        if ls == "DESCEND":
            self._hold(st, now, dt, (hn, he), LAND_HANDOVER, vmax=1.0, vz_max=DESCEND_V)
            if st["alt"] <= LAND_HANDOVER + 0.8 or now - self.t_land_state > 60.0:
                self.veh.land()
                self.cmd("SET_MODE", "LAND")
                self.log("landing on my home point")
                self._enter("DOWN", now)
            return


##############################################################################
# SECTION: handlers.py   (was agent/handlers.py)
##############################################################################
class HandlersMixin:
    """Incoming bus messages: peers, mission, RTL."""

    # ---- bus input ----
    def on_msg(self, msg, now):
        k = msg.get("k")
        if k == "state":
            sid = msg.get("id")
            if sid is not None and sid != self.id:
                msg["rx"] = now
                self.peers[sid] = msg
                if self.peer_landed(msg):
                    self.landed.add(sid)
                rs = self.rx_stats.setdefault(sid, {"n": 0, "w": [None, 0, 0.0]})
                rs["n"] += 1
                _bump(rs["w"], now)
        elif k == "mission":
            self.on_mission(msg, now)
        elif k == "param":
            if self.m and msg.get("mid") == self.m["mid"] and msg.get("d"):
                self.m["d"] = float(msg["d"])
                self.evt("bus", dir="rx", what=f"param d={self.m['d']:g} m", frm=msg.get("_src", 0))
                self.log(f"spacing changed to {self.m['d']:g} m")
        elif k == "stop":
            if self.m and self.phase not in ("DONE", "STOPPED", "IDLE"):
                self.evt("bus", dir="rx", what="STOP", frm=msg.get("_src", 0))
                self.stop(f"stop request ({msg.get('why', 'GCS')})", brake=False)
            elif self.m and self.phase == "DONE" and self.land_state not in ("", "LANDED"):
                # operator ABORT while the formation is being broken for landing: freeze here
                self.evt("bus", dir="rx", what="STOP", frm=msg.get("_src", 0))
                self.land_state = "ABORT"
                self.phase = "DONE"
                self.stop_from_done(f"stop request ({msg.get('why', 'GCS')})")
        elif k == "rtl":
            self.on_rtl(msg, now)
        elif k == "ping":
            self.broadcast(force=True)

    @staticmethod
    def peer_landed(p):
        return p.get("ls") == "LANDED" or (p.get("phase") == "STOPPED"
                                           and str(p.get("why", "")).startswith("RTL landed"))

    def on_rtl(self, msg, now):
        ids = msg.get("ids") or (self.m.get("ids") if self.m else None)
        if not ids or self.id not in ids:
            return
        if self.m is None:                       # never had a mission: nothing to return from
            return

        mid = msg.get("mid")
        if mid:
            if mid in self.seen_missions:
                return
            self.seen_missions.add(mid)

        # gossip once to peer drones
        others = [s for s in ids if s != self.id]
        self.bus.send(msg, others)

        if self.phase in ("IDLE", "STOPPED"):    # not flying: nothing to bring home
            return
        if self.land_state not in ("", ):        # already breaking formation / landing
            return

        # RTL always goes DIRECTLY to the return point (centre of the home points, chosen by the GCS).
        # The GCS already sent alt = mission_alt + 5.0 so we simply adopt it.
        o = msg.get("o") or self.m["o"]
        route = msg.get("route") or [list(o)]
        rtl_alt = float(msg.get("alt", self.m["alt"] + 5.0))

        self.m["route"] = [list(p) for p in route]
        self.m["alt"] = rtl_alt    # agents will climb to this before translating horizontally
        if mid:
            self.m["mid"] = mid    # new "mission id": GCS + consensus can tell RTL agents from old-mission agents
        if msg.get("d"):
            self.m["d"] = float(msg["d"])
        if msg.get("speed"):
            self.m["speed"] = float(msg["speed"])

        self.is_rtl = True
        self.climbing_rtl = True   # hold horizontal movement until the climb is complete
        self.t_rtl_start = now
        self.wi = 0                # (vref is NOT reset: the reference brakes smoothly while we climb)
        self.ivel = [0.0, 0.0]
        self.ok_since = None
        self.stop_wait = None
        self.why = ""

        if self.phase in ("DONE", "SETTLE", "TRANSIT"):
            self.set_phase("TRANSIT", now)
            self.log(f"RTL initiated: climbing to {rtl_alt:g} m, then flying back in formation "
                     f"(spacing {self.m['d']:g} m)")
            self.evt("bus", dir="rx", what="RTL", frm=msg.get("_src", 0))
        elif self.phase in ("FORM", "WAIT", "PREP"):
            self.log(f"RTL queued: will climb to {rtl_alt:g} m then return after the formation is built")
            self.evt("bus", dir="rx", what="RTL", frm=msg.get("_src", 0))

    def on_mission(self, msg, now):
        mid = msg.get("mid")
        if self.id not in msg.get("ids", []):
            return
        if mid in self.seen_missions:
            return
        if self.phase not in ("IDLE", "DONE", "STOPPED"):
            return
        self.seen_missions.add(mid)
        self.is_rtl = False
        self.climbing_rtl = False
        self.m = {k: msg[k] for k in ("mid", "ids", "d", "alt", "speed", "route", "o")}
        self.m["shape"] = msg.get("shape", "polygon")
        self.m["ids"] = sorted(self.m["ids"])
        self.peers = {}
        self.wi = 0
        self.R = [0.0, 0.0]
        self.th = 0.0
        self.sm = {}
        self.fm_wait = None
        self.vcmd = [0.0, 0.0]
        self.vref = 0.0
        self.ivel = [0.0, 0.0]
        self.omega = 0.0
        self.ok_since = None
        self.emax = self.togo = 0.0
        self.inpos = 0
        self.why = ""
        self.stop_wait = None
        self.home = None
        self.land_state = ""
        self.land_order = None
        self.land_layer = None
        self.land_hold = None
        self.landed = set()
        self.over_since = None
        self.prep = {"mode": 0.0, "arm": 0.0, "to": 0.0, "tries": 0, "speed": False}
        self.set_phase("PREP", now)
        self.log(f"mission received: {len(self.m['ids'])} drones, spacing {self.m['d']:g} m, "
                 f"{len(self.m['route'])} waypoint(s), alt {self.m['alt']:g} m")
        self.evt("bus", dir="rx", what="mission", frm=msg.get("_src", 0))
        # gossip: pass the goal on to the other members (decentralised distribution)
        others = [s for s in self.m["ids"] if s != self.id]
        self.bus.send(msg, others)
        self.evt("bus", dir="tx", what="mission (gossip)", to=others)


##############################################################################
# SECTION: agent.py   (was agent/agent.py)
##############################################################################
class Agent(HandlersMixin, ControlMixin, RtlMixin, FormationMixin):
    """One drone's decentralised controller. Behaviour is split over the mixins in agent/*.
    This file keeps the state, the 10 Hz tick, and the bus broadcast."""

    def __init__(self, sysid, veh, bus, clock=time.time, log=print):
        self.id = sysid
        self.veh = veh
        self.bus = bus
        self.now = clock
        self._log = log

        self.m = None              # current mission dict
        self.phase = "IDLE"
        self.why = ""
        self.peers = {}            # sid -> last state msg (+ "rx" receive time)
        self.last_tick = None
        self.seen_missions = set()

        self.R = [0.0, 0.0]        # formation reference (north, east) from mission origin
        self.th = 0.0              # formation heading (rad, 0 = north)
        self.wi = 0                # index of the waypoint the reference is heading for
        self.sm = {}               # sysid -> slot index
        self.fm_wait = None
        self.vcmd = [0.0, 0.0]     # last velocity command (for the slew limit)
        self.emax = 0.0            # worst slot error among all members (low-pass filtered)
        self.omega = 0.0
        self.vref = 0.0            # speed of the reference point (m/s)
        self.ivel = [0.0, 0.0]     # integral part of the velocity command
        self.inpos = 0
        self.togo = 0.0
        self.t_phase = 0.0
        self.ok_since = None
        self.prep = {"mode": 0.0, "arm": 0.0, "to": 0.0, "tries": 0, "speed": False}
        self.t_state = 0.0
        self.stop_wait = None

        # ---- home point (where THIS drone took off) ----
        self.home = None           # (lat, lon)

        # ---- RTL ----
        self.is_rtl = False
        self.climbing_rtl = False  # True while the swarm climbs (in formation) before flying home
        self.t_rtl_start = 0.0
        # ---- RTL: breaking the formation and landing one by one ----
        self.land_state = ""       # "" | HOLD | LAYER | READY | BREAK | OVER | DOWN | LANDED
        self.land_order = None     # landing order (sysids), decided by the lowest-id agent
        self.land_layer = None     # my private altitude while the formation is being broken
        self.land_hold = None      # (n, e) I hold while waiting
        self.t_land = 0.0
        self.t_land_state = 0.0
        self.landed = set()        # sysids known to be on the ground at home
        self.over_since = None

        # ---- link / command telemetry for the GCS window (display only, never used for control) ----
        self.rx_stats = {}         # sid -> {"n": packets received, "w": rate counter}
        self.tx_n = 0
        self.tx_w = [None, 0, 0.0]
        self.vel_n = 0
        self.vel_w = [None, 0, 0.0]
        self.last_vel = None       # (vn, ve, vd, yaw_deg) of the last velocity set-point sent to ArduPilot
        self.t_link = 0.0

    # ---- reporting to the GCS window ----
    def evt(self, ev, **kw):
        """One-off event for the GCS SWARM NAV panel (bus message or autopilot command)."""
        kw.update({"k": "evt", "id": self.id, "ev": ev})
        self.bus.send(kw, [0])

    def cmd(self, what, arg=""):
        self.evt("cmd", what=what, arg=arg)

    def report_link(self, now):
        """~2 Hz: what THIS agent really receives from each peer, what it sends, what it feeds ArduPilot."""
        if now - self.t_link < 0.5:
            return
        self.t_link = now
        rx = {}
        for sid, p in self.peers.items():
            s_ = self.rx_stats.get(sid)
            if s_ is None:
                continue
            rx[str(sid)] = {
                "n": s_["n"], "hz": round(_hz(s_["w"], now), 1), "age": round(now - p["rx"], 2),
                **{k: p.get(k) for k in ("lat", "lon", "alt", "vn", "ve", "hdg", "phase", "mode",
                                         "armed", "R", "th", "wi", "vr", "emax")}}
        to = [s for s in self.m["ids"] if s != self.id] if self.m and self.phase != "IDLE" else []
        self.bus.send({"k": "link", "id": self.id, "mid": self.m["mid"] if self.m else 0,
                       "tx_n": self.tx_n, "tx_hz": round(_hz(self.tx_w, now), 1), "to": to, "rx": rx,
                       "vel": list(self.last_vel) if self.last_vel else None,
                       "vel_hz": round(_hz(self.vel_w, now), 1), "vel_n": self.vel_n}, [0])

    # ---- logging (also forwarded to the GCS) ----
    def log(self, text):
        self._log(f"[agent {self.id}] {text}")
        self.bus.send({"k": "log", "id": self.id, "m": text}, [0])

    def set_phase(self, phase, now):
        if phase != self.phase:
            self.evt("phase", frm=self.phase, to=phase)
            self.phase = phase
            self.t_phase = now
            self.ok_since = None

    # ---- stopping ----
    def stop(self, why, brake):
        if self.phase in ("STOPPED", "DONE"):
            return
        if brake:
            self.veh.brake()
            self.cmd("SET_MODE", "BRAKE")
        self.evt("phase", frm=self.phase, to="STOPPED")
        self.phase = "STOPPED"
        self.why = why
        self.log(f"STOPPED: {why}" + (" (holding position)" if brake else ""))

    def stop_from_done(self, why):
        """Stop while in DONE (RTL landing sequence): freeze with BRAKE and report STOPPED."""
        self.veh.brake()
        self.cmd("SET_MODE", "BRAKE")
        self.evt("phase", frm=self.phase, to="STOPPED")
        self.phase = "STOPPED"
        self.why = why
        self.log(f"STOPPED: {why} (holding position)")

    # ---- one control tick ----
    def tick(self):
        now = self.now()
        dt = 1.0 / CTRL_HZ if self.last_tick is None else min(0.5, max(0.02, now - self.last_tick))
        self.last_tick = now
        for msg in self.bus.recv_all():
            self.on_msg(msg, now)
        self.veh.poll()
        st = self.veh.state()
        if st is not None and self.m and self.phase not in ("IDLE", "DONE", "STOPPED"):
            try:
                self.control(st, now, dt)
            except Exception as e:                      # never fly blind
                self.stop(f"internal error: {e}", brake=True)
        elif st is not None and self.m and self.phase == "DONE" and self.is_rtl \
                and self.land_state not in ("ABORT", "LANDED"):
            try:
                self.rtl_land(st, now, dt)
            except Exception as e:                      # never fly blind: land right here, right now
                self.log(f"landing sequence error ({e}) -- landing in place")
                self.veh.land()
                self.cmd("SET_MODE", "LAND")
                self.land_state = "DOWN"
                self.t_land_state = now
        if st is not None:
            self.broadcast(st=st, now=now)
        self.report_link(now)

    # ---- helpers ----
    def fresh_peers(self, now, max_age=1.0):
        return {s: p for s, p in self.peers.items()
                if s in self.m["ids"] and now - p["rx"] <= max_age}

    def positions(self, st, now):
        """Best known (north, east) of every member, neighbours extrapolated by age."""
        o = self.m["o"]
        out = {self.id: ll_to_ne(o, st["lat"], st["lon"])}
        for s, p in self.peers.items():
            if s not in self.m["ids"] or s == self.id:
                continue
            age = min(0.5, now - p["rx"])
            n, e = ll_to_ne(o, p["lat"], p["lon"])
            out[s] = (n + p.get("vn", 0.0) * age, e + p.get("ve", 0.0) * age)
        return out

    def send_vel(self, vn, ve, st, yaw, alt=None, vz_max=1.0):
        # altitude hold through the vertical velocity
        tgt = self.m["alt"] if alt is None else alt
        vz_up = max(-vz_max, min(vz_max, 0.8 * (tgt - st["alt"])))
        self.veh.send_vel(vn, ve, -vz_up, yaw)
        self.last_vel = (vn, ve, -vz_up, None if yaw is None else math.degrees(yaw))
        self.vel_n += 1
        _bump(self.vel_w, self.now())

    def slew(self, vn, ve, dt):
        dn, de = vn - self.vcmd[0], ve - self.vcmd[1]
        mag = math.hypot(dn, de)
        lim = ACC_LIMIT * dt
        if mag > lim:
            dn, de = dn * lim / mag, de * lim / mag
        self.vcmd = [self.vcmd[0] + dn, self.vcmd[1] + de]
        return self.vcmd

    # ---- bus output ----
    def broadcast(self, st=None, now=None, force=False):
        now = now if now is not None else self.now()
        if st is None:
            st = self.veh.state()
            if st is None:
                return
        landing = self.phase == "DONE" and self.land_state not in ("", "LANDED", "ABORT")
        idle = self.phase in ("IDLE", "DONE", "STOPPED") and not landing and not force
        if idle and now - self.t_state < 0.5:
            return
        if not idle and now - self.t_state < 0.9 / CTRL_HZ and not force:
            return
        self.t_state = now
        msg = {"k": "state", "id": self.id, "mid": self.m["mid"] if self.m else 0,
               "phase": self.phase, "why": self.why,
               "lat": st["lat"], "lon": st["lon"], "alt": st["alt"],
               "vn": st["vn"], "ve": st["ve"], "hdg": st.get("hdg"), "mode": st["mode"], "armed": st["armed"],
               "R": self.R, "th": self.th, "wi": self.wi, "vr": self.vref,
               "emax": self.emax, "inpos": self.inpos, "togo": self.togo,
               "d": self.m["d"] if self.m else 0.0,
               "home": list(self.home) if self.home else None,
               "ls": self.land_state}
        if self.land_order and self.m and self.id == self.m["ids"][0]:
            msg["lo"] = list(self.land_order)
        if self.sm and self.m and self.id == self.m["ids"][0]:
            msg["fm"] = {"sm": self.sm, "R": self.R if self.phase == "FORM" else list(self.R), "th": self.th}
        targets = [0]
        if self.m and self.phase not in ("IDLE",):
            targets += [s for s in self.m["ids"] if s != self.id]
        self.bus.send(msg, targets)
        self.tx_n += 1
        _bump(self.tx_w, now)


##############################################################################
# SECTION: main.py   (was agent/main.py)
##############################################################################
def main():
    ap = argparse.ArgumentParser(description="Decentralised swarm agent (one per drone)")
    ap.add_argument("--master", required=True, help="this drone's MAVLink, e.g. tcp:127.0.0.1:5762")
    ap.add_argument("--sysid", type=int, default=0, help="expected sysid (checked against the heartbeat)")
    a = ap.parse_args()

    veh = MavVehicle(a.master)
    if a.sysid and a.sysid != veh.sysid:
        print(f"warning: --sysid {a.sysid} but the autopilot says {veh.sysid}; using {veh.sysid}")
    try:
        bus = UdpBus(veh.sysid)
    except OSError as e:
        raise SystemExit(f"cannot open swarm bus port {BUS_BASE + veh.sysid}: {e} "
                         f"(is an agent for sysid {veh.sysid} already running?)")
    agent = Agent(veh.sysid, veh, bus)
    print(f"[agent {veh.sysid}] online -- MAVLink {a.master}, bus port {BUS_BASE + veh.sysid}")
    try:
        while True:
            t0 = time.time()
            agent.tick()
            time.sleep(max(0.0, 1.0 / CTRL_HZ - (time.time() - t0)))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
