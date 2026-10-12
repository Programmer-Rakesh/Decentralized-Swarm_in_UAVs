#!/usr/bin/env python3
"""
GAZEBO LIVE MAVLink BRIDGE -- multi-drone (swarm)
-------------------------------------------------
Same idea as 3dmap.py, but the "3D view" is Gazebo Sim instead of Cesium.

It listens to a THIRD SITL MAVLink output, and for every vehicle (MAVLink
sysid) it sees it:
    1. spawns one Iris model in the Gazebo world  (iris_1, iris_2, ...)
    2. keeps that model's pose in sync with the vehicle's live position and
       attitude, so Gazebo shows exactly what the dashboard shows.

Gazebo is only a VISUAL MIRROR here: ArduPilot's own SITL physics still
flies the drones (sim_vehicle.py as you already run it), Gazebo just draws
them.  The mirrored models are static / physics-free, so nothing fights
the pose updates.

PHYSICS: each mirrored drone is a real (non-static) Gazebo model with
revolute rotor joints, so the Gazebo physics engine is active for it.  The
four propellers spin with the live motor outputs (SERVO_OUTPUT_RAW) and stop
when the drone is disarmed.  Gravity is off on the mirrored links (ArduPilot
already computes the flight), so the drone follows the telemetry instead of
falling.

START SITL (note the NEW third --out on port 14553):
    sim_vehicle.py -v ArduCopter --console --map \
        --count 3 --auto-sysid \
        --location CMAC --auto-offset-line 90,10 \
        --out=udp:127.0.0.1:14551 \
        --out=udp:127.0.0.1:14552 \
        --out=udp:127.0.0.1:14553

    14551 -> telemetry_dashboard
    14552 -> 3dmap.py
    14553 -> gazebo_bridge.py   (this file)

THEN either press the "Gazebo" button in the dashboard, or run by hand:
    python3 gazebo_bridge.py

Useful flags:
    --demo 3        no SITL needed: 3 fake drones fly circles in Gazebo
                    (use this to check the Gazebo side works on its own)
    --dry-run       no Gazebo needed: only prints the poses it WOULD send
    --no-launch     don't start "gz sim", attach to one you started yourself
    --connection X  MAVLink source (default udp:127.0.0.1:14553)

Requirements:
    pip install pymavlink
    Gazebo Sim (Garden / Harmonic / Ionic) with the `gz` command.
    Recommended (fast, in-process):  sudo apt install python3-gz-transport13 python3-gz-msgs10
        (Harmonic; Garden = transport12/msgs9, Ionic = transport14/msgs11)
    Without those python bindings the bridge falls back to calling the
    `gz service` command line tool, which works but is choppier.

All files (this script, worlds/, models/) live in ~/Documents/UAV.
"""

import argparse
import importlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

from pymavlink import mavutil

# ---------------------------------------------------------------- CONFIG --
UAV_DIR = os.path.dirname(os.path.abspath(__file__))
WORLD_FILE = os.path.join(UAV_DIR, "worlds", "plain_world.sdf")
MODELS_DIR = os.path.join(UAV_DIR, "models")
WORLD_NAME = "plain_world"                 # must match <world name=...>

MAVLINK_CONNECTION = "udp:127.0.0.1:14553"  # THIRD sim_vehicle --out

# Gazebo world origin == ArduPilot "CMAC" home (sim_vehicle --location CMAC)
ORIGIN_LAT = -35.363261
ORIGIN_LON = 149.165230

POSE_RATE_HZ = 30          # how often poses are pushed into Gazebo
STREAM_RATE_HZ = 20        # rate we ask the autopilots to stream at
IRIS_Z_OFFSET = 0.195      # Iris base_link height when sitting on the ground
WORLD_WAIT_S = 90          # how long to wait for Gazebo to come up

# Same colour code as the dashboard / 3D map: A green, B yellow, C red
DRONE_COLORS = {
    1: (0.24, 0.86, 0.52),
    2: (1.00, 0.83, 0.00),
    3: (1.00, 0.30, 0.30),
}
EXTRA_COLORS = [(0.0, 0.83, 1.0), (1.0, 0.55, 0.0), (0.78, 0.45, 1.0),
                (1.0, 0.31, 0.64), (0.36, 0.55, 1.0), (0.85, 0.85, 0.85)]

M_PER_DEG_LAT = 111319.49

# Propeller spin (visual).  Real Iris rotors turn ~800 rad/s, which at 60 fps
# just looks like a frozen/strobing prop, so the speed is scaled down to a
# value that LOOKS like spinning.  Speed follows the motor PWM (1000..2000).
ROTOR_MAX_RAD_S = 70.0
ROTOR_PUBLISH_HZ = 10      # how often rotor speed commands are sent
EXTRAPOLATE_MAX_S = 0.15   # dead-reckon between telemetry packets (smooth)

# (python transport module, python msgs module) per Gazebo release
PY_BACKENDS = [
    ("gz.transport14", "gz.msgs11"),   # Ionic
    ("gz.transport13", "gz.msgs10"),   # Harmonic
    ("gz.transport12", "gz.msgs9"),    # Garden
]
# ---------------------------------------------------------------------------


def log(*a):
    print("[Gazebo]", *a, flush=True)


# ============================================================ MATH =========
def matrix_to_quat(m):
    """3x3 rotation matrix -> (x, y, z, w)."""
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    return (x / n, y / n, z / n, w / n)


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def ardupilot_att_to_gz_quat(roll, pitch, yaw):
    """ArduPilot ATTITUDE (NED world, FRD body, radians) -> Gazebo
    quaternion (ENU world, FLU body) as (x, y, z, w)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    r_ned_frd = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]
    t_enu_ned = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
    t_frd_flu = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
    return matrix_to_quat(_matmul(_matmul(t_enu_ned, r_ned_frd), t_frd_flu))


def latlon_to_xy(lat, lon):
    """Flat-earth metres east/north of the world origin."""
    x = (lon - ORIGIN_LON) * M_PER_DEG_LAT * math.cos(math.radians(ORIGIN_LAT))
    y = (lat - ORIGIN_LAT) * M_PER_DEG_LAT
    return x, y


def look_at_quat(px, py, pz, tx, ty, tz):
    """Gazebo camera orientation (x,y,z,w) looking from P towards T."""
    dx, dy, dz = tx - px, ty - py, tz - pz
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)  # roll = 0


def color_for(sysid):
    if sysid in DRONE_COLORS:
        return DRONE_COLORS[sysid]
    return EXTRA_COLORS[(sysid - 4) % len(EXTRA_COLORS)]


# ==================================================== IRIS MODEL BUILDER ===
def _resource_dirs():
    dirs = []
    for var in ("GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH"):
        dirs += [p for p in os.environ.get(var, "").split(os.pathsep) if p]
    home = os.path.expanduser("~")
    dirs += [
        MODELS_DIR,
        os.path.join(home, "ardupilot_gazebo", "models"),
        os.path.join(home, "gz_ws", "src", "ardupilot_gazebo", "models"),
        os.path.join(home, "ardupilot", "ardupilot_gazebo", "models"),
        os.path.join(home, ".gz", "models"),
    ]
    return dirs


def find_iris_model_dir():
    """Locate the stock ArduPilot Iris model that is already on this
    machine (the one your iris_with_gimbal is built from)."""
    for d in _resource_dirs():
        for name in ("iris_with_standoffs", "iris"):
            cand = os.path.join(d, name)
            if os.path.isfile(os.path.join(cand, "model.sdf")):
                return cand
    return None


def _tag(e):
    return e.tag.split("}")[-1]


BASE_LINK = "base_link"        # link the rotors / marker are attached to
ROTORS = []                    # [(joint_name, servo_channel_0based, spin_sign)]
DEFAULT_SIGNS = [1, 1, -1, -1]  # Iris: rotor_0/1 one way, rotor_2/3 the other

BASE_INERTIAL = ("<inertial><mass>1.5</mass><inertia><ixx>0.029125</ixx>"
                 "<iyy>0.029125</iyy><izz>0.055225</izz><ixy>0</ixy>"
                 "<ixz>0</ixz><iyz>0</iyz></inertia></inertial>")
# Tiny rotor inertia so spinning a prop never pushes the airframe around.
ROTOR_INERTIAL = ("<inertial><mass>0.005</mass><inertia><ixx>6e-7</ixx>"
                  "<iyy>6e-7</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz>"
                  "<iyz>0</iyz></inertia></inertial>")


def _short(name):
    return (name or "").strip().split("::")[-1]


def rotor_topic(model, joint):
    return f"/model/{model}/joint/{joint}/cmd_vel"


def _read_controls(model_dir):
    """ArduPilotPlugin <control> entries of the stock Iris ->
    {joint_name: (servo_channel, spin_sign)} so each prop turns the way the
    real one does."""
    out = {}
    base = os.path.dirname(model_dir)
    for d in (model_dir, os.path.join(base, "iris_with_ardupilot")):
        f = os.path.join(d, "model.sdf")
        if not os.path.isfile(f):
            continue
        try:
            root = ET.parse(f).getroot()
        except Exception:
            continue
        for ctl in root.iter("control"):
            j = ctl.find("jointName")
            if j is None or not j.text:
                continue
            try:
                ch = int(ctl.get("channel"))
                mult = float((ctl.findtext("multiplier") or "1").strip())
            except (TypeError, ValueError):
                continue
            out[_short(j.text)] = (ch, 1 if mult >= 0 else -1)
        if out:
            break
    return out


def extract_iris_links(model_dir):
    """Reads the REAL iris model.sdf and returns the XML text of its links
    (pose + inertia + visuals, gravity off, no collisions/plugins/sensors)
    and its rotor joints, with mesh URIs rewritten to absolute file:// paths.
    The rotor joints are what lets the propellers spin.  Returns None if the
    model can't be used (then the simple built-in quad is used instead)."""
    global BASE_LINK, ROTORS
    try:
        tree = ET.parse(os.path.join(model_dir, "model.sdf"))
        model = tree.getroot().find("model")
        if model is None:
            return None
        if any(_tag(c) == "include" for c in model):
            return None                      # nested model, too fancy
        base = os.path.basename(model_dir)
        prefix = f"model://{base}/"

        kept = {}
        for link in model.findall("link"):
            keep = [c for c in link if _tag(c) in ("pose", "visual")]
            if not any(_tag(c) == "visual" for c in keep):
                continue
            name = link.get("name", "link")
            new = ET.Element("link", {"name": name})
            for c in keep:
                new.append(c)
            ET.SubElement(new, "gravity").text = "false"
            new.append(ET.fromstring(
                ROTOR_INERTIAL if "rotor" in name.lower() or "prop" in name.lower()
                else BASE_INERTIAL))
            for uri in new.iter("uri"):
                if uri.text and uri.text.strip().startswith(prefix):
                    rel = uri.text.strip()[len(prefix):]
                    uri.text = "file://" + os.path.join(model_dir, rel)
            kept[name] = new
        if not kept:
            return None

        base_name = "base_link" if "base_link" in kept else next(iter(kept))
        joints, rotor_joints, jointed = [], [], set()

        # rotor joints straight from the real model
        for j in model.findall("joint"):
            if j.get("type") not in ("revolute", "continuous"):
                continue
            parent = _short(j.findtext("parent"))
            child = _short(j.findtext("child"))
            if parent not in kept or child not in kept:
                continue
            if not any(k in (child + j.get("name", "")).lower()
                       for k in ("rotor", "prop")):
                continue
            nj = ET.Element("joint", {"name": j.get("name"),
                                      "type": j.get("type")})
            ET.SubElement(nj, "parent").text = parent
            ET.SubElement(nj, "child").text = child
            pz = j.find("pose")
            if pz is not None:
                nj.append(pz)
            ax = j.find("axis")
            if ax is None:
                ax = ET.fromstring("<axis><xyz>0 0 1</xyz></axis>")
            nj.append(ax)
            joints.append(nj)
            rotor_joints.append(j.get("name"))
            jointed.add(child)
            if parent == child or parent not in kept:
                return None

        # rotor links the model gave no joint for -> add a spinning one
        for name in kept:
            if name == base_name or name in jointed:
                continue
            if "rotor" in name.lower() or "prop" in name.lower():
                jn = f"{name}_joint"
                nj = ET.fromstring(
                    f'<joint name="{jn}" type="revolute"><parent>{base_name}'
                    f'</parent><child>{name}</child><axis><xyz>0 0 1</xyz>'
                    f'</axis></joint>')
                joints.append(nj)
                rotor_joints.append(jn)
            else:
                joints.append(ET.fromstring(
                    f'<joint name="{name}_fixed" type="fixed"><parent>'
                    f'{base_name}</parent><child>{name}</child></joint>'))
            jointed.add(name)

        controls = _read_controls(model_dir)
        rotors = []
        for i, jn in enumerate(sorted(rotor_joints)):
            ch, sg = controls.get(jn, (i, DEFAULT_SIGNS[i % 4]))
            rotors.append((jn, ch, sg))
        BASE_LINK, ROTORS = base_name, rotors

        return "\n".join(ET.tostring(e, encoding="unicode")
                         for e in list(kept.values()) + joints)
    except Exception as e:
        log(f"Could not read iris model ({e}); using simple quad model")
        return None


def _visual(name, pose, geom, rgb):
    r, g, b = rgb
    return (f'<visual name="{name}"><pose>{pose}</pose><geometry>{geom}'
            f'</geometry><material><ambient>{r} {g} {b} 1</ambient>'
            f'<diffuse>{r} {g} {b} 1</diffuse></material></visual>')


def use_simple_quad_rotors():
    global BASE_LINK, ROTORS
    BASE_LINK = "body"
    ROTORS = [(f"rotor_{i}_joint", i, 1 if i % 2 == 0 else -1)
              for i in range(4)]


def simple_quad_links(rgb):
    """Fallback drone: hub + 2 arms + skids on the body, plus 4 spinning
    propellers on revolute joints (only used when the real Iris meshes
    can't be found)."""
    dark = (0.15, 0.15, 0.15)
    v = [
        _visual("hub", "0 0 0 0 0 0",
                "<box><size>0.22 0.22 0.07</size></box>", rgb),
        _visual("arm_a", "0 0 0 0 0 0.7854",
                "<box><size>0.62 0.03 0.02</size></box>", dark),
        _visual("arm_b", "0 0 0 0 0 -0.7854",
                "<box><size>0.62 0.03 0.02</size></box>", dark),
        _visual("skid_l", "0 0.12 -0.185 0 0 0",
                "<box><size>0.34 0.015 0.015</size></box>", dark),
        _visual("skid_r", "0 -0.12 -0.185 0 0 0",
                "<box><size>0.34 0.015 0.015</size></box>", dark),
    ]
    xml = ('<link name="body"><gravity>false</gravity>' + BASE_INERTIAL
           + "".join(v) + "</link>")
    for i, (rx, ry) in enumerate([(0.22, 0.22), (-0.22, 0.22),
                                  (-0.22, -0.22), (0.22, -0.22)]):
        xml += (f'<link name="rotor_{i}"><pose>{rx} {ry} 0.03 0 0 0</pose>'
                f'<gravity>false</gravity>{ROTOR_INERTIAL}'
                + _visual("blade", "0 0 0 0 0 0",
                          "<box><size>0.26 0.025 0.006</size></box>",
                          (0.9, 0.5, 0.0))
                + '</link>'
                f'<joint name="rotor_{i}_joint" type="revolute">'
                f'<parent>body</parent><child>rotor_{i}</child>'
                '<axis><xyz>0 0 1</xyz></axis></joint>')
    return xml


IRIS_LINKS_XML = None       # filled once in main()


def build_model_sdf(name, sysid):
    """Real (non-static) Iris with spinning rotor joints.  Gravity is off on
    every link: ArduPilot does the flying, Gazebo's physics engine just runs
    the joints (props)."""
    rgb = color_for(sysid)
    links = IRIS_LINKS_XML if IRIS_LINKS_XML else simple_quad_links(rgb)
    plugins = "".join(
        '<plugin filename="gz-sim-joint-controller-system" '
        'name="gz::sim::systems::JointController">'
        f'<joint_name>{jn}</joint_name>'
        '<use_force_commands>false</use_force_commands>'
        f'<topic>{rotor_topic(name, jn)}</topic></plugin>'
        for jn, _, _ in ROTORS)
    return (f'<?xml version="1.0"?><sdf version="1.9">'
            f'<model name="{name}">'
            f'{links}{plugins}</model></sdf>')


# ========================================================= GAZEBO CLIENTS ==
class GzPy:
    """Talks to Gazebo through the python gz-transport bindings (fast)."""

    def __init__(self, world):
        self.world = world
        self.err = None
        self.node = None
        for t_name, m_name in PY_BACKENDS:
            try:
                tr = importlib.import_module(t_name)
                self.msgs = {
                    "Pose": importlib.import_module(m_name + ".pose_pb2").Pose,
                    "Boolean": importlib.import_module(
                        m_name + ".boolean_pb2").Boolean,
                    "EntityFactory": importlib.import_module(
                        m_name + ".entity_factory_pb2").EntityFactory,
                    "GUICamera": importlib.import_module(
                        m_name + ".gui_camera_pb2").GUICamera,
                }
                try:      # optional: spins the propellers
                    self.msgs["Double"] = importlib.import_module(
                        m_name + ".double_pb2").Double
                except Exception:
                    self.msgs["Double"] = None
                self.pubs = {}
                try:      # optional: lets us move all drones in ONE call
                    self.msgs["Pose_V"] = importlib.import_module(
                        m_name + ".pose_v_pb2").Pose_V
                except Exception:
                    self.msgs["Pose_V"] = None
                self.node = tr.Node()
                self.backend = f"{t_name}/{m_name}"
                return
            except Exception as e:
                self.err = e
        raise ImportError(f"no gz python bindings ({self.err})")

    @staticmethod
    def _fill(p, x, y, z, q):
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.x, p.orientation.y = q[0], q[1]
        p.orientation.z, p.orientation.w = q[2], q[3]

    def _call(self, service, req, req_t, timeout=500):
        try:
            ok, rep = self.node.request(
                service, req, req_t, self.msgs["Boolean"], timeout)
            return bool(ok and rep.data)
        except Exception as e:
            log(f"service {service} failed: {e}")
            return False

    def services(self):
        try:
            return list(self.node.service_list())
        except Exception:
            return []

    def create(self, name, sdf, x, y, z, q):
        req = self.msgs["EntityFactory"]()
        req.sdf = sdf
        req.name = name
        req.allow_renaming = False
        self._fill(req.pose, x, y, z, q)
        return self._call(f"/world/{self.world}/create", req,
                          self.msgs["EntityFactory"], 2000)

    def set_pose(self, name, x, y, z, q):
        req = self.msgs["Pose"]()
        req.name = name
        self._fill(req, x, y, z, q)
        return self._call(f"/world/{self.world}/set_pose", req,
                          self.msgs["Pose"], 300)

    def set_pose_vector(self, items):
        """items = [(name, x, y, z, q), ...] -> one service call."""
        if self.msgs.get("Pose_V") is None:
            return False
        req = self.msgs["Pose_V"]()
        for name, x, y, z, q in items:
            p = req.pose.add()
            p.name = name
            self._fill(p, x, y, z, q)
        return self._call(f"/world/{self.world}/set_pose_vector", req,
                          self.msgs["Pose_V"], 300)

    def set_joint_speed(self, topic, rad_s):
        dbl = self.msgs.get("Double")
        if dbl is None:
            return False
        try:
            pub = self.pubs.get(topic)
            if pub is None:
                pub = self.pubs[topic] = self.node.advertise(topic, dbl)
            m = dbl()
            m.data = float(rad_s)
            return bool(pub.publish(m))
        except Exception as e:
            log(f"joint command {topic} failed: {e}")
            return False

    def move_camera(self, x, y, z, q):
        req = self.msgs["GUICamera"]()
        self._fill(req.pose, x, y, z, q)
        return self._call("/gui/move_to/pose", req,
                          self.msgs["GUICamera"], 1000)


class GzCli:
    """Fallback: shells out to `gz service`.  Works with no python bindings
    but each call spawns a process, so motion is choppier."""

    def __init__(self, world):
        self.world = world
        self.gz = shutil.which("gz")
        if not self.gz:
            raise RuntimeError("`gz` command not found in PATH")
        self.backend = "gz service (command line)"
        self._procs = []

    @staticmethod
    def _pose_txt(x, y, z, q):
        return (f"position {{x: {x} y: {y} z: {z}}} "
                f"orientation {{x: {q[0]} y: {q[1]} z: {q[2]} w: {q[3]}}}")

    def _call(self, service, reqtype, req, timeout_ms):
        try:
            r = subprocess.run(
                [self.gz, "service", "-s", service,
                 "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
                 "--timeout", str(timeout_ms), "--req", req],
                capture_output=True, text=True, timeout=timeout_ms / 1000 + 3)
            return "true" in r.stdout.lower()
        except Exception as e:
            log(f"gz service {service} failed: {e}")
            return False

    def services(self):
        try:
            r = subprocess.run([self.gz, "service", "-l"],
                               capture_output=True, text=True, timeout=6)
            return r.stdout.split()
        except Exception:
            return []

    def create(self, name, sdf, x, y, z, q):
        req = (f'sdf: {json.dumps(sdf)} name: "{name}" allow_renaming: false '
               f'pose {{{self._pose_txt(x, y, z, q)}}}')
        return self._call(f"/world/{self.world}/create",
                          "gz.msgs.EntityFactory", req, 3000)

    def set_pose(self, name, x, y, z, q):
        req = f'name: "{name}" {self._pose_txt(x, y, z, q)}'
        return self._call(f"/world/{self.world}/set_pose",
                          "gz.msgs.Pose", req, 500)

    def set_pose_vector(self, items):
        req = " ".join(
            f'pose {{name: "{n}" {self._pose_txt(x, y, z, q)}}}'
            for n, x, y, z, q in items)
        return self._call(f"/world/{self.world}/set_pose_vector",
                          "gz.msgs.Pose_V", req, 500)

    def set_joint_speed(self, topic, rad_s):
        self._procs = [p for p in self._procs if p.poll() is None]
        if len(self._procs) >= 6:      # don't pile up processes
            return False
        try:
            self._procs.append(subprocess.Popen(
                [self.gz, "topic", "-t", topic, "-m", "gz.msgs.Double",
                 "-p", f"data: {rad_s}"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            return True
        except Exception as e:
            log(f"gz topic {topic} failed: {e}")
            return False

    def move_camera(self, x, y, z, q):
        req = f"pose {{{self._pose_txt(x, y, z, q)}}}"
        return self._call("/gui/move_to/pose", "gz.msgs.GUICamera", req, 1500)


class GzDryRun:
    """No Gazebo at all: just prints what would be sent."""
    backend = "dry-run (printing only)"

    def __init__(self):
        self._last = {}

    def services(self):
        return [f"/world/{WORLD_NAME}/create"]

    def create(self, name, sdf, x, y, z, q):
        log(f"[dry-run] CREATE {name} at ({x:.2f},{y:.2f},{z:.2f}) "
            f"sdf={len(sdf)} bytes")
        ET.fromstring(sdf.split("?>", 1)[-1])   # must be valid XML
        return True

    def set_pose(self, name, x, y, z, q):
        now = time.time()
        if now - self._last.get(name, 0) > 1.0:
            self._last[name] = now
            log(f"[dry-run] {name}: xyz=({x:.2f},{y:.2f},{z:.2f}) "
                f"q=({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f})")
        return True

    def set_pose_vector(self, items):
        for n, x, y, z, q in items:
            self.set_pose(n, x, y, z, q)
        return True

    def set_joint_speed(self, topic, rad_s):
        now = time.time()
        if now - self._last.get(topic, 0) > 1.0:
            self._last[topic] = now
            log(f"[dry-run] {topic} = {rad_s:.1f} rad/s")
        return True

    def move_camera(self, x, y, z, q):
        log(f"[dry-run] camera -> ({x:.1f},{y:.1f},{z:.1f})")
        return True


def make_client(dry_run):
    if dry_run:
        return GzDryRun()
    # apt installs the bindings into the system dist-packages, which a
    # virtualenv python can't see -- add it as a last resort.
    for extra in ("/usr/lib/python3/dist-packages",):
        if os.path.isdir(extra) and extra not in sys.path:
            sys.path.append(extra)
    try:
        c = GzPy(WORLD_NAME)
        log(f"Using python bindings: {c.backend}")
        return c
    except Exception as e:
        log(f"Python gz bindings not available ({e}).")
        log("Falling back to `gz service` CLI (works, but choppier). For "
            "smooth motion:  sudo apt install python3-gz-transport13 "
            "python3-gz-msgs10")
        return GzCli(WORLD_NAME)


# ============================================================ TELEMETRY ====
class DroneState:
    def __init__(self):
        self.lat = None
        self.lon = None
        self.rel_alt = 0.0        # metres above home
        self.roll = self.pitch = 0.0
        self.yaw = 0.0
        self.has_att = False
        self.last_update = 0.0
        self.vn = self.ve = self.vd = 0.0   # m/s north / east / down
        self.t_pos = 0.0                    # when the last position arrived
        self.armed = False
        self.pwm = {}                       # servo channel (0-based) -> PWM


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.connected = False
        self.drones = {}

    def get(self, sysid):
        if sysid not in self.drones:
            self.drones[sysid] = DroneState()
        return self.drones[sysid]


state = State()
running = True


def request_streams(master, sysid):
    """Ask this vehicle for fast position + attitude + motor outputs so
    Gazebo is smooth and the propellers follow the real motors.
    (Travels back through the same UDP link; MAVProxy routes it by sysid.)"""
    interval_us = int(1e6 / STREAM_RATE_HZ)
    for msg_id in (33, 30, 36):    # GLOBAL_POSITION_INT, ATTITUDE, SERVO_OUTPUT_RAW
        master.mav.command_long_send(
            sysid, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, interval_us, 0, 0, 0, 0, 0)


def mavlink_worker(connection):
    log(f"MAVLink: connecting to {connection}")
    while running:
        master = None
        try:
            master = mavutil.mavlink_connection(connection)
            log("MAVLink: waiting for heartbeat...")
            if master.wait_heartbeat(timeout=10) is None:
                raise RuntimeError("heartbeat timeout (is sim_vehicle.py "
                                   "running with --out=udp:127.0.0.1:14553 ?)")
            log("MAVLink: connected")
            with state.lock:
                state.connected = True

            last_req = {}
            while running:
                msg = master.recv_match(
                    type=["GLOBAL_POSITION_INT", "ATTITUDE", "HEARTBEAT",
                          "SERVO_OUTPUT_RAW"],
                    blocking=True, timeout=2)
                if msg is None:
                    continue
                sysid = msg.get_srcSystem()
                if not (1 <= sysid <= 250):
                    continue
                if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                    continue
                t = msg.get_type()

                if t == "HEARTBEAT":
                    if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                        continue      # GCS / gimbal / etc, not a vehicle
                    with state.lock:
                        state.get(sysid).armed = bool(
                            msg.base_mode
                            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    # Ask for fast streams only ONCE per drone (every ACK
                    # shows up in the dashboard MAVLINK log, so repeating
                    # this spammed it). Re-ask only if this drone's
                    # position has gone silent for 20 s.
                    now = time.time()
                    with state.lock:
                        dd = state.drones.get(sysid)
                        silent = (dd is not None and dd.last_update > 0.0
                                  and now - dd.last_update > 20)
                    if sysid not in last_req or (
                            silent and now - last_req[sysid] > 20):
                        last_req[sysid] = now
                        try:
                            request_streams(master, sysid)
                        except Exception as e:
                            log(f"stream request to sysid {sysid} failed: {e}")

                elif t == "GLOBAL_POSITION_INT":
                    lat, lon = msg.lat / 1e7, msg.lon / 1e7
                    if abs(lat) > 90 or abs(lon) > 180 or (lat == 0 and lon == 0):
                        continue
                    with state.lock:
                        d = state.get(sysid)
                        d.lat, d.lon = lat, lon
                        d.rel_alt = max(0.0, msg.relative_alt / 1000.0)
                        d.last_update = time.time()
                        d.t_pos = d.last_update
                        d.vn, d.ve = msg.vx / 100.0, msg.vy / 100.0
                        d.vd = msg.vz / 100.0
                        if not d.has_att and msg.hdg != 65535:
                            d.yaw = math.radians(msg.hdg / 100.0)

                elif t == "SERVO_OUTPUT_RAW":
                    with state.lock:
                        d = state.get(sysid)
                        for ch in range(8):
                            d.pwm[ch] = getattr(msg, f"servo{ch + 1}_raw", 0)

                elif t == "ATTITUDE":
                    with state.lock:
                        d = state.get(sysid)
                        d.roll, d.pitch, d.yaw = msg.roll, msg.pitch, msg.yaw
                        d.has_att = True

        except Exception as e:
            log(f"MAVLink error: {e}")
            with state.lock:
                state.connected = False
            time.sleep(2)
        finally:
            if master is not None:
                try:
                    master.close()
                except Exception:
                    pass


def demo_worker(count):
    """Fake drones flying circles, so the Gazebo half can be tested with
    no SITL at all."""
    log(f"DEMO MODE: {count} fake drones (no MAVLink)")
    with state.lock:
        state.connected = True
    t0 = time.time()
    while running:
        t = time.time() - t0
        with state.lock:
            for i in range(count):
                sysid = i + 1
                d = state.get(sysid)
                ang = t * 0.4 + i * 0.8
                x, y = i * 10.0 + 8 * math.cos(ang), 8 * math.sin(ang)
                d.lat = ORIGIN_LAT + y / M_PER_DEG_LAT
                d.lon = ORIGIN_LON + x / (
                    M_PER_DEG_LAT * math.cos(math.radians(ORIGIN_LAT)))
                d.rel_alt = 6 + 2 * math.sin(t * 0.7 + i)
                d.yaw = math.pi / 2 - (ang + math.pi / 2)   # nose along path
                d.roll = 0.15 * math.sin(t + i)
                d.pitch = 0.1 * math.cos(t + i)
                d.has_att = True
                d.armed = True
                d.last_update = time.time()
        time.sleep(0.05)


# ============================================================ GAZEBO RUN ===
gz_proc = None


def gazebo_env():
    env = os.environ.copy()
    extra = [MODELS_DIR, os.path.join(UAV_DIR, "worlds")]
    for var in ("GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH"):
        cur = env.get(var, "")
        env[var] = os.pathsep.join(extra + ([cur] if cur else []))
    return env


def launch_gazebo():
    global gz_proc
    gz = shutil.which("gz")
    if not gz:
        raise RuntimeError("`gz` command not found. Is Gazebo Sim installed "
                           "and sourced?")
    if not os.path.isfile(WORLD_FILE):
        raise RuntimeError(f"World file missing: {WORLD_FILE}")
    log(f"Starting Gazebo:  gz sim -r {WORLD_FILE}")
    gz_proc = subprocess.Popen(
        [gz, "sim", "-r", WORLD_FILE], env=gazebo_env(),
        start_new_session=(os.name != "nt"))


def stop_gazebo():
    """Only stops a Gazebo that THIS script started."""
    global gz_proc
    p, gz_proc = gz_proc, None
    if p is None or p.poll() is not None:
        return
    log("Stopping Gazebo...")
    try:
        if os.name != "nt":
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        else:
            p.terminate()
        p.wait(timeout=5)
    except Exception:
        try:
            if os.name != "nt":
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            else:
                p.kill()
        except Exception:
            pass


def world_ready(client):
    return f"/world/{WORLD_NAME}/create" in client.services()


def check_world_physics():
    """Warn (never edits the file) if the world has no physics engine, since
    the propeller joints need it to turn."""
    try:
        with open(WORLD_FILE, encoding="utf-8") as f:
            txt = f.read()
    except OSError:
        return
    if "physics-system" not in txt:
        log("WARNING: this world has no physics system, so the propellers "
            "will NOT spin. Add inside <world>:")
        log('    <plugin filename="gz-sim-physics-system" '
            'name="gz::sim::systems::Physics"/>')


def wait_for_world(client):
    log(f"Waiting for world '{WORLD_NAME}' (up to {WORLD_WAIT_S}s)...")
    deadline = time.time() + WORLD_WAIT_S
    while time.time() < deadline and running:
        if gz_proc is not None and gz_proc.poll() is not None:
            raise RuntimeError(f"Gazebo exited early (code {gz_proc.returncode})")
        if world_ready(client):
            log("World is up.")
            return
        time.sleep(1.0)
    raise RuntimeError(
        f"World '{WORLD_NAME}' never appeared. Check the Gazebo window, and "
        "that its world <world name=...> matches WORLD_NAME.")


# ============================================================ MAIN LOOP ====
def compute_pose(d):
    x, y = latlon_to_xy(d.lat, d.lon)
    z = d.rel_alt + IRIS_Z_OFFSET
    if d.t_pos > 0.0:
        # telemetry arrives at ~20 Hz, poses go out at 30 Hz: carry the
        # position forward with the reported velocity so motion is smooth
        # instead of stair-stepping.
        dt = min(max(time.time() - d.t_pos, 0.0), EXTRAPOLATE_MAX_S)
        x += d.ve * dt
        y += d.vn * dt
        z = max(IRIS_Z_OFFSET, z - d.vd * dt)
    return x, y, z, ardupilot_att_to_gz_quat(d.roll, d.pitch, d.yaw)


def rotor_speeds(d):
    """[(joint_name, rad/s)] for one drone from its motor PWM (1000..2000).
    Props stop when disarmed.  No servo data -> a steady mid spin."""
    out = []
    for jn, ch, sign in ROTORS:
        pwm = d.pwm.get(ch) if d.pwm else None
        if not d.armed:
            norm = 0.0
        elif pwm:
            norm = min(1.0, max(0.0, (pwm - 1000.0) / 1000.0))
        else:
            norm = 0.5
        out.append((jn, sign * ROTOR_MAX_RAD_S * norm))
    return out


def frame_camera(client, points):
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    spread = max(max(p[0] for p in points) - min(p[0] for p in points),
                 max(p[1] for p in points) - min(p[1] for p in points))
    dist = max(14.0, spread * 1.2 + 10.0)
    px, py, pz = cx, cy - dist, max(8.0, spread * 0.5 + 6.0)
    return client.move_camera(px, py, pz,
                              look_at_quat(px, py, pz, cx, cy, 2.0))


def pose_loop(client):
    spawned = {}            # sysid -> consecutive set_pose failures
    last_spawn = 0.0
    framed_count = 0
    period = 1.0 / POSE_RATE_HZ
    use_vec = f"/world/{WORLD_NAME}/set_pose_vector" in client.services()
    vec_fail = 0
    rotor_sent = {}         # sysid -> {joint: (last rad/s sent, when)}
    last_rotor_tx = 0.0
    log("Pose updates: " + ("one batched call for all drones"
                            if use_vec else "one call per drone"))

    while running:
        t0 = time.time()
        with state.lock:
            snap = {sid: (d.lat, d.lon, d.rel_alt, d.roll, d.pitch, d.yaw,
                          d.vn, d.ve, d.vd, d.t_pos, d.armed, dict(d.pwm))
                    for sid, d in state.drones.items() if d.lat is not None}

        points = []
        updates = []        # (sysid, name, x, y, z, q) for already-spawned drones
        drones_now = {}
        for sysid in sorted(snap):
            d = DroneState()
            (d.lat, d.lon, d.rel_alt, d.roll, d.pitch, d.yaw,
             d.vn, d.ve, d.vd, d.t_pos, d.armed, d.pwm) = snap[sysid]
            drones_now[sysid] = d
            x, y, z, q = compute_pose(d)
            points.append((x, y))
            name = f"iris_{sysid}"

            if sysid not in spawned:
                ok = client.create(name, build_model_sdf(name, sysid), x, y, z, q)
                # If it "failed" because the model already exists from an
                # earlier run, set_pose below will still work -- so carry on.
                log(f"Spawned {name} (sysid {sysid}) at "
                    f"({x:.1f}, {y:.1f}) {'OK' if ok else '(create returned false)'}")
                spawned[sysid] = 0
                rotor_sent.pop(sysid, None)
                last_spawn = time.time()
                continue

            updates.append((sysid, name, x, y, z, q))

        if updates:
            done = False
            if use_vec:
                done = client.set_pose_vector(
                    [(n, x, y, z, q) for _, n, x, y, z, q in updates])
                if done:
                    vec_fail = 0
                    for sid, *_ in updates:
                        spawned[sid] = 0
                else:
                    vec_fail += 1
                    if vec_fail >= 5:
                        use_vec = False
                        log("Batched pose call not working - using one "
                            "call per drone instead")
            if not done:
                for sid, name, x, y, z, q in updates:
                    if client.set_pose(name, x, y, z, q):
                        spawned[sid] = 0
                    else:
                        spawned[sid] += 1
                        if spawned[sid] >= 20:   # model probably vanished
                            log(f"{name} not responding; will respawn it")
                            del spawned[sid]
                            rotor_sent.pop(sid, None)

        # Propellers: send each rotor's speed (only when it changes, plus a
        # once-a-second refresh so a late-loading model still catches up).
        now = time.time()
        if ROTORS and now - last_rotor_tx >= 1.0 / ROTOR_PUBLISH_HZ:
            last_rotor_tx = now
            for sysid, d in drones_now.items():
                if sysid not in spawned:
                    continue
                name = f"iris_{sysid}"
                sent = rotor_sent.setdefault(sysid, {})
                for jn, spd in rotor_speeds(d):
                    last_v, last_t = sent.get(jn, (None, 0.0))
                    if (last_v is None or abs(spd - last_v) > 0.5
                            or now - last_t > 1.0):
                        if client.set_joint_speed(rotor_topic(name, jn), spd):
                            sent[jn] = (spd, now)

        # After the newest drone has settled in, frame all drones once.
        if (points and len(spawned) != framed_count
                and time.time() - last_spawn > 3.0):
            frame_camera(client, points)
            framed_count = len(spawned)

        if gz_proc is not None and gz_proc.poll() is not None:
            log("Gazebo window was closed - bridge stopping.")
            return

        time.sleep(max(0.0, period - (time.time() - t0)))


def handle_signal(signum, frame):
    global running
    running = False


def main():
    global running, IRIS_LINKS_XML, MAVLINK_CONNECTION, ORIGIN_LAT, ORIGIN_LON

    ap = argparse.ArgumentParser(description="MAVLink -> Gazebo drone mirror")
    ap.add_argument("--connection", default=MAVLINK_CONNECTION)
    ap.add_argument("--origin", help="lat,lon of Gazebo (0,0); "
                                     "default = CMAC home")
    ap.add_argument("--no-launch", action="store_true",
                    help="do not start gz sim; attach to a running one")
    ap.add_argument("--demo", type=int, metavar="N",
                    help="N fake circling drones, no SITL needed")
    ap.add_argument("--dry-run", action="store_true",
                    help="no Gazebo, just print poses")
    args = ap.parse_args()

    MAVLINK_CONNECTION = args.connection
    if args.origin:
        ORIGIN_LAT, ORIGIN_LON = (float(v) for v in args.origin.split(","))

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    print("=" * 60)
    print("GAZEBO LIVE DRONE BRIDGE")
    print("=" * 60)
    print(f"World   : {WORLD_FILE}")
    print(f"MAVLink : {'(demo)' if args.demo else MAVLINK_CONNECTION}")
    print("=" * 60, flush=True)

    # Which drone model will be mirrored?
    iris_dir = find_iris_model_dir()
    if iris_dir:
        IRIS_LINKS_XML = extract_iris_links(iris_dir)
    if IRIS_LINKS_XML:
        log(f"Using real Iris model from {iris_dir}")
    else:
        use_simple_quad_rotors()
        log("Real Iris model not found - using simple quad model. (Put your "
            "ardupilot_gazebo/models folder on GZ_SIM_RESOURCE_PATH to get "
            "the real Iris.)")
    log("Rotors: " + (", ".join(f"{j}(servo{c + 1},{'+' if s > 0 else '-'})"
                                for j, c, s in ROTORS) or "none found"))
    if not args.dry_run:
        check_world_physics()

    try:
        client = make_client(args.dry_run)

        if not args.dry_run:
            if world_ready(client):
                log("Gazebo world already running - attaching to it.")
            elif args.no_launch:
                log("Waiting for a Gazebo world to appear...")
            else:
                launch_gazebo()
            wait_for_world(client)

        if args.demo:
            threading.Thread(target=demo_worker, args=(args.demo,),
                             daemon=True).start()
        else:
            threading.Thread(target=mavlink_worker,
                             args=(MAVLINK_CONNECTION,), daemon=True).start()

        pose_loop(client)
        rc = 0
    except Exception as e:
        log(f"ERROR: {e}")
        rc = 1
    finally:
        running = False
        stop_gazebo()
    sys.exit(rc)


if __name__ == "__main__":
    main()
