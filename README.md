<<<<<<< HEAD
# Swarm GCS — compact layout (4 code files)

    python3 gcs5.py                                                # the ground station (run this)
    python3 swarm_agent.py --master tcp:127.0.0.1:5762 --sysid 1   # one agent (the GCS starts these in AUTO MODE)

Install: `pip install -r requirements.txt`

## Tree
```
swarm_gcs/
├── gcs5.py          WHOLE GCS: MAVLink link, connection bar, layout, map, panels,
│                    telemetry loop, swarm control (bus, mission, RTL, navigation panel), DashboardApp, main()
├── gcs_ui.py        ALL looks: colours, sizes, map-style list, timings, glass panel, icons,
│                    every widget (buttons, gauges, RSSI, battery, check box), button/section helpers
├── gcs_3d.py        3D Map button (starts drone_3d.py -- put drone_3d.py in this same folder)
├── swarm_agent.py   ALL algorithms: formation geometry, formation/transit control, RTL + landing,
│                    UDP bus, MAVLink vehicle, message handlers, Agent, main()   (one copy runs per drone)
├── requirements.txt
└── README.md
```
Every file is divided by banners — search for `# SECTION:` to jump (each banner says which old file it came from).
The GCS imports `formation_slots` from `swarm_agent.py`, so formation geometry now exists in ONE place only.
Keep `swarm_agent.py` next to `gcs5.py` (the GCS spawns it from there).

## "I want to change ..." -> where
| Change | File / section |
|---|---|
| Colours, ports, timings, default paths, map styles | `gcs_ui.py` -> SECTION config.py (top) |
| Button / gauge / panel look | `gcs_ui.py` -> SECTION edge_button, glass_panel, ... , ui_helpers |
| Window layout, side tabs | `gcs5.py` -> SECTION layout.py, side_panels.py |
| Connect button, port list, SIM/TELE | `gcs5.py` -> SECTION connection.py, status_bar.py |
| Map, waypoints, mission path | `gcs5.py` -> SECTION map_panel, map_style, map_overlays, waypoints |
| Takeoff/AUTO, gauges, failsafe, params, log | `gcs5.py` -> SECTION control_panel ... log_panel |
| Telemetry -> UI | `gcs5.py` -> SECTION poll_loop.py |
| Swarm mission / RTL / formation bar / nav panel | `gcs5.py` -> SECTION mission, rtl, formation_bar, nav_* |
| MAVLink send/receive | `gcs5.py` -> SECTION mavlink, mav_commands, mav_mission |
| 3D button | `gcs_3d.py` |
| Formation shapes, control gains, RTL/landing, agent <-> autopilot | `swarm_agent.py` (config at top, then geometry, control, rtl, vehicle) |
=======
# 🚀 Decentralized UAV Swarm GCS | Multi-Agent Drone Coordination

**Individual R&D Project — Rakesh Roy**  
B.Tech ECE | UAV Systems • Robotics • Autonomous Systems • Embedded Systems

---

## 📌 Project Overview

This project focuses on developing a **Agent for decentralized multi-UAV swarm coordination**, integrating simulated UAVs, individual swarm agents, inter-agent coordination, and MAVLink-based communication.

The system is being developed using **ROS 2, Gazebo, ArduPilot SITL, and MAVLink**, providing a simulation environment for testing multi-drone coordination and formation control.

Unlike a conventional ground control interface that primarily focuses on individual vehicle monitoring and control, this project explores how multiple UAV agents can coordinate their reference states, maintain formation slots, and respond to changes in formation geometry through distributed coordination mechanisms.

The long-term objective is to develop a modular platform for experimenting with decentralized multi-agent systems, cooperative UAV navigation, and autonomous swarm behaviour.

---

## 🎯 Core Objectives

- Develop a custom GCS for monitoring and managing multiple UAV agents.
- Integrate multiple simulated UAVs using ArduPilot SITL and MAVLink.
- Implement decentralized coordination between individual swarm agents.
- Maintain assigned formation slots while the formation moves.
- Coordinate heading, reference movement, and waypoint progression.
- Implement formation-error monitoring and adaptive reference-speed scaling.
- Account for rotational movement when a formation changes heading.
- Build a simulation environment for observing and testing swarm behaviour.
- Establish a modular foundation for future autonomy and multi-agent research.

---

## 🖥️ Project Visuals

### 1. Ground Control Station (GCS)

<!-- Replace with your GCS interface image -->

<p align="center">
  <img src="Media/gcs_interface.png" width="900" alt="Decentralized UAV Swarm Ground Control Station">
</p>

<p align="center">
  <i>Custom GCS interface for swarm monitoring and simulation management.</i>
</p>

### 2. Decentralized Swarm Agent Interface

<!-- Replace with your swarm agent interface image -->

<p align="center">
  <img src="Media/swarm_agent_interface.png" width="900" alt="Decentralized UAV Swarm Agent Interface">
</p>

<p align="center">
  <i>Swarm agent interface for observing individual agent states and coordination behaviour.</i>
</p>

### 3. Multi-UAV Simulation in Gazebo

<!-- Replace with a GitHub-supported video link or an animated GIF -->

<p align="center">
  <a href="Media/swarm_simulation.mp4">
    <img src="Media/swarm_simulation_thumbnail.png" width="900" alt="UAV Swarm Simulation in Gazebo">
  </a>
</p>

<p align="center">
  <i>Multi-UAV simulation using Gazebo, ArduPilot SITL, and MAVLink.</i>
</p>

> **Note:** GitHub does not reliably display MP4 files embedded directly through an HTML `<video>` element in a README. For a reliable presentation, upload the video to YouTube or GitHub Releases and link it here. Alternatively, use a GIF preview that links to the full video.

---

## 🧩 System Architecture

The project combines the simulation environment, vehicle communication layer, GCS, and individual swarm agents.

### High-Level Architecture

```text
                 ┌─────────────────────────────┐
                 │     Ground Control Station  │
                 │            (GCS)            │
                 └──────────────┬──────────────┘
                                │
                         MAVLink / UDP
                                │
                ┌───────────────┴───────────────┐
                │                               │
        ┌───────▼────────┐              ┌───────▼────────┐
        │  ArduPilot SITL│              │  ArduPilot SITL│
        │    Vehicle 1   │              │    Vehicle 2   │
        └───────┬────────┘              └───────┬────────┘
                │                               │
        ┌───────▼────────┐              ┌───────▼────────┐
        │  Swarm Agent 1 │◄────────────►│  Swarm Agent 2 │
        │ Coordination   │  Peer Data   │ Coordination   │
        └────────────────┘              └────────────────┘
                │                               │
                └───────────────┬───────────────┘
                                │
                     Additional UAV Agents
                                │
                       Gazebo Simulation
>>>>>>> df823946bd4e6a0e78b0edfe428cbd4596b7f3dd
