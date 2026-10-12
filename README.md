# 🚀 Decentralized UAV Swarm GCS | Multi-Agent Drone Coordination

**Individual R&D Project — Rakesh Roy**  
B.Tech ECE | UAV Systems • Robotics • Autonomous Systems • Embedded Systems

---

## 📌 Project Overview

This project focuses on developing a **decentralized multi-UAV swarm coordination system**, integrating simulated UAVs, individual swarm agents, inter-agent coordination, and MAVLink-based communication.

The system is being developed using **ROS 2, Gazebo, ArduPilot SITL, and MAVLink**, providing a simulation environment for experimenting with multi-drone coordination and formation control.

Unlike a conventional ground control station (GCS) that primarily focuses on individual vehicle monitoring and control, this project explores how multiple UAV agents can coordinate their reference states, maintain assigned formation slots, and respond to changes in formation geometry through distributed coordination mechanisms.

The long-term objective is to develop a modular platform for researching decentralized multi-agent systems, cooperative UAV navigation, and autonomous swarm behaviour.

---

## 🎯 Core Objectives

- Develop a custom GCS for monitoring and managing multiple UAV agents.
- Integrate multiple simulated UAVs using ArduPilot SITL and MAVLink.
- Implement decentralized coordination between individual swarm agents.
- Maintain assigned formation slots while the formation moves.
- Coordinate heading, reference movement, and waypoint progression.
- Monitor formation errors and implement adaptive reference-speed scaling.
- Account for rotational movement during formation heading changes.
- Build a simulation environment for observing and testing swarm behaviour.
- Establish a modular foundation for future autonomy and multi-agent research.

---

## 🖥️ Project Visuals

### 1. Ground Control Station (GCS)

<p align="center">
  <img src="Media/Gcs.png" width="900" alt="Decentralized UAV Swarm Ground Control Station">
</p>

<p align="center">
  <i>Custom GCS interface for swarm monitoring and simulation management.</i>
</p>

### 2. Decentralized Swarm Agent Interface

<p align="center">
  <img src="Media/7_Drones.png" width="900" alt="Decentralized UAV Swarm Agent Interface">
</p>

<p align="center">
  <i>Swarm agent interface for observing individual UAV states and coordination behaviour.</i>
</p>

### 3. Multi-UAV Simulation in Gazebo

<p align="center">
  <a href="Media/Drone.mp4">
    ▶️ Watch the Multi-UAV Swarm Simulation
  </a>
</p>

<p align="center">
  <i>Multi-UAV simulation using Gazebo, ArduPilot SITL, and MAVLink.</i>
</p>

> **Note:** The video is stored in the repository. Click the link above to open it on GitHub. For inline playback directly within the README, consider using a GitHub-supported video attachment URL.

---

## 🧩 System Architecture

The system integrates the simulation environment, vehicle communication layer, Ground Control Station, and individual swarm agents.

### High-Level Architecture

```text
                  ┌─────────────────────────────┐
                  │     Ground Control Station  │
                  │             (GCS)           │
                  └──────────────┬──────────────┘
                                 │
                          MAVLink / UDP
                                 │
                  ┌──────────────┴──────────────┐
                  │                             │
          ┌───────▼────────┐            ┌───────▼────────┐
          │ ArduPilot SITL │            │ ArduPilot SITL │
          │   Vehicle 1    │            │   Vehicle 2    │
          └───────┬────────┘            └───────┬────────┘
                  │                             │
          ┌───────▼────────┐            ┌───────▼────────┐
          │  Swarm Agent 1 │◄──────────►│  Swarm Agent 2 │
          │  Coordination  │ Peer Data  │  Coordination  │
          └────────────────┘            └────────────────┘
                  │                             │
                  └──────────────┬──────────────┘
                                 │
                      Additional UAV Agents
                                 │
                       Gazebo Simulation
```

---

## 🛠️ Technology Stack

- **ROS 2 Humble** — Robotics middleware
- **Gazebo** — UAV simulation environment
- **ArduPilot SITL** — Software-in-the-loop flight simulation
- **MAVLink** — UAV telemetry and command communication
- **Python** — Agent logic, coordination, and supporting tools

---

## 🚧 Development Status

This is an ongoing individual R&D project.

Current development focuses on multi-UAV simulation, swarm-agent coordination, formation management, and GCS integration. The system will continue to evolve as coordination mechanisms, control strategies, and autonomous behaviours are tested and refined.

Future improvements may include more advanced distributed coordination algorithms, improved formation control, collision avoidance, and cooperative navigation.

---
