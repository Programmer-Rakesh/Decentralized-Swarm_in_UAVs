# 🚀 Decentralized UAV Swarm GCS | Multi-Agent Drone Coordination

**Individual R&D Project — Rakesh Roy**  
B.Tech ECE | UAV Systems • Robotics • Autonomous Systems • Embedded Systems

---

## 📌 Project Overview

This project focuses on developing a **decentralized multi-UAV swarm coordination system**, integrating simulated UAVs, individual swarm agents, inter-agent coordination, and MAVLink-based communication.

The system is being developed using **ROS 2, Gazebo, ArduPilot SITL, and MAVLink**, providing a simulation environment for experimenting with multi-drone coordination and formation control.

Unlike a conventional Ground Control Station (GCS) that primarily focuses on individual vehicle monitoring and control, this project explores how multiple UAV agents can coordinate their reference states, maintain assigned formation slots, and respond to changes in formation geometry through distributed coordination mechanisms.

The long-term objective is to develop a modular platform for experimenting with decentralized multi-agent systems, cooperative UAV navigation, and autonomous swarm behaviour.

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
  <a href="https://youtu.be/cAJDP-TjUQA">
    <img src="https://img.youtube.com/vi/cAJDP-TjUQA/maxresdefault.jpg"
         width="900"
         alt="Watch the Decentralized UAV Swarm Simulation">
  </a>
</p>

<p align="center">
  <b>▶️ <a href="https://youtu.be/cAJDP-TjUQA">Watch the Multi-UAV Swarm Simulation on YouTube</a></b>
</p>

<p align="center">
  <i>Multi-UAV simulation using Gazebo, ArduPilot SITL, and MAVLink.</i>
</p>

---

## 🧩 System Architecture

The system integrates the Ground Control Station, simulated UAVs, individual swarm agents, and the simulation environment.

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

> **Note:** This is a conceptual high-level architecture. Adjust the communication arrows to match the actual connections implemented in your system.

---

## ⚙️ Coordination and Control

The project explores multiple mechanisms for multi-UAV coordination, including:

- **Formation-slot assignment:** Assigning UAVs to formation positions.
- **Reference-state coordination:** Coordinating movement references between agents.
- **Heading coordination:** Maintaining coordinated heading behaviour.
- **Formation-error monitoring:** Evaluating deviations from assigned formation positions.
- **Adaptive reference-speed scaling:** Adjusting reference movement according to formation error.
- **Turn coordination:** Accounting for formation rotation during heading changes.
- **Waypoint progression:** Coordinating movement through predefined reference points.

These mechanisms are being developed and evaluated as part of the ongoing R&D work.

---

## 🛠️ Technology Stack

| Technology | Purpose |
|---|---|
| ROS 2 Humble | Robotics middleware |
| Gazebo | Simulation environment |
| ArduPilot SITL | Software-in-the-loop flight simulation |
| MAVLink | UAV telemetry and command communication |
| Python | Agent logic and supporting tools |
| UDP | Network communication between software components |

---

## 🚧 Development Status

This is an ongoing individual R&D project focused on decentralized multi-UAV coordination and simulation.

Current development focuses on swarm-agent coordination, formation management, simulation integration, and GCS functionality.

Future development may explore:

- Advanced distributed coordination algorithms
- Improved formation-control strategies
- Inter-UAV collision avoidance
- Cooperative navigation
- Scalable multi-agent simulation
- Integration with real UAV hardware

---

## 🎥 Project Demonstration

**Watch the simulation:**  
https://youtu.be/cAJDP-TjUQA

---

## 👨‍💻 Author

**Rakesh Roy**  
B.Tech — Electronics and Communication Engineering (ECE)

Interested in UAV systems, robotics, embedded systems, autonomous navigation, and multi-agent coordination.

---
