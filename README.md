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
