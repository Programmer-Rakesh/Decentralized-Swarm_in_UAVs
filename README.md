## Decentralized UAV Swarm GCS | Multi-Agent Drone Coordination

<img src="Media/image1.png" align="right" width="42%" alt="UAV Swarm">

**Started:** 15 September 2026

**Goal:** Coordinate multiple simulated UAVs through individual swarm agents and a custom Ground Control Station (GCS).

### Built with

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-FF6F00?style=for-the-badge)
![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-0A5FFF?style=for-the-badge)
![MAVLink](https://img.shields.io/badge/MAVLink-Protocol-2EA043?style=for-the-badge)

<br clear="right">

## 📌 Overview

A multi-UAV simulation project exploring **decentralized coordination, formation management, and cooperative movement**. It combines ROS 2 Humble, Gazebo, ArduPilot SITL, MAVLink, and a custom GCS.

Each swarm agent manages its own simulated UAV and exchanges state information with other agents. The GCS is used to configure missions and monitor the swarm.

---

##  Simulation Pipeline

The system connects the GCS, swarm agents, ArduPilot SITL, and Gazebo.

<p align="center">
  <img src="Media/simulation_pipeline.svg" alt="UAV Swarm Simulation Pipeline" width="850">
</p>

### How it works

1. **Configure mission:** Select UAVs and set formation, spacing, altitude, speed, and route in the GCS.
2. **Distribute mission:** The GCS sends mission data to available agents through the custom UDP swarm bus.
3. **Coordinate agents:** Agents exchange peer-state information over UDP and update their movement references.
4. **Control vehicles:** Each agent uses Pymavlink and MAVLink to send commands to its own ArduPilot SITL instance and receive telemetry.
5. **Simulate flight:** ArduPilot SITL interacts with Gazebo, which simulates vehicle movement and the environment.
6. **Monitor swarm:** Agent states and vehicle telemetry are used by the GCS to display swarm status.

### Communication summary

| Link | Technology | Purpose |
|---|---|---|
| GCS ↔ swarm agents | Custom UDP swarm bus | Mission distribution and status |
| Agent ↔ agent | UDP | Peer-state exchange and coordination |
| Agent ↔ ArduPilot SITL | MAVLink via Pymavlink | Vehicle commands and telemetry |
| ArduPilot SITL ↔ Gazebo | SITL–Gazebo integration | Simulated vehicle and state exchange |
| GCS ↔ ArduPilot SITL | MAVLink | Direct telemetry and supported GCS commands |

> **Implementation note:** The described code uses a custom UDP bus and Pymavlink; it does not use MAVROS. The exact SITL–Gazebo transport depends on the simulation configuration.

---

##  Project Demonstration

### 1. Ground Control Station

<p align="center">
  <img src="Media/Gcs.png" width="900" alt="Decentralized UAV Swarm Ground Control Station">
</p>

The custom GCS provides mission configuration, swarm monitoring, and simulation management.

### 2. Swarm Agent Interface

<p align="center">
  <img src="Media/7_Drones.png" width="900" alt="Multi-UAV Swarm Agent Interface">
</p>

Individual agents manage their vehicle state and formation references, supporting coordinated movement and formation-error correction.

### 3. Multi-UAV Simulation in Gazebo

<p align="center">
  <a href="https://youtu.be/cAJDP-TjUQA">
    <img src="https://img.youtube.com/vi/cAJDP-TjUQA/maxresdefault.jpg" width="900" alt="Multi-UAV Swarm Simulation in Gazebo">
  </a>
</p>

<p align="center">
  **▶️ [Watch the full simulation on YouTube](https://youtu.be/cAJDP-TjUQA)**
</p>

---

##  Swarm Coordination Methods

The current project explores the following methods. Their exact behaviour depends on the agent implementation.

| Method | Role |
|---|---|
| **Permutation-based formation-slot assignment** | Evaluates possible UAV-to-slot assignments and selects a low-cost assignment. |
| **Neighbour-based reference coordination** | Uses information from neighbouring agents to update movement references. |
| **Circular-mean heading coordination** | Averages heading angles correctly around the 0°/360° boundary. |
| **Formation-error-based speed scaling** | Adjusts reference speed in response to formation-position error. |
| **Turn-rate and rotation compensation** | Accounts for formation rotation while the swarm changes direction. |
| **PI-like formation-slot control** | Uses position error to generate corrective movement adjustments. |

### 📚 Research References

The coordination methods explored in this project are related to established research in distributed control, multi-agent consensus, and formation assignment.

1. **Distributed UAV Formation Control**  
   Xue, R., Song, J., & Cai, G. (2016). *Distributed formation flight control of multi-UAV system with nonuniform time-delays and jointly connected topologies.*  
   [Read paper (DOI)](https://doi.org/10.1177/0954410015619446)  
   **Relevance:** Neighbour-to-neighbour information exchange and distributed formation coordination.

2. **Consensus-Based Formation Control**  
   Seo, J., et al. (2012). *Consensus-based reconfigurable controller design for unmanned aerial vehicle formation flight.*  
   [Read paper (DOI)](https://doi.org/10.1177/0954410011415157)  
   **Relevance:** Distributed coordination using local information to maintain a desired formation.

3. **Decentralized Formation and Collision Avoidance**  
   Kuriki, Y., & Namerikawa, T. (2015). *Formation Control with Collision Avoidance for a Multi-UAV System Using Decentralized MPC and Consensus-Based Control.*  
   [Read paper (DOI)](https://doi.org/10.9746/jcmsi.8.285)  
   **Relevance:** Background for decentralized formation control and potential future collision-avoidance extensions.

4. **Optimal Assignment Methods**  
   Kuhn, H. W. (1955). *The Hungarian Method for the Assignment Problem.*  
   [Read paper (DOI)](https://doi.org/10.1002/nav.3800020109)  
   **Relevance:** Foundational work on minimum-cost assignment. The Hungarian method is a possible future alternative to the project's current permutation-based slot assignment.

> **Scope:** These publications provide related theoretical foundations and comparison points. They should not be interpreted as evidence that the project implements each paper's complete algorithm or reproduces its reported results.

### Formation error analysis

<p align="center">
  <img src="Media/left.png" width="49%" alt="Formation Error Before Correction">
  <img src="Media/right.png" width="49%" alt="Formation Error After Correction">
</p>

<p align="center">
  <b>Before correction</b>
  &emsp;&emsp;&emsp;&emsp;
  <b>After correction</b>
</p>

The graphs compare formation error before and after correction.

### Coordination sequence

```text
Formation-slot assignment
          ↓
Neighbour-based reference coordination
          ↓
Circular-mean heading coordination
          ↓
Formation-error-based speed scaling
          ↓
Turn-rate / rotation compensation
          ↓
Formation-slot correction
          ↓
Updated agent references and control outputs
```

> **Note:** This is a conceptual sequence; the runtime call order may differ.

---

## 🧩 System Architecture

```text
                 ┌──────────────────────────┐
                 │  Ground Control Station  │
                 │          (GCS)           │
                 └────────────┬─────────────┘
                              │
                         MAVLink / UDP
                              │
                ┌─────────────┴─────────────┐
                │                           │
        ┌───────▼────────┐          ┌───────▼────────┐
        │ ArduPilot SITL │          │ ArduPilot SITL │
        │   Vehicle 1    │          │   Vehicle 2    │
        └───────┬────────┘          └───────┬────────┘
                │                           │
        ┌───────▼────────┐          ┌───────▼────────┐
        │  Swarm Agent 1 │◄────────►│  Swarm Agent 2 │
        │  Agent Logic   │          │  Agent Logic   │
        └────────────────┘          └────────────────┘
                │                           │
                └─────────────┬─────────────┘
                              │
                       Additional Agents

                    Gazebo Simulation
```

> **Architecture note:** This is a conceptual overview. Refer to the communication table above for the links described by the current implementation.

---

## 🛠️ Technology Stack

| Technology | Purpose |
|---|---|
| ROS 2 Humble | Robotics middleware |
| Gazebo | UAV simulation environment |
| ArduPilot SITL | Software-in-the-loop flight simulation |
| MAVLink / Pymavlink | Vehicle commands and telemetry |
| Python | Agent logic and supporting software |
| UDP | Communication between software components |

---

## 🔄 Swarm Communication Pipeline

<p align="center">
  <img src="Media/swarm_communication_pipeline.svg" alt="Decentralized UAV Swarm Communication Pipeline" width="100%">
</p>

---

## 🚧 Development Status

This is an ongoing individual R&D project focused on multi-UAV simulation, formation management, agent coordination, and GCS integration.

**Potential future work**
- Hungarian-algorithm-based formation assignment
- Improved distributed coordination and formation control
- Inter-UAV collision avoidance and cooperative navigation
- More scalable multi-agent architectures
- Integration with physical UAV hardware

These are planned exploration areas, not claims of currently implemented features.

---

## 🎥 Project Video

[▶️ Multi-UAV Swarm Simulation](https://youtu.be/cAJDP-TjUQA)

> **Source code:** The repository's source code is not currently public while development continues. Selected implementation details are omitted to protect project intellectual property.
'''

out = Path("/mnt/data/README_refined.md")
out.write_text(readme, encoding="utf-8")
print(f"Created {out} ({len(readme.splitlines())} lines)")
