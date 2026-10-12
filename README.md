# 🚀 Decentralized UAV Swarm GCS | Multi-Agent Drone Coordination

**Individual R&D Project — Rakesh Roy**  
B.Tech ECE | UAV Systems • Robotics • Autonomous Systems • Embedded Systems

---

## 📌 Project Overview

This project focuses on developing a **decentralized multi-UAV swarm coordination system** integrating simulated UAVs, individual swarm agents, inter-agent coordination, and MAVLink-based communication.

The system uses **ROS 2 Humble, Gazebo, ArduPilot SITL, and MAVLink** to provide an environment for experimenting with multi-drone coordination, formation management, and distributed control.

The project explores how individual UAV agents can coordinate their reference states, maintain assigned formation slots, synchronize heading behaviour, and respond to changes in formation geometry.

The objective is to develop a modular platform for researching decentralized multi-agent systems, cooperative UAV navigation, and autonomous swarm behaviour.

---

## 🎯 Core Objectives

- Develop a custom Ground Control Station (GCS) for monitoring multiple UAVs.
- Integrate multiple simulated UAVs using ArduPilot SITL and MAVLink.
- Develop individual swarm agents with coordination capabilities.
- Assign and maintain formation slots.
- Coordinate reference movement, heading, and waypoint progression.
- Monitor formation errors and adjust reference speed.
- Account for formation rotation during heading changes.
- Evaluate swarm behaviour in a multi-UAV simulation environment.
- Establish a foundation for future decentralized autonomy research.

---

## 🖥️ Project Demonstration

### 1. Ground Control Station (GCS)

<p align="center">
  <img src="Media/Gcs.png" width="900" alt="Decentralized UAV Swarm Ground Control Station">
</p>

**Ground Control Station — Monitoring and Simulation Management**

The custom GCS provides an interface for monitoring and managing the multi-UAV simulation. It serves as the main interface for observing swarm activity and interacting with the simulation environment.

The GCS and swarm agents form separate components of the overall system, with their communication and responsibilities determined by the implemented software architecture.

---

### 2. Decentralized Swarm Agent Interface

<p align="center">
  <img src="Media/7_Drones.png" width="900" alt="Multi-UAV Swarm Agent Interface">
</p>

**Individual Agents — Coordination and Formation Management**

The swarm-agent component focuses on coordinating multiple UAVs using individual agent logic.

Each agent works with its vehicle state and assigned formation position. The coordination mechanisms explore reference-state updates, heading alignment, formation-error correction, and coordinated movement.

The objective is to maintain the desired formation while the swarm moves and changes direction.

---

### 3. Multi-UAV Simulation in Gazebo

<p align="center">
  <a href="https://youtu.be/cAJDP-TjUQA">
    <img src="https://img.youtube.com/vi/cAJDP-TjUQA/maxresdefault.jpg"
         width="900"
         alt="Multi-UAV Swarm Simulation in Gazebo">
  </a>
</p>

<p align="center">
  **▶️ [Watch the Full Multi-UAV Swarm Simulation on YouTube](https://youtu.be/cAJDP-TjUQA)**
</p>

**Simulation — Multi-Agent Coordination in Action**

The simulation demonstrates the multi-UAV environment using Gazebo and ArduPilot SITL, with MAVLink supporting vehicle telemetry and command communication.

It provides a test environment for observing vehicle movement, formation behaviour, and interactions between the simulation and swarm-agent software.

This simulation serves as a foundation for evaluating and improving the coordination mechanisms used by the swarm agents.

---

## 🧠 Swarm Agent Algorithms and Coordination Methods

The swarm agents use a sequence of assignment, coordination, and control methods. The following describes the approaches used in the current agent logic; these should not be confused with more advanced algorithms planned for future development.

### 1. Formation-Slot Assignment — Permutation-Based Minimum-Cost Search

The first stage assigns UAVs to formation slots.

The assignment process evaluates possible UAV-to-slot mappings using a cost based on the distance between a UAV and a candidate slot. The objective is to find an assignment with a low total cost.

A simplified assignment objective is:

\[
J = \sum_{i=1}^{N} d_{i,\pi(i)}
\]

Where:

- \(N\) is the number of UAVs.
- \(d_{i,j}\) is the assignment cost between UAV \(i\) and slot \(j\).
- \(\pi(i)\) identifies the slot assigned to UAV \(i\).
- \(J\) is the total assignment cost.

The permutation-based search evaluates candidate assignments to identify a low-cost mapping.

**Important:** This is a permutation-based assignment approach, not the Hungarian algorithm, unless the implementation is subsequently replaced with that algorithm.

### 2. Neighbour-Based Reference Coordination

After assignment, agents use neighbouring-agent information to coordinate their reference states.

A simplified neighbour-averaging operation is:

\[
\bar{x}_i =
\frac{1}{|\mathcal{N}_i|}
\sum_{j \in \mathcal{N}_i} x_j
\]

Where:

- \(x_j\) is the reference state of neighbour \(j\).
- \(\mathcal{N}_i\) is the neighbour set of agent \(i\).
- \(\bar{x}_i\) is the average reference state of the neighbours.

The averaged information can support reference coordination and waypoint progression.

This should be understood as neighbour-based averaging, not necessarily a complete graph-Laplacian consensus algorithm.

### 3. Circular-Mean Heading Coordination

Heading angles require special treatment because they wrap around at \(360^\circ\).

For example, the arithmetic average of \(359^\circ\) and \(1^\circ\) is \(180^\circ\), even though the headings are almost identical.

The circular mean avoids this problem by averaging the sine and cosine components:

\[
\bar{c} = \frac{1}{N}\sum_{i=1}^{N}\cos(\psi_i)
\]

\[
\bar{s} = \frac{1}{N}\sum_{i=1}^{N}\sin(\psi_i)
\]

The mean heading is then calculated as:

\[
\bar{\psi} = \operatorname{atan2}(\bar{s},\bar{c})
\]

Where:

- \(\psi_i\) is the heading of agent \(i\).
- \(\bar{c}\) and \(\bar{s}\) are the averaged cosine and sine components.
- \(\bar{\psi}\) is the resulting circular-mean heading.

This method provides a more meaningful average heading when angles lie on opposite sides of the \(0^\circ/360^\circ\) boundary.

### 4. Formation-Error-Based Reference-Speed Scaling

The system uses formation error to adjust the reference movement speed.

A formation-error measure can be represented as:

\[
e_f = \|\mathbf{p}_i-\mathbf{p}_{i,\mathrm{desired}}\|
\]

Where:

- \(\mathbf{p}_i\) is the position of UAV \(i\).
- \(\mathbf{p}_{i,\mathrm{desired}}\) is its desired formation position.
- \(e_f\) is the positional formation error.

The reference speed is adjusted using a heuristic based on the formation error.

Conceptually:

\[
v_{\mathrm{ref}} = f(e_f)
\]

The function \(f\) represents the implemented speed-scaling rule. Its exact equation depends on the code.

The purpose is to regulate reference movement according to formation behaviour rather than maintaining a fixed reference speed under every condition.

### 5. Turn-Rate Coordination and Formation-Rotation Compensation

When a formation changes heading, its slots rotate around the formation reference point.

For a rigid formation rotating with angular velocity \(\boldsymbol{\omega}\), the rotational velocity contribution of a slot is:

\[
\mathbf{v}_{\mathrm{rot},i}
=
\boldsymbol{\omega}
\times
\mathbf{r}_i
\]

Where:

- \(\boldsymbol{\omega}\) is the formation's angular-velocity vector.
- \(\mathbf{r}_i\) is the slot position relative to the formation reference point.
- \(\mathbf{v}_{\mathrm{rot},i}\) is the rotational velocity contribution.

This relationship explains the kinematic compensation needed when formation slots rotate.

The current implementation should be described as turn-rate coordination and formation-turn compensation to the extent that these terms are represented in the actual agent logic.

The equation above expresses the underlying kinematic relationship; it does not, by itself, establish that the full vector equation is implemented in the code.

### 6. PI-Like Formation-Slot Control

The agent logic also uses a PI-like correction approach to reduce the error between a UAV's actual and desired formation positions.

A conventional PI controller has the form:

\[
u(t) = K_p e(t) + K_i \int_0^t e(\tau)\,d\tau
\]

Where:

- \(e(t)\) is the formation-slot error.
- \(K_p\) is the proportional gain.
- \(K_i\) is the integral gain.
- \(u(t)\) is the resulting control correction.

The proportional component responds to the current error, while the integral component accumulates error over time.

This equation describes a conventional PI controller. The exact controller structure and gains should be taken from the implementation rather than assumed from the term "PI-like."

---

## 🔄 Coordination Pipeline

The overall process can be understood as the following sequence:

```text
        ┌─────────────────────────────┐
        │ 1. Formation-Slot Assignment│
        │ Permutation-Based Search    │
        └──────────────┬──────────────┘
                       │
                       ▼
        ┌─────────────────────────────┐
        │ 2. Neighbour-Based Reference│
        │ Coordination                │
        └──────────────┬──────────────┘
                       │
                       ▼
        ┌─────────────────────────────┐
        │ 3. Circular-Mean Heading    │
        │ Coordination                │
        └──────────────┬──────────────┘
                       │
                       ▼
        ┌─────────────────────────────┐
        │ 4. Formation-Error-Based    │
        │ Reference-Speed Scaling     │
        └──────────────┬──────────────┘
                       │
                       ▼
        ┌─────────────────────────────┐
        │ 5. Turn-Rate Coordination   │
        │ and Rotation Compensation   │
        └──────────────┬──────────────┘
                       │
                       ▼
        ┌─────────────────────────────┐
        │ 6. Formation-Slot Correction│
        │ PI-Like Control              │
        └──────────────┬──────────────┘
                       │
                       ▼
        ┌─────────────────────────────┐
        │ Updated Agent References    │
        │ and Control Outputs         │
        └─────────────────────────────┘
```

> **Note:** This diagram represents the conceptual sequence of the coordination methods, not necessarily the exact runtime call order. The actual execution order depends on the implementation.

---

## 🧩 System Architecture

The project combines the GCS, swarm-agent software, ArduPilot SITL vehicles, MAVLink communication, and Gazebo simulation.

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
       │ Agent Logic    │          │ Agent Logic    │
       └────────────────┘          └────────────────┘
               │                           │
               └─────────────┬─────────────┘
                             │
                    Additional Agents
                             │
                     Gazebo Simulation
```

> **Architecture note:** The diagram is conceptual. Ensure that direct agent-to-agent links and GCS connections match the communication paths actually implemented in your project.

---

## 🛠️ Technology Stack

| Technology | Purpose |
|---|---|
| ROS 2 Humble | Robotics middleware |
| Gazebo | UAV simulation environment |
| ArduPilot SITL | Software-in-the-loop flight simulation |
| MAVLink | UAV telemetry and command communication |
| Python | Agent logic and supporting software |
| UDP | Network transport for communication between software components |

---

## 🚧 Development Status

This is an ongoing individual R&D project.

The current development focuses on multi-UAV simulation, formation management, swarm-agent coordination, and GCS integration.

Future development may explore:

- More advanced distributed consensus algorithms
- Hungarian-algorithm-based formation assignment
- Improved formation-control strategies
- Inter-UAV collision avoidance
- Cooperative navigation
- More scalable multi-agent architectures
- Integration with physical UAV hardware

These are potential future improvements and should not be interpreted as already implemented features.

---

## 🎥 Project Video

**Multi-UAV Swarm Simulation:**  
https://youtu.be/cAJDP-TjUQA

## 💻 Source Code

**GitHub Repository:**  
https://github.com/Programmer-Rakesh/Decentralized-Swarm_in_UAVs

---

## 👨‍💻 Author

**Rakesh Roy**  
B.Tech — Electronics and Communication Engineering

Interests: UAV Systems • Robotics • Embedded Systems • Autonomous Navigation • Multi-Agent Coordination

---
