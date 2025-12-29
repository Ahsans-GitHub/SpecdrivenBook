---
title: "Lesson 1: Gazebo Simulation Environment Setup"
sidebar_label: "Lesson 1: Gazebo Setup"
tags: [ros2, gazebo, simulation, setup, harmonic, fortress]
level: [beginner, normal, pro, advanced, research]
description: "Architecture of the Gazebo simulator, its integration with ROS 2, and setting up a robust humanoid testing environment."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Lesson 1: Gazebo Simulation Environment Setup

## 1. The Gazebo Ecosystem: Ignition vs. Harmonic

To build Physical AI, you need a playground. **Gazebo** is that playground. It is a multi-robot simulator for outdoor and indoor environments. It combines a high-performance physics engine with a rendering engine and a robust communication layer.

### The Evolution: A Note on Versions
For years, the community used **Gazebo Classic** (versions 9 and 11). However, the project has transitioned to a modular architecture originally called **Ignition Gazebo**, now simply called **Gazebo Harmonic/Fortress**.
*   **Fortress**: The standard for ROS 2 Humble.
*   **Harmonic**: The standard for ROS 2 Jazzy (2024+).
*   **Key Change**: The new Gazebo is "Plugin-first." Almost every feature (LIDAR, Gravity, UI) is a separate library that you load only when needed. This makes it faster and more stable for humanoid simulation.

## 2. Architecture: How ROS 2 talks to Gazebo

Gazebo and ROS 2 are separate programs. They speak different languages. To make them talk, we use the **ros_gz_bridge**.

1.  **Gazebo Transport**: A high-speed communication bus inside the simulator.
2.  **ROS 2 Topics**: The communication bus we learned in Module 1.
3.  **The Bridge**: A translator node that maps Gazebo topics (e.g., `/world/test/clock`) to ROS 2 topics (e.g., `/clock`).

### Defensive Bridge Configuration
A common failure in simulation is "Time Drift." 
*   **The Trap**: Your AI thinks it's 12:00:00 (system time), but the simulator paused for a millisecond, so it thinks it's 11:59:59.
*   **The Defensive Fix**: Always bridge the `/clock` topic first. Set the ROS parameter `use_sim_time: True` in every node you launch. This forces your AI to march to the beat of the simulator's drum.

## 3. Practical Scenario: The "Empty World" Setup

Let's configure a basic testing world for a **Unitree G1**.

### Installation (Defensive Cross-Platform)
*   **Linux (Native)**: `sudo apt install ros-humble-ros-gz`
*   **Windows (WSL2)**: Ensure you have an X-Server (like GWSL) or are using WSLg for GUI support.
*   **Mac**: Use a Docker container with `vnc` support, as native Gazebo performance on M-series chips is currently non-optimal for high-DoF humanoids.

### Creating your first `.sdf` World
SDF (Simulation Description Format) is an XML format used to describe the world.

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="humanoid_lab">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- DEFENSIVE: Always include a sun and ground -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>

    <!-- Simulation Plugins -->
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics" />
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands" />
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster" />
  </world>
</sdf>
```

## 4. Critical Edge Cases: The Physics Step

In Module 1, we talked about "Real-Time." In simulation, we have **Step Size**.
*   **Max Step Size**: The amount of virtual time that passes in one physics calculation (e.g., 0.001s).
*   **Real Time Factor (RTF)**: The ratio of sim-time to real-time. RTF=1.0 means 1 second in sim takes 1 second in reality.

### The "Kraken" Bug (Numerical Instability)
Humanoid robots have many complex joints. If your `max_step_size` is too large (e.g., 0.01), the forces calculated in one step will be so large they "teleport" the robot's leg through the floor.
*   **Defensive Tip**: For humanoid balance loops, set `max_step_size` to **0.001 (1ms)** or even **0.0005 (0.5ms)**. If the simulation is too slow, don't increase the step size; instead, simplify your collision geometry.

## 5. Analytical Research: Solver Stability

For research-level simulation, we must choose a **Physics Engine**.
1.  **DART (Dynamic Animation and Robotics Toolkit)**: The default for new Gazebo. Optimized for "Articulated Bodies" (Humanoids). Very stable but can be slow.
2.  **Bullet**: Good for rigid body collisions and many objects.
3.  **ODE (Open Dynamics Engine)**: The legacy default. Fast but prone to "jitter" in complex joint chains.

**Research Question**: How does the choice of solver impact the convergence of an RL walking policy for the Unitree G1?
*   *Observation*: Policies trained in DART transfer to the real world with 20% higher success rates than those trained in ODE, due to more accurate contact modeling.

## 6. Defensive Programming Checklist
*   [ ] Did you verify the `ros_gz_bridge` is running before launching your nodes?
*   [ ] Is `use_sim_time` set to `True`?
*   [ ] Is your GPU driver up to date (Gazebo Harmonic requires OGRE 2.x)?
*   [ ] Did you check the terminal for "Physics Solver Error" warnings?

---

**Summary**: Your simulation environment is the foundation of your Physical AI safety. A well-configured Gazebo world prevents hardware damage and provides the ground-truth data needed to debug your "Brain." Next, we define the "Body" of our robot using URDF.

**Next Lesson**: [Lesson 2: URDF and SDF Robot Description Formats](./lesson2)
