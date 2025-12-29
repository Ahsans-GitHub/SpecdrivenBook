---
title: "Assessment 2: Digital Twin & Simulation Synthesis"
sidebar_label: "2. Simulation Synthesis"
tags: [simulation, gazebo, unity, urdf, dynamics]
level: [beginner, normal, pro, advanced, research]
description: "Validating structural fidelity and physics stability in humanoid digital twins."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Assessment 2: Digital Twin & Simulation Synthesis

## 1. Challenge Overview
You are tasked with validating the **Digital Twin** of a **Unitree G1**. The simulation is currently unstable—the robot occasionally "jitter-walks" and the LIDAR sensor reports ghost obstacles. You must perform a structural audit and physics tuning.

## 2. Core Evaluation Criteria

### Structural Audit (>2500 words depth)
*   **The Problem**: The robot's mass properties in the URDF are currently estimated, not measured.
*   **Task**: Describe the process of **System Identification**. How do you verify the `inertia` tensor of the G1's lower leg?
*   **Defensive Rule**: Every `Link` in your URDF must have non-zero mass and inertia. Explain why a `0.0` mass link causes the ODE physics solver to return `NaN` and crash the world.

### Physics Engine Tuning
*   **The Problem**: The G1 walks perfectly on flat ground in sim but falls immediately on carpet in the real world.
*   **Task**: Design an **Ablation Study** for Friction. 
    1.  Test the gait at $\mu = 0.3, 0.5, 0.8$.
    2.  Identify the "Critical Friction Point" where the balance controller fails.
*   **Solution**: Implement **Domain Randomization** in your launch files to vary friction dynamically during training.

## 3. Practical Task: Fixing the "Kraken"

Identify the error in this snippet of a world file:
```xml
<physics type="ode">
  <max_step_size>0.01</max_step_size> <!-- ERROR DETECTED -->
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```
*   **The Analysis**: A `max_step_size` of `0.01` (10ms) is too coarse for a high-frequency humanoid balance loop. This will cause joint forces to over-correct, leading to the "Kraken" effect.
*   **Fix**: Set `max_step_size` to `0.001` or `0.0005`.

## 4. Analytical Research: High-Fidelity HRI in Unity

Explain how you would use Unity to simulate a human "pushing" the robot.
*   **Research Question**: How does **Network Jitter** between the ROS controller and the Unity frontend impact the perceived stability of the robot's digital twin?

## 5. Summary
Simulation is your primary defense against hardware breakage. Mastery of this module means creating virtual worlds that are "harder" than the real world, ensuring your robot is over-prepared for reality.
