---
title: "Lesson 2: URDF and SDF Robot Description Formats"
sidebar_label: "Lesson 2: Robot Descriptions"
tags: [urdf, sdf, xacro, robot-modeling, kinematics]
level: [beginner, normal, pro, advanced, research]
description: "Defining the skeleton, visual identity, and physical mass of a humanoid robot using standard XML formats."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Lesson 2: URDF and SDF Robot Description Formats

## 1. The Blueprint of a Robot

Before a ROS 2 node can control a motor, or a simulator can calculate gravity, the computer must understand the robot's **Structure**. How long are the legs? Where are the joints? How much does the head weigh? 

In robotics, we use XML-based formats to describe this "Blueprint." There are two industry standards you must master: **URDF** and **SDF**.

## 2. URDF: The Tree of Life

**URDF (Unified Robot Description Format)** is the core standard for ROS 2. It describes a robot as a **Tree of Links** connected by **Joints**.

### Anatomy of a URDF Link
A `link` represents a single physical part (e.g., the "forearm"). It has three distinct layers:
1.  **Visual**: What it looks like. Usually a `.dae` (Collada) or `.stl` mesh.
2.  **Collision**: The "Invisible Box" around the link used for physics. 
    *   **Defensive Rule**: Never use high-poly visual meshes for collision. It will slow down the simulator by 10x. Use simple boxes, cylinders, and spheres.
3.  **Inertial**: The mass ($m$) and the rotational inertia tensor ($I$).
    *   **The Trap**: If you leave mass at 0.0, the physics engine will divide by zero and your robot will vanish from the universe.

### Anatomy of a Joint
A `joint` defines how two links move relative to each other.
*   **Fixed**: Part A is bolted to Part B.
*   **Revolute**: Rotation around an axis with limits (e.g., an Elbow).
*   **Prismatic**: Linear sliding movement.
*   **Continuous**: Rotation without limits (e.g., a Wheel).

## 3. Xacro: Making XML Smart

Writing a URDF for a **Unitree G1** (with 27+ joints) would require thousands of lines of repetitive XML. We use **Xacro** (XML Macros).
*   **Constants**: `MAX_EFFORT = 20.0`
*   **Math**: `${pi / 2}`
*   **Macros**: Define a "Finger" once, and call it 10 times with different offsets.

### Practical Defensive Xacro Example

```xml
<xacro:macro name="g1_joint" params="name parent child origin_xyz origin_rpy min_limit max_limit">
  <joint name="${name}" type="revolute">
    <parent link="${parent}"/>
    <child link="${child}"/>
    <origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
    <axis xyz="0 0 1"/>
    <!-- DEFENSIVE: Explicitly define hardware safety limits -->
    <limit lower="${min_limit}" upper="${max_limit}" effort="30.0" velocity="5.0"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>
</xacro:macro>
```

## 4. SDF: The World Standard

While URDF is great for kinematic chains, **SDF (Simulation Description Format)** is the Gazebo standard.
*   **Why use SDF?**: It can describe complex environmental properties (lighting, sky, friction) and parallel linkages (which URDF cannot).
*   **Integration**: Usually, you write in URDF (for ROS 2) and the simulator converts it to SDF on the fly.

## 5. Critical Edge Cases: The "Pretzel" Robot

A common beginner mistake is misconfiguring the **Rotation Axis** or the **Joint Limits**.
*   **The Problem**: You tell the robot to "Bend Knee," but because the axis is wrong, it rotates the leg through the torso.
*   **The Defensive Fix**: Use the `joint_state_publisher_gui` tool. Move every slider manually before writing a single line of AI code. Verify that "Positive Command" moves the joint in the "Natural Direction."

## 6. Analytical Research: Inertia Identification

For research-level accuracy (Sim-to-Real), estimated mass is not enough.
*   **The Gap**: Most CAD models estimate inertia as if the robot were a solid block of aluminum. In reality, cables, wires, and hollow joints create a different "Center of Gravity."
*   **Research Solution**: **System Identification (SysID)**. We move the physical G1 in a specific trajectory, measure the motor current (torque), and use an optimization algorithm to "back-calculate" the real inertia values for the URDF.

## 7. Multi-Level Summary

### [Beginner]
URDF is like a "Lego Instruction Manual" for the computer. It tells the robot where its parts are. Links are the parts, and Joints are the hinges. Always remember: if you don't give a part a "Collision" box, it will pass through walls like a ghost.

### [Pro/Expert]
Humanoid modeling is about **Dynamic Fidelity**. We must carefully model the **Damping** and **Friction** of each joint in Xacro to ensure that our PID controllers don't oscillate when the robot is standing still.

### [Researcher]
We are moving toward **Neural Robot Descriptions**. Instead of XML files, we are training "Implicit Kinematic Models" that can learn the robot's structure purely from visual observation, handling cases where the robot is damaged or modified.

## 8. Conclusion

Your URDF is the "Source of Truth" for your robot. Every algorithm we write in Chapter 4 and 5 depends on the accuracy of this file. In the next lesson, we bring this blueprint to life with **Physics and Sensors**.

---

**Next Lesson**: [Lesson 3: Physics Simulation and Sensor Simulation](./lesson3)
