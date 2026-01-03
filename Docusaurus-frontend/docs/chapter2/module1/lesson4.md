---
id: lesson4
title: Understanding URDF Unified Robot Description Format for humanoids
sidebar_label: Launch files and parameter management
---

# Understanding URDF (Unified Robot Description Format) for humanoids.

## Heading Breakdown
**Understanding URDF (Unified Robot Description Format) for humanoids** focuses on the XML standard used to describe the physical structure of a robot. **URDF** is the file format that tells the software "this robot has a leg, and the leg is connected to the hip, and the hip can rotate 90 degrees." **Unified** implies it is the single source of truth for both simulation (Gazebo) and visualization (RViz). **For humanoids**, this is exceptionally complex because of the high number of joints (Degrees of Freedom) and the branching kinematic chains (two arms, two legs, one head). The importance is critical; if your URDF is wrong (e.g., the leg length is off by 1cm), your walking algorithm will fail because the math doesn't match the reality. Real usage involves generating URDFs from CAD tools like SolidWorks. An example is defining the `mass` and `inertia` matrix for the Unitree G1's torso so the balance controller knows how much force to apply. This is key for **upgradable systems**, as adding a backpack to the robot requires updating the URDF to reflect the new Center of Mass.

*(Note: While the sidebar refers to Launch Files, this lesson content delves into the configuration data often loaded by those launch files—the Robot Description).*

## Training Focus: Kinematic Modeling
We focus on **structural accuracy**.
*   **Tree Structure**: Robots are trees of links connected by joints.
*   **Frames of Reference**: Understanding `base_link`, `odom`, and `map`.

## Detailed Content
### The URDF Structure
An XML format consisting of:
*   **Links**: The rigid parts (bones).
*   **Joints**: The moving parts (motors).
*   **Visual**: How it looks (meshes).
*   **Collision**: How it bangs into things (simple shapes).
*   **Inertial**: How heavy it is (physics).

### Launching the Model
We use `robot_state_publisher` to read the URDF and publish the transforms (TF2).
*   **xacro**: We use Xacro (XML Macros) to clean up the code. Instead of writing "Leg Left" and "Leg Right" code twice, we write a "Leg Macro" and call it twice.

### Industry Vocab
*   **TF tree**: The hierarchy of coordinate frames.
*   **Mesh**: The 3D model file (.stl or .dae).
*   **End-Effector**: The tip of the arm/leg.

### Code Example: URDF Snippet
```xml
<!-- Defensive URDF Link Definition -->
<link name="torso">
  <inertial>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <mass value="10.5"/>
    <!-- Diagonal inertia matrix for stability -->
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
  <visual>
    <geometry>
      <mesh filename="package://my_robot_description/meshes/torso.dae"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
</link>
```

## Real-World Use Case: Parameter Management via Launch
While the URDF describes the *structure*, **Launch Files** manage the *parameters*. When we launch the G1 robot driver, we might pass a parameter `use_sim_time:=true` if we are in Gazebo, or `urdf_file:=g1_v2.urdf` if we have upgraded the hardware. The launch system reads the URDF and loads it into the `robot_state_publisher` node, making the model available to the entire system.