---
id: lesson1
title: Focus Physics simulation and environment building
sidebar_label: Gazebo simulation environment setup
---

# Focus: Physics simulation and environment building.

## Heading Breakdown
**Focus: Physics simulation and environment building** directs our attention to the creation of the virtual world. **Physics simulation** is the mathematical approximation of the laws of nature—gravity, friction, momentum—that governs how objects move and interact. **Environment building** is the construction of the stage upon which the robot acts—the walls, floors, tables, and terrains. The importance is absolute for validation; a robot that walks on a flat plane in a void has not been tested. It must walk on slippery floors, uneven grass, and cluttered rooms. Real usage involves designing a "Digital Twin" of a factory floor to test if the robot fits in the aisles. An example is using **SDF (Simulation Description Format)** to define a world with a specific sun angle and shadow softness. This is key for **upgradable high-DoF humanoids** because it allows us to test "what if" scenarios (e.g., "What if the robot carries a 10kg box?") without risking the hardware.

## Training Focus: Environmental Fidelity
We focus on **context**. A robot exists in a context.
*   **Static vs. Dynamic**: Walls don't move; people do. We learn to model both.
*   **Material Properties**: Friction coefficients are critical for walking robots.

## Detailed Content
### The Gazebo World
A `.sdf` file that defines the scene.
*   **Physics Engines**: ODE (Open Dynamics Engine), Bullet, Dart, Simbody.
*   **Lighting**: Ambient, Point, and Directional lights.
*   **GUI**: Using the Gazebo visual editor to drag-and-drop assets.

### Industry Vocab
*   **SDF**: Simulation Description Format.
*   **Mesh**: The 3D geometry.
*   **Collision Box**: A simplified shape used for physics calculations to save CPU.

### Code Example: World SDF
```xml
<!-- Defensive World Definition -->
<sdf version="1.6">
  <world name="default">
    <!-- Global Light Source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- Ground Plane with specific friction -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface>
            <friction>
              <ode><mu>100</mu><mu2>50</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Real-World Use Case: Warehouse Sim
For the G1, we build a warehouse world with concrete floors (low friction) and rubber mats (high friction). We test the walking controller's ability to transition between these surfaces without slipping.