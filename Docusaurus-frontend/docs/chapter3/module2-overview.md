---
id: module2-overview
title: Module 2 The Digital Twin (Gazebo & Unity)
sidebar_label: Module 2 Overview
---

# Module 2: The Digital Twin (Gazebo & Unity) -> Weeks 6-7: Robot Simulation with Gazebo

## Module Heading Breakdown
**The Digital Twin (Gazebo & Unity)** defines the essential virtual replica framework needed for modern robotics. **The** implies a singular, authoritative copy of the physical system. **Digital** refers to the computational nature of the model, residing in bits rather than atoms. **Twin** establishes the concept of synchronization—the virtual robot should behave exactly like the real one. This is a physics-based environment for robot prototyping, vital for **closing sim-to-real gaps** in humanoid dynamics. Real usage involves running collision detection on a Jetson-simulated **Unitree Go2** before turning on the motors. An example is a Gazebo plugin using `gz::physics::SetGravity` to simulate Mars gravity for space robotics. This module is key for training **upgradable high-DoF systems**, allowing us to test modular sensor plugins (like a new depth camera) virtually. **Gazebo** is the open-source engine for high-fidelity physics, launched via `gz sim -v 4 worlds/robot.sdf`. **Unity** adds a layer of high-fidelity visualization, using ray-tracing (`UnityEngine.Physics.Raycast`) to simulate how light interacts with optical sensors, crucial for testing vision algorithms.

## What We Gonna Learn
Learners will delve into **Gazebo's simulation environment setup** for creating virtual labs, understanding **URDF and SDF formats** for robot modeling. We will move beyond simple boxes and cylinders to importing complex CAD meshes of humanoid parts. You will learn to **simulate physics and sensors** for realistic interactions, modeling how a rubber foot slips on a tiled floor versus a carpet. We will integrate **Unity for high-fidelity visualization**, bridging digital twins with physical humanoid behaviors. This includes setting up a **ROS-Unity Bridge** to stream joint angles from ROS 2 into Unity, allowing you to see your robot move in a photorealistic VR environment.

## Highlights and Key Concepts
*   **Highlight**: **URDF as Unified Robot Description Format**. This is the standard for modular joint definitions. We will build a URDF from scratch, defining parent-child relationships and rotation limits.
*   **Key Concept**: **SDF (Simulation Description Format)**. While URDF describes the robot, SDF describes the world. We will use SDF to incorporate collision meshes and inertial properties (mass matrix) for accurate sensor emulation.
*   **Highlight**: **Physics Engines (Dart, Bullet, ODE)**. We will compare how different solvers handle contact dynamics. For a walking robot, a "stiff" solver is needed to prevent the feet from sinking into the ground.
*   **Key Concept**: **Visual vs. Collision Geometry**. Learning to use high-poly meshes for looking good (Visual) and simple primitives for calculating physics fast (Collision).

## Summaries of Outcomes
*   **Part 1**: Students will gain expertise in creating custom Gazebo worlds, populating them with obstacles and actors.
*   **Part 2**: Students will master the conversion of CAD models (SolidWorks/Fusion360) into URDF/SDF formats.
*   **Part 3**: Outcomes include robust sensor simulation, enabling accurate modeling of LiDAR and IMUs for humanoid perception, with outcomes like robust collision handling.
*   **Part 4**: Learners will achieve proficiency in Unity integration, enabling adaptive sensor simulations for edge-case testing in humanoid environments.

## Adaption to Real Robots (Unitree G1/Go2 & Jetson)
*   **Scenario**: **Adapt Gazebo simulations to real Jetson hardware**. We will verify that the camera resolution and frame rate in simulation match the specs of the real Realsense camera on the Unitree Go2.
*   **Calibrating URDF**. We will perform experiments to measure the "real" friction of the G1's feet and update the URDF `<friction>` tags to match, ensuring the sim walk behaves like the real walk.
*   **Hardware-in-the-Loop (HIL)**. We will run the physics on a powerful desktop but run the ROS 2 control nodes on the actual Jetson, connected via Ethernet, to test the compute limits of the embedded hardware.

## Learning Outcomes
*   **Outcome**: Proficiency in **Unity integration** for HRI visualization, enabling you to test how humans react to robot gestures in VR.
*   **Outcome**: Mastery of **inertial modeling**, understanding how errors in the Center of Mass (CoM) definition can cause a real robot to fall.
*   **Outcome**: Ability to write **Gazebo Plugins** in C++ to simulate custom actuators or environmental forces (like wind).
*   **Outcome**: Creation of a **validated Digital Twin**, ready for RL training.

## Different Scenarios
*   **Simulated**: **Multi-physics collisions**. We will simulate a humanoid falling down stairs to test the durability of the virtual chassis and the response of the IMU.
*   **Real**: **URDF calibration on Go2**. We use a force gauge to measure joint torque limits and update the URDF `<effort>` tags.
*   **Edge Cases**: **Overload in Unity**. We test what happens when the VR scene is too complex, causing frame drops, and how to optimize assets for real-time HRI.
*   **Upgradable**: **SDF extensions**. We create a "mounting point" system in our robot description so we can easily attach different end-effectors (hands, grippers) in simulation.

## Industry Vocab & Code Snippets
*   **Vocab**: "Xacro" (XML Macros), "Inertia Tensor" (rotational mass), "Mesh Collider" (complex hit detection).
*   **Integration Example**:
    ```xml
    <!-- Defensive URDF Joint Definition -->
    <joint name="knee_joint" type="revolute">
      <parent link="thigh"/>
      <child link="shin"/>
      <origin xyz="0 0 -0.4" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <!-- Safety Controller: Soft limits to prevent hardware damage -->
      <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.0" soft_upper_limit="0.0"/>
      <limit lower="-2.1" upper="0.0" effort="100" velocity="5.0"/>
    </joint>
    ```