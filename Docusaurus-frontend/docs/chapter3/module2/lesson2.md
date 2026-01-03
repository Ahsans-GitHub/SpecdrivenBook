---
id: lesson2
title: Simulating physics gravity and collisions in Gazebo
sidebar_label: URDF and SDF robot description formats
---

# Simulating physics, gravity, and collisions in Gazebo.

## Heading Breakdown
**Simulating physics, gravity, and collisions in Gazebo** drills down into the engine that powers the simulation. **Physics** here refers to the solver (like DART or Bullet) that calculates F=ma for every link in the robot. **Gravity** is the constant force that makes humanoid balance difficult. **Collisions** are the contacts between the robot and the world, or the robot and itself (self-collision). Understanding this is critical for **sim-to-real transfer**; if the physics engine is "spongy" or "jittery," the control gains tuned in simulation will fail on the real robot. Real usage involves tuning the `solver_iterations` parameter to ensure stiff contacts for feet. An example is simulating a **Unitree Go2** falling; if the collision meshes are wrong, the robot might fall *through* the floor. This is key for **upgradable high-DoF humanoids** to verify that new hardware attachments (like a heavier battery) don't destabilize the walking gait.

*(Note: While the sidebar refers to URDF/SDF, this lesson focuses on how those formats interact with the Physics Engine).*

## Training Focus: Contact Dynamics
We focus on **stability**.
*   **Solver Iterations**: More iterations = more stable but slower.
*   **ERP/CFM**: Error Reduction Parameter and Constraint Force Mixing—tuning these is an art form for stable walking.

## Detailed Content
### The Physics Engine
Gazebo supports multiple engines.
*   **DART**: Best for articulated chains (robots).
*   **ODE**: Good general purpose, fast.

### Collision Meshes
Why we don't use the visual mesh for physics.
*   **Performance**: Checking collision on a 100k polygon mesh is too slow.
*   **Primitives**: We use boxes, spheres, and cylinders to approximate the robot's shape.

### Industry Vocab
*   **Restitution**: Bounciness.
*   **Penetration Depth**: How far objects are allowed to overlap before the solver pushes them apart.
*   **Self-Collision**: Checking if the left leg hits the right leg.

### Code Example: Physics Config
```xml
<!-- Defensive Physics Configuration -->
<physics type="dart">
  <max_step_size>0.001</max_step_size> <!-- 1kHz simulation -->
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <dart>
    <solver>
      <solver_type>pgs</solver_type>
    </solver>
    <collision_detector>bullet</collision_detector> <!-- Hybrid approach -->
  </dart>
</physics>
```

## Real-World Use Case: Falling Correctly
We simulate thousands of falls. We verify that when the G1 falls, the knees hit the ground first (as per the collision model), protecting the expensive torso sensors. We adjust the friction of the "plastic" casing material in SDF to match reality.