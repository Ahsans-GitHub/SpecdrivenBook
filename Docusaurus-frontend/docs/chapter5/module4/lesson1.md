---
title: "Lesson 1: Humanoid Robot Kinematics and Bipedal Locomotion"
sidebar_label: "Lesson 1: Kinematics & Balance"
tags: [kinematics, locomotion, balance, zmp, lipm, dynamics]
level: [beginner, normal, pro, advanced, research]
description: "The science of walking: Mastering the physics, mathematical models, and control loops required for stable bipedal gait."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Lesson 1: Humanoid Robot Kinematics and Bipedal Locomotion

## 1. The Physics of the Stride

Walking is one of the most complex tasks in robotics. Unlike a wheeled robot, which is inherently stable, a humanoid is an **Inverted Pendulum**. It is constantly in a state of "Controlled Falling." To build a robot that walks, we must understand the dual disciplines of **Kinematics** (the study of motion) and **Dynamics** (the study of the forces that cause motion).

## 2. Humanoid Kinematics: The Tree of Joints

A humanoid like the **Unitree G1** is modeled as a tree of rigid bodies (Links) connected by joints.

### Degrees of Freedom (DoF)
*   **The Bipedal Leg (6 DoF)**: To place a foot perfectly on uneven ground, you need 6 degrees of freedom: Hip (3: Yaw, Pitch, Roll), Knee (1: Pitch), and Ankle (2: Pitch, Roll).
*   **The Humanoid Arm (7 DoF)**: 7 is the "Magic Number" for humanoids. It provides **Redundancy**, allowing the robot to reach a target while keeping its elbow away from an obstacle.

### Forward vs. Inverse Kinematics
1.  **Forward Kinematics (FK)**: "If the hip is at 10° and the knee is at 20°, where is the foot?"
2.  **Inverse Kinematics (IK)**: "I want the foot to be at $(x, y, z)$. What should the joint angles be?"
    *   **The Defensive Fix**: IK often has multiple solutions (different ways to bend). Your code must choose the "Natural" solution that doesn't collide with the other leg.

## 3. The Math of Balance: ZMP and LIPM

To keep the robot from tipping over, we use two fundamental models.

### Zero Moment Point (ZMP)
The ZMP is the point on the floor where the total net force acting on the robot is zero.
*   **The Stability Rule**: As long as the ZMP stays within the **Support Polygon** (the area under the robot's feet), the robot will not fall.
*   **Defensive Practice**: During high-speed walking, the support polygon changes with every step. Your balance controller must predict where the next foot will land to ensure the ZMP never leaves the safe zone.

### Linear Inverted Pendulum Model (LIPM)
We simplify the 27-joint humanoid into a single "Point Mass" on a weightless stick.
*   **Benefit**: This simplifies the complex dynamics into linear equations that can be solved in milliseconds on a Jetson Orin.
*   **The Trap**: LIPM assumes the floor is perfectly flat. For stairs, we must use more complex models like the **Divergent Component of Motion (DCM)**.

## 4. Practical Scenario: The PD Balance Loop

Let's look at a defensive implementation of a balance controller.

```python
# DEFENSIVE: Humanoid Balance Loop (Pseudo-code)
def update_balance(current_imu_roll, current_imu_pitch):
    # 1. PARANOIA: Validate IMU health
    if imu_has_failed():
        trigger_emergency_crouch()
        return

    # 2. DEFENSIVE: Check for 'Fall state'
    # If tilt > 45 degrees, the robot has already lost balance.
    # Trying to correct now will only break the motors.
    if abs(current_imu_roll) > 0.78 or abs(current_imu_pitch) > 0.78:
        self.get_logger().fatal("TILT LIMIT EXCEEDED - CUTTING POWER")
        isolate_actuators()
        return

    # 3. PD Control Calculation
    target_pitch = 0.0 # Vertical
    error = target_pitch - current_imu_pitch
    
    # DEFENSIVE: Integral Windup Protection
    # We avoid the 'I' term in bipedal balance to prevent oscillations
    torque = (Kp * error) + (Kd * derivative_error)
    
    # 4. Final Output Clamping
    safe_torque = clamp(torque, -MAX_MOTOR_TORQUE, MAX_MOTOR_TORQUE)
    apply_to_ankles(safe_torque)
```

## 5. Critical Edge Cases: Singularities

A **Singularity** occurs when two joint axes align (e.g., a fully straight leg).
*   **The Problem**: The math for IK requires dividing by the sine of the angle. If the angle is 0, the motor command becomes "Infinite."
*   **The Defensive Fix**: Implement **Virtual Software Stops**. Never let your joints reach 100% of their physical limit. Stop at 95% to ensure the math stays stable.

## 6. Analytical Research: Capture Points

Researchers focus on the **Capture Point**—the exact spot on the ground where the robot should step to come to a complete stop.
*   **Research Problem**: Predicting the Capture Point on slippery terrain (ice).
*   **Solution**: Using **Reinforcement Learning** to update the LIPM model in real-time based on the amount of "Slip" detected by the foot sensors.

## 7. Multi-Level Summary

### [Beginner]
Walking is "Falling and catching yourself." To keep the robot upright, you must keep its "Shadow" (Center of Mass) between its feet. If the shadow moves outside the feet, the robot falls.

### [Pro/Expert]
Humanoid control is a high-frequency task. We use **Model Predictive Control (MPC)** to look 5 steps into the future and calculate the most efficient path for the ZMP.

### [Researcher]
We are moving toward **Whole-Body Control (WBC)**. Instead of just balancing the legs, the robot uses its arms and head to create "Counter-moments" that help it recover from violent disturbances.

## 8. Conclusion

Balance is the foundation of the humanoid existence. If the robot cannot balance, it cannot manipulate, speak, or assist. Next, we give the robot "Hands" to interact with the world it has learned to navigate.

---

**Next Lesson**: [Lesson 2: Manipulation and Grasping + Natural HRI](./lesson2)
