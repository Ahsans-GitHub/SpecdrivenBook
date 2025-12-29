---
title: "Lesson 2: Manipulation and Grasping + Natural HRI"
sidebar_label: "Lesson 2: Manipulation & HRI"
tags: [manipulation, grasping, hri, moveit2, compliant-control, humanoid-hands]
level: [beginner, normal, pro, advanced, research]
description: "Mastering the art of humanoid manipulation: From motion planning to safe, natural interaction with humans."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Lesson 2: Manipulation and Grasping + Natural HRI

## 1. The Manipulation Challenge: High-Stakes Interactions

In Chapter 4, we learned how to move a robot through a room. In this lesson, we learn how to **Change the Room**. Manipulation is the "Action" in Vision-Language-Action. It is the ability to use the **Unitree G1**'s 7-DoF arms to pick up a tool, open a door, or hand a bottle to a human.

Manipulation is uniquely difficult because it involves **Physical Contact**. While walking only involves contact with the floor, manipulation involves contact with delicate, unpredictable, and expensive objects. This requires a transition from "Position Control" to **Compliance and Force Control**.

## 2. The Manipulation Pipeline

To pick up an object, the robot must execute a sequence of four complex stages:

1.  **Target Perception**: Use Isaac ROS to find the object's 6D Pose ($x, y, z$ + Roll, Pitch, Yaw).
2.  **Motion Planning**: Use **MoveIt 2** to calculate a path for the arm that doesn't hit the robot's own body or the environment.
3.  **Grasp Selection**: Identify the "Handles" of the object. Should the robot use a "Power Grasp" (whole hand) or a "Pinch Grasp" (fingertips)?
4.  **Force Execution**: Close the fingers until the **Force/Torque sensors** indicate a secure hold without crushing the object.

## 3. Humanoid Hand Morphology and Synergies

A human hand has 27 degrees of freedom. A robot hand usually has between 2 and 15.
*   **The Problem**: Controlling 15 joints for every grasp is computationally overwhelming for an LLM brain.
*   **The Solution: Synergies**. Most human grasps follow a few "Master Shapes" (e.g., closing all fingers at once). We program these shapes as "Grasp Primitives" in ROS 2, so the AI can simply say: `CLOSE_GRIPPER(strength=0.5)`.

## 4. Natural Human-Robot Interaction (HRI)

Robots don't work in isolation; they work with people. **Natural HRI** is the design of interfaces that feel human-like and safe.

### Key HRI Pillars
1.  **Proxemics**: Respecting personal space. The G1 should never walk closer than 1 meter to a human unless invited.
2.  **Deictic Gestures**: Understanding pointing. If a human points and says "Get that," the robot must use vision to follow the pointing ray.
3.  **Visual Feedback**: The robot should look at the human's eyes during conversation and at the object during manipulation. This establishes "Trust."

## 5. Practical Scenario: Compliant Force Control

A "Hard" robot arm is dangerous. A **Compliant** robot arm "gives" when pushed.

```python
# DEFENSIVE: Compliant Grasping Algorithm
def secure_object(target_force_newtons):
    current_force = 0.0
    
    # 1. Start slow approach
    while current_force < target_force_newtons:
        move_fingers_incrementally(step=0.005) # 5mm
        current_force = read_wrist_force_sensor()
        
        # DEFENSIVE: The "Crush" Guard
        if current_force > (target_force_newtons * 1.5):
            self.get_logger().error("OVERFORCE DETECTED - RELEASING GRIP")
            open_fingers_fully()
            return False
            
        # DEFENSIVE: The "Slip" Check
        if is_robot_moving() and current_force < 0.1:
            self.get_logger().warn("OBJECT SLIPPED - HALTING ARM")
            stop_all_arm_movement()
            return False
            
    return True
```

## 6. Critical Edge Cases: Occlusion

One of the hardest problems in manipulation is when the **Hand blocks the Eye**.
*   **The Trap**: As the robot reaches for a cup, its arm moves between the camera and the cup. The AI "loses" the target.
*   **The Fix: Eye-in-Hand Perception**. Modern humanoids use small secondary cameras on the wrist to maintain a close-up view of the target during the final grasp phase.

## 7. Analytical Research: Tactile Skin

For research-tier mastery, we explore **Electronic Skins**.
*   **The Concept**: Wrapping the entire G1 arm in a pressure-sensitive fabric.
*   **Research Question**: Can a robot learn to identify the "Fullness" of a water bottle purely through tactile feedback (feeling the weight and sloshing) without using vision?

## 8. Multi-Level Summary

### [Beginner]
Manipulation is how the robot touches the world. It uses "Planning" to find a path and "Force Sensors" to know when it has a good grip. Think of it as playing a "Claw Machine" at an arcade, but with a robot that can feel how hard it's squeezing.

### [Pro/Expert]
We use **MoveIt 2** for collision-free planning. Defensive manipulation means using **Inverse Dynamics** to compensate for the weight of the object once it's picked up, so the robot doesn't tilt forward.

### [Researcher]
The future is **Diffusion Policies for Grasping**. Instead of math equations, we use "Generative Models" to predict the flow of motor commands needed to manipulate complex objects like fabrics or liquids.

## 9. Conclusion

Manipulation turns a "Moving Camera" into a "Working Assistant." It is the bridge between navigation and utility. In the next lesson, we give the robot its "Voice" and "High-Level Brain" using LLMs.

---

**Next Lesson**: [Lesson 3: Voice-to-Action with Whisper + LLM Planning](./lesson3)
