---
title: "Lesson 4: Sim-to-Real Transfer Techniques"
sidebar_label: "Lesson 4: Sim-to-Real"
tags: [sim-to-real, domain-randomization, system-identification, latency, jetson]
level: [beginner, normal, pro, advanced, research]
description: "Bridging the Reality Gap: Ensuring that AI policies trained in pixels can handle the messy physics of physical hardware."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Lesson 4: Sim-to-Real Transfer Techniques

## 1. The Reality Gap is a Wall

If you train a humanoid to walk in a perfect simulation and then deploy that brain to a real **Unitree G1**, it will fall over in three seconds. This is the **Reality Gap**. 

No simulator, not even NVIDIA Isaac Sim, can model the world perfectly.
*   **Latency**: In sim, commands are instant. In reality, there is a 5ms delay between the Jetson and the motors.
*   **Friction**: In sim, floor friction is a constant. In reality, it changes if the robot walks over a dust bunny or a wet spot.
*   **Mass**: Your URDF says the arm is 1.2kg. In reality, the wiring and grease make it 1.25kg.

Sim-to-Real transfer is the art of making your AI brain "Gap-Proof."

## 2. Domain Randomization (DR): The Master Tool

The most successful strategy for crossing the gap is **Domain Randomization**. Instead of simulation being one "Perfect World," we make it millions of "Slightly Broken Worlds."

During training, we randomly change the physics parameters for every robot instance:
1.  **Mass**: The robot is 5kg, then 6kg, then 4.5kg.
2.  **Friction**: The floor is "Ice," then "Sandpaper."
3.  **Latency**: We add random delays (1ms to 10ms) to the control loop.
4.  **Sensor Noise**: We add jitter to the IMU and joint encoders.

**The result**: The AI learns a policy that works across *all* these variations. When it finally hits the real world, it perceives reality as just another "Random Variation" it has already mastered.

## 3. System Identification (SysID)

For high-precision tasks (like a humanoid using a screwdriver), randomization isn't enough. You need **SysID**.
*   **Method**: You move the real robot arm in a known pattern and measure the torque. You then use an optimization algorithm to calculate the *exact* physical constants (Mass, Inertia, Friction) of the real hardware.
*   **Application**: Update your Isaac Sim USD file with these measured numbers. This shrinks the "Gap" before you even start training.

## 4. Practical Scenario: Measuring Jetson Latency

To ensure your AI "expects" the delay of the Jetson Orin:
1.  **Benchmark**: Use `ros2 topic echo --clock` to measure the time difference between a sensor message and a motor command.
2.  **Inject**: In your Python training script, use a buffer to "Delay" the actions sent to the simulator by that exact amount.

```python
# DEFENSIVE: Simulating real-world latency during AI training
class LatencyBuffer:
    def __init__(self, delay_steps: int):
        self.buffer = []
        self.delay = delay_steps

    def get_safe_action(self, new_action):
        self.buffer.append(new_action)
        if len(self.buffer) > self.delay:
            return self.buffer.pop(0)
        return np.zeros_like(new_action) # Initially stand still
```

## 5. Critical Edge Cases: The "Hardware-in-the-Loop" (HIL)

Sometimes, you can't simulate the communication stack.
*   **HIL**: You connect the physical Jetson Orin from the G1 to your RTX Workstation. The simulator provides the "Eyes" (Virtual Images), but the "Brain" runs on the real robot hardware.
*   **Defensive Tip**: This catches bugs in your OS, thermal throttling issues, and driver conflicts that simulation will never show you.

## 6. Analytical Research: Zero-Shot Transfer

Current research is moving toward **Zero-Shot Transfer**—policies that work on real hardware the very first time.
*   **Technique: Feature Alignment**. We use an AI to "Clean" the messy real-world camera feed so it looks like the clean "Sim" feed before the brain sees it.
*   **Research Question**: Can we use **Foundation Models** (trained on 10,000 different robots) to provide a "Universal Balance" policy that doesn't need sim-to-real tuning for the Unitree G1?

## 7. Defensive Sim-to-Real Checklist
*   [ ] Did you randomize mass by at least ±10%?
*   [ ] Did you add 5ms of "Action Latency" during training?
*   [ ] Is the robot's real-world environment well-lit (to match sim visuals)?
*   [ ] **Paranoid Step**: Before full walking, test the policy on a "Suspended Gantry" (robot hanging from a rope) to see if it moves legs correctly without falling.

---

**Conclusion of Module 3**: You have built the instincts. Your robot can see, it can learn, and it can survive the transition to atoms. In the final module, we move to the "Thinking" layer: **Vision-Language-Action (VLA)**.

**Next Module**: [Chapter 5: Vision-Language-Action (VLA) & Capstone](../chapter5/module4-overview)
