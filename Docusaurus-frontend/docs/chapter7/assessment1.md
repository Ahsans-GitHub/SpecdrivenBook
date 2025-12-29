---
title: "Assessment 1: ROS 2 Mastery Recap"
sidebar_label: "1. ROS 2 Mastery"
tags: [ros2, assessment, middleware, architecture]
level: [beginner, normal, pro, advanced, research]
description: "In-depth evaluation of ROS 2 middleware patterns, lifecycle nodes, and secure communications."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Assessment 1: ROS 2 Mastery Recap

## 1. Challenge Overview
This assessment tests your ability to architect the "Nervous System" of a humanoid robot. You are given a scenario where a **Unitree G1** must maintain a balance loop at 500Hz while simultaneously processing high-bandwidth LIDAR data and accepting intermittent voice commands.

## 2. Core Evaluation Criteria

### Architecture Design (>2500 words depth)
*   **The Problem**: A single node handling everything will jitter and crash.
*   **The Solution**: You must describe a **Multi-Node Graph** using **Managed (Lifecycle) Nodes**. 
*   **Task**: Define the transitions for the `balance_controller` node. What happens when the `lidar_driver` enters the `Error` state?
*   **Defensive Rule**: The balance loop must be isolated in a dedicated **Real-time Executor** to prevent high-level AI reasoning from blocking the motor commands.

### QoS Strategy
*   **The Problem**: WiFi interference causes the `/cmd_vel` (Velocity) topic to drop 20% of packets.
*   **Task**: Design a **Quality of Service (QoS)** profile for the `cmd_vel` topic. Should it be `Reliable` or `Best Effort`? 
    *   *Correct Answer*: For a walking robot, `Reliable` with a small `depth` (Queue) is better than `Best Effort`, but we must implement a **Deadline** policy to halt the robot if no command is received within 50ms.

## 3. Practical Coding Task: The Safety Heartbeat

Write a Python class `SafetyMonitor` that subscribes to multiple heartbeats (Vision, IMU, Battery) and publishes a global `EMERGENCY_STOP` if any sensor drifts or fails.

```python
# DEFENSIVE: Use Type Hints and explicit validation
from typing import Dict
from std_msgs.msg import Bool, Float32

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        # ... Init subscribers ...
        self.health_map: Dict[str, bool] = {"imu": True, "vision": True}
        
    def imu_callback(self, msg: Float32):
        # DEFENSIVE: Check for IMU drift or NaN
        if abs(msg.data) > 45.0: # Extreme tilt
            self.trigger_estop("IMU TILT LIMIT EXCEEDED")
```

## 4. Analytical Research: Zero-Copy in Production

For research-level mastery, explain how you would move from Python-based communication to **Shared Memory (Zero-Copy)** for the G1's 4K camera feed.
*   **Research Question**: How does the choice of DDS Vendor (e.g., CycloneDDS vs. FastDDS) impact the jitter of the 500Hz balance loop?

## 5. Summary
To pass this assessment, your design must prioritize **Determinism** and **Fault Tolerance**. A humanoid robot is only as stable as the middleware that controls it.
