---
title: "Sensor Systems (LIDAR, Cameras, IMUs, Force/Torque)"
sidebar_label: "Sensors"
tags: [sensors, perception, lidar, camera, imu, realsense]
level: [beginner, normal, pro, advanced, research]
description: "The eyes, ears, and inner ear of the humanoid: A technical deep dive into robotic perception hardware."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Sensor Systems (LIDAR, Cameras, IMUs, Force/Torque)

## 1. Perception is Reality

For a humanoid robot, the external world does not exist except as a stream of raw bytes. If a sensor fails, or if the code misinterprets a signal, the robot is effectively "blind." In Physical AI, we spend as much time filtering noise as we do making decisions. This lesson explores the four critical sensor suites used in modern humanoids like the **Unitree G1**.

## 2. Photonic Cartography: LIDAR

**LIDAR (Light Detection and Ranging)** is the "Golden Standard" for spatial awareness. It measures distance by shooting laser pulses and measuring the Time-of-Flight (ToF) for the light to bounce back.

### Technical Strata
*   **2D LIDAR**: Scans a single plane (usually for floor-level obstacle avoidance).
*   **3D LIDAR**: Rotates the laser vertically to create a full **Point Cloud** of the environment.
*   **Solid-State LIDAR**: No moving parts. Uses a chip to steer the laser beams. This is the future of humanoid vision because it's resistant to the vibrations of walking.

### Defensive LIDAR Notes
LIDAR is accurate but has "Blind Spots."
*   **Glass and Mirrors**: Lasers pass through glass or reflect off mirrors, making a window appear as an empty doorway.
*   **Dust and Rain**: Particles can scatter the laser, creating "Ghost Obstacles."
*   **Paranoia Check**: Always cross-reference LIDAR data with Camera data before committing to a high-speed movement.

## 3. Stereoscopic Vision: RGB-D Cameras

The **Intel RealSense** is the most common camera in this course. It is an **RGB-D** camera (Red, Green, Blue + Depth).

### How it works
1.  **Dual IR Sensors**: Two cameras look at the world from slightly different angles (Stereo Vision).
2.  **Structured Light**: The camera projects an invisible infrared pattern. If the pattern is "bent," the camera calculates the distance.
3.  **Result**: You get a standard image plus a **Depth Map**, where each pixel represents a distance in meters.

### The RealSense Pipeline
*   **Alignment**: You must align the depth pixels to the color pixels so you can say: "That red pixel is exactly 1.23 meters away."
*   **Defensive Tip**: Use the **Confidence Filter**. If the camera isn't 90% sure about a depth reading (e.g., in a dark corner), discard it.

## 4. Proprioception: The IMU (Inertial Measurement Unit)

The **IMU** is the "Inner Ear" of the robot. It is the single most important sensor for balance.

### Components
1.  **Accelerometer**: Measures linear acceleration (which way is gravity pulling?).
2.  **Gyroscope**: Measures rotational velocity (how fast am I tilting?).
3.  **Magnetometer**: Measures magnetic north (for heading).

### The Math of Drift
If you integrate acceleration to get position, small errors add up.
*   **Example**: A 0.01% error in acceleration leads to a 10-meter position error after one minute.
*   **The Solution: Sensor Fusion**. We use a **Kalman Filter** or a **Complementary Filter** to combine the fast-but-drifting Gyro with the slow-but-steady Accelerometer.

## 5. Haptic Feedback: Force and Torque Sensors

A humanoid must "feel" the world to be safe around humans.
*   **Contact Detection**: Sensors in the feet tell the robot: "I have hit the floor; it is safe to shift weight."
*   **Manipulation**: Sensors in the wrist (like those on the **Unitree G1** arms) measure the resistance of an object.
*   **Compliance**: If a human pushes the robot's arm, the force sensor detects the pressure and the robot "gives" way rather than resisting—preventing injury.

## 6. Defensive Programming: Sensor Sanity Checks

In your ROS 2 nodes, never trust raw sensor data. Implement **Defensive Validation**.

```python
# DEFENSIVE: LIDAR data validation
def scan_callback(self, msg: LaserScan):
    # 1. Range Validation
    for r in msg.ranges:
        if r < msg.range_min or r > msg.range_max:
            # Data is likely noise or out of bounds
            continue
            
        if math.isnan(r) or math.isinf(r):
            # Hardware error or reflection
            self.handle_sensor_error("LIDAR_INVALID_VAL")
            return

    # 2. Rate Validation
    current_time = self.get_clock().now()
    if (current_time - self.last_received).nanoseconds > 200_000_000: # 200ms
        self.get_logger().error("SENSOR STALL: LIDAR data delayed!")
        self.trigger_safety_halt()
```

## 7. Analytical Research: Event-Based Vision

Standard cameras take 30 or 60 snapshots per second. For high-speed humanoid movements (like catching a ball), this is too slow.
*   **Event Cameras (Neuromorphic)**: These sensors only report *changes* in light for each pixel. 
*   **Benefit**: Microsecond latency and near-infinite dynamic range.
*   **Research**: How can we fuse Event data with standard RGB data to provide high-speed reflexes for the Unitree G1?

## 8. Multi-Level Summary

### [Beginner]
Sensors are how the robot sees. Lidar uses lasers, cameras use light, and IMUs use gravity. The biggest challenge is "Noise"—sensors are often wrong, so we use multiple sensors to double-check the world.

### [Pro/Expert]
Mastery involves **Calibration**. An uncalibrated sensor is a lying sensor. You must perform "Hand-Eye Calibration" to ensure the camera's $x,y,z$ matches the arm's $x,y,z$.

### [Researcher]
The field is moving toward **Multi-modal Consensus**. We don't just "fuse" sensors; we require them to "agree." If the camera sees a person but the Lidar doesn't, we treat it as an anomaly (e.g., a poster of a person) and reject the detection.

## 9. Conclusion

Chapter 1 is now complete. You have the philosophy, the physics, the morphology, and the perception. You are ready to move from "Theory" to "Implementation." In Chapter 2, we begin building the **Robotic Nervous System** using ROS 2.

---

**Next Module**: [Chapter 2: ROS 2 Fundamentals](../chapter2/module1-overview)
