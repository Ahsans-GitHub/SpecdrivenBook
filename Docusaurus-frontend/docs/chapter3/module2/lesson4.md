---
id: lesson4
title: Simulating sensors LiDAR Depth Cameras and IMUs
sidebar_label: Introduction to Unity for robot visualization
---

# Simulating sensors: LiDAR, Depth Cameras, and IMUs.

## Heading Breakdown
**Simulating sensors: LiDAR, Depth Cameras, and IMUs** focuses on generating the synthetic inputs that drive the robot's brain. **LiDAR** (Light Detection and Ranging) provides 360-degree distance maps. **Depth Cameras** (like RealSense) provide dense 3D point clouds. **IMUs** (Inertial Measurement Units) provide acceleration and rotation rates. Simulating these means mathematically calculating what the sensor *would* see if it were real. The importance is **testing perception algorithms**; you can't test a SLAM algorithm without data. Real usage involves configuring a **Gazebo plugin** to emit a `/scan` topic that mimics the noise characteristics of a Velodyne VLP-16. An example is adding Gaussian noise to the IMU data to test if the **Unitree G1**'s balance controller is robust to drift. This is key for **upgradable high-DoF humanoids** because we can swap virtual sensors instantly to see if a better camera improves navigation.

*(Note: Sidebar refers to Unity Intro, but per mapping, we cover Sensors here).*

## Training Focus: Sensor Models
We focus on **noise**. Perfect sensors don't exist.
*   **Gaussian Noise**: Adding random jitter to values.
*   **Update Rate**: Simulating the fact that cameras are slow (30Hz) and IMUs are fast (1000Hz).

## Detailed Content
### Gazebo Sensor Plugins
*   **Ray Sensor**: For LiDAR. GPU-accelerated versions are essential for performance.
*   **Camera Sensor**: Renders the scene to a texture and publishes the pixels.
*   **IMU Sensor**: Reads the physics engine's acceleration of the link.

### Visualizing in RViz
We verify the simulation by looking at the data in RViz.
*   **Point Cloud**: Does the virtual wall match the dots?
*   **Frames**: Is the camera mounted upside down? (Common error).

### Industry Vocab
*   **Intrinsic Matrix**: The camera's focal length and center point.
*   **Bias stability**: How much the IMU drifts over time.
*   **Shadowing**: When objects block the LiDAR beam.

### Code Example: Sensor SDF
```xml
<!-- Defensive Sensor Definition with Noise -->
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>1000</update_rate>
  <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
    <topicName>imu/data</topicName>
    <bodyName>imu_link</bodyName>
    <updateRateHZ>1000.0</updateRateHZ>
    <gaussianNoise>0.005</gaussianNoise> <!-- Defensive: Simulation of real-world noise -->
    <xyzOffset>0 0 0</xyzOffset>
    <rpyOffset>0 0 0</rpyOffset>
  </plugin>
</sensor>
```

## Real-World Use Case: Blind Spots
We use simulation to find the blind spots of the G1. By visualizing the LiDAR rays in Gazebo, we realize that the robot cannot see small objects directly in front of its toes. We then add a secondary downward-facing depth camera to the design to cover this "dead zone."