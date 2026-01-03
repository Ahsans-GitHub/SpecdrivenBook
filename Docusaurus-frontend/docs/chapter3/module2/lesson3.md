---
id: lesson3
title: High-fidelity rendering and human-robot interaction in Unity
sidebar_label: Physics simulation and sensor simulation
---

# High-fidelity rendering and human-robot interaction in Unity.

## Heading Breakdown
**High-fidelity rendering and human-robot interaction in Unity** bridges the gap between engineering simulation and user experience. **High-fidelity rendering** means producing images that look real—accurate lighting, shadows, textures, and reflections. **Human-Robot Interaction (HRI)** is the study of how people communicate with robots. **Unity** is a game engine used here not for games, but for its superior graphics pipeline compared to Gazebo. The importance is **visualization**; you cannot test how a human reacts to a robot's gesture if the robot looks like a collection of gray cylinders. Real usage involves streaming ROS 2 data into Unity to visualize a **Unitree G1** greeting a person in a VR environment. An example is using the **ROS-TCP-Connector** to mirror the robot's joint states in Unity in real-time. This is key for **upgradable high-DoF humanoids** designed for social interaction, allowing us to test "approach behaviors" (e.g., how close is too close?) without putting a human in danger.

*(Note: Sidebar refers to Physics/Sensors, but per mapping, we cover Unity/HRI here).*

## Training Focus: Visual Realism
We focus on **perception**. Not the robot's perception, but the *human's* perception of the robot.
*   **Uncanny Valley**: Avoiding the creepy look by smoothing motions and improving textures.
*   **VR Integration**: Putting on a headset to stand "next to" the robot.

## Detailed Content
### The Unity-ROS Bridge
How we connect the two worlds.
*   **ROS-TCP-Endpoint**: A ROS node that listens for TCP connections.
*   **Unity Robotics Hub**: A set of C# scripts for parsing ROS messages.

### HRI Scenarios
*   **Proxemics**: Testing social distances.
*   **Gaze**: Making the robot "look" at the user (Head tracking).

### Industry Vocab
*   **Prefab**: A reusable asset in Unity (the robot model).
*   **Raycast**: Checking line-of-sight.
*   **Shader**: Code that calculates the color of pixels (for realistic skin/metal).

### Code Example: Unity C# Script
```csharp
// Defensive ROS Subscriber in Unity
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraVisualizer : MonoBehaviour
{
    public string topicName = "camera/image_raw";

    void Start()
    {
        // Subscribe to ROS topic with safety check
        ROSConnection.GetOrCreateInstance().Subscribe<ImageMsg>(topicName, UpdateTexture);
    }

    void UpdateTexture(ImageMsg msg)
    {
        if (msg.data.Length == 0) {
            Debug.LogWarning("Received empty image frame!");
            return;
        }
        // ... decoding logic ...
    }
}
```

## Real-World Use Case: Safety bubbles
We visualize the robot's "safety bubble" in Unity. While the real robot moves, we project a red sphere around it in Unity representing the danger zone. This helps safety engineers understand exactly where the robot thinks it is safe to be.