---
id: lesson1
title: Focus Middleware for robot control
sidebar_label: ROS 2 architecture and core concepts
---

# Focus: Middleware for robot control.

## Heading Breakdown
**Focus: Middleware for robot control** establishes the central theme of this lesson. **Focus** implies a targeted, deep dive into the specific layer of the software stack that matters most for robotics. **Middleware** is the software glue that sits between the operating system (Linux) and the application code (your robot logic). It handles the low-level details of data transport, serialization, and process management, so you don't have to. **For robot control** specifies the domain: we aren't building web servers; we are building control loops that must execute with strict timing constraints to keep a humanoid balanced. The importance of this is foundational; without middleware, every robot engineer would have to reinvent the wheel of network socket programming. Real usage is found in **ROS 2**, the industry standard middleware. An example is the **DDS (Data Distribution Service)** layer, which ensures that a "Stop" command from the safety node overrides a "Go" command from the navigation node. This is key for **upgradable high-DoF humanoids**, providing a stable API that allows different hardware modules to interoperate seamlessly.

## Training Focus: Upgradable High-DoF Humanoids
This lesson trains you to think in terms of **distributed systems**. A humanoid robot is not one computer; it is a network of actuators, sensors, and computers. We focus on how middleware enables **modularity**. You will learn to design systems where the "Arm Controller" can be upgraded from a PID loop to a Neural Network without changing a single line of code in the "Vision System," simply because they communicate through a standardized middleware interface. This preparation is essential for **ASI-ready** architectures, where AI agents will need to inspect and modify the control graph programmatically.

## Detailed Content
### The Role of Middleware
Middleware abstracts the complexity of communication. In ROS 2, this is achieved through the **RMW (ROS Middleware)** interface.
*   **Abstraction**: You write `publisher.publish(msg)`, and the middleware handles the serialization to CDR (Common Data Representation) and transmission over UDP/IP.
*   **Discovery**: Middleware handles the automatic discovery of nodes. When you start a new node, it announces its presence to the network.

### Industry Vocab
*   **DDS (Data Distribution Service)**: The underlying connectivity standard used by ROS 2.
*   **QoS (Quality of Service)**: Policies that define how data is exchanged (e.g., specific reliability or durability).
*   **IDL (Interface Definition Language)**: A language-neutral way to define data structures (messages).

### Code Example: Middleware Configuration
```python
# Defensive RMW Configuration
import os
from rclpy.qa import QoSProfile, ReliabilityPolicy

# Enforcing specific RMW implementation for determinism
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

def get_defensive_qos():
    # Reliable profile for critical control signals
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=ReliabilityPolicy.VOLATILE
    )
```

### Integration Steps
1.  **Install ROS 2**: Set up the environment (Humble/Iron).
2.  **Select RMW**: Choose `rmw_cyclonedds_cpp` or `rmw_fastrtps_cpp` based on network needs.
3.  **Validate**: Use `ros2 doctor` to check middleware connectivity.

## Real-World Use Case: Unitree G1
On the **Unitree G1**, the middleware handles the high-bandwidth traffic between the internal limb controllers and the high-level Jetson computer. It ensures that the 500Hz joint state updates are delivered with minimal jitter, which is critical for the walking stabilizer.