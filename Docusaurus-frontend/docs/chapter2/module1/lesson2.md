---
id: lesson2
title: ROS 2 Nodes, Topics, and Services
sidebar_label: Nodes, topics, services, and actions
---

# ROS 2 Nodes, Topics, and Services.

## Heading Breakdown
**ROS 2 Nodes, Topics, and Services** are the atomic units of the ROS graph. **Nodes** are the executable processes that perform computation—the "neurons" of the robot. **Topics** are the named buses over which nodes exchange messages—the "axons" carrying signals. **Services** are the synchronous request-response interfaces—the "reflexes" where you ask for a specific outcome and wait for it. Understanding these is critical because they define the **computational graph** of the robot. Real usage involves a "Camera Node" publishing images to an "Image Topic," which a "Vision Node" subscribes to. An example is the `/cmd_vel` topic, the universal interface for telling a mobile base how fast to move. This is key for **upgradable high-DoF humanoids** because it decouples the sender from the receiver; you can replace the joystick teleoperation node with an AI path planner node, and the wheel controller doesn't know the difference.

## Training Focus: Modular Architecture
We focus on **decoupling**. A well-architected humanoid system has dozens of nodes. We train you to define clear boundaries.
*   **Single Responsibility Principle**: A node should do one thing well (e.g., "Face Detection," not "Face Detection and Arm Control").
*   **Interface Stability**: Topics serve as the contract between modules.

## Detailed Content
### Nodes
A node in ROS 2 is an object that interacts with the ROS graph.
*   **Lifecycle**: Nodes can be managed (Unconfigured, Inactive, Active, Finalized).
*   **Composition**: Multiple nodes can run in a single process to save memory.

### Topics (Publish-Subscribe)
The primary way data moves.
*   **Many-to-Many**: One publisher can talk to ten subscribers.
*   **Asynchronous**: The publisher doesn't wait for the subscriber to process the data.

### Services (Request-Response)
For "transactions."
*   **Synchronous**: The client blocks until the server replies.
*   **Example**: `spawn_entity` in Gazebo.

### Industry Vocab
*   **Callback**: A function that runs when a message arrives.
*   **Spin**: The loop that keeps the node alive and checking for callbacks.
*   **Service Definition (.srv)**: The file defining the request and response structure.

### Code Example: Typed Publisher
```python
# Defensive Node with Typed Interfaces
from rclpy.node import Node
from std_msgs.msg import String

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')
        # Explicit type declaration prevents serialization errors
        self.publisher_ = self.create_publisher(String, 'system_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'System Nominal: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

## Real-World Use Case: Unitree G1
In the G1, the **Battery Node** publishes the voltage to the `/battery_state` topic. The **Safety Node** subscribes to this. If the voltage drops below 20V, the Safety Node triggers a service call to the **Motion Manager** to `enter_crouch_mode`, ensuring the robot sits down safely before power failure.