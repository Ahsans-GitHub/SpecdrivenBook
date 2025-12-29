---
title: "Lesson 3: Building ROS 2 Packages with Python"
sidebar_label: "Lesson 3: Packages"
tags: [ros2, python, colcon, packaging, software-engineering]
level: [beginner, normal, pro, advanced, research]
description: "Professional robotic software engineering: Structuring code into modular, distributable ROS 2 packages."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Lesson 3: Building ROS 2 Packages with Python

## 1. The Package: The Unit of Robotic Distribution

In simple Python development, you write a `.py` file and run it. In Physical AI, you build **Packages**. A package is a standardized container for your code, data, and configuration. It allows your "Walking Algorithm" to be easily installed on a **Unitree G1**, a Jetson Orin, or a simulator in the cloud.

The ROS 2 ecosystem uses the **Ament** build system and the **Colcon** build tool. Mastery of these tools is what separates a student from a professional robotic engineer.

## 2. Anatomy of a Python ROS 2 Package

When you create a package, ROS 2 generates a specific folder structure. Every file has a "Defensive" purpose.

```text
my_robot_pkg/
├── package.xml       # The Manifest (Dependencies and Metadata)
├── setup.py          # The Installer (Build and Entry Points)
├── setup.cfg         # Tool configuration
├── my_robot_pkg/     # The Source Folder
│   ├── __init__.py
│   └── controller.py # Your Code
├── resource/         # Markers for the build tool
└── test/             # Defensive Unit Tests
```

### The package.xml: The Dependency Contract
This is the most important file for system stability. You must declare every library your code uses.
```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
```
*   **The Trap**: Your code works locally but fails on the robot because you forgot to list `sensor_msgs`.
*   **The Defensive Fix**: Use `rosdep install` to automatically verify dependencies before running.

## 3. Creating and Building: The Workspace Workflow

You don't build packages in random folders. You use a **Colcon Workspace** (usually `~/ros2_ws`).

### The Workflow
1.  **Create**: `ros2 pkg create --build-type ament_python --node-name my_node my_pkg`
2.  **Code**: Write your defensive logic in `my_pkg/my_node.py`.
3.  **Build**: `colcon build --symlink-install`
    *   *Defensive Tip*: Use `--symlink-install` for Python. It links the files instead of copying them, so you can edit code and see changes instantly without rebuilding.
4.  **Source**: `source install/setup.bash` (This tells your terminal where the new code is).

## 4. The Entry Point: Making Code Executable

In `setup.py`, you map "Robot Commands" to "Python Functions."

```python
entry_points={
    'console_scripts': [
        'g1_controller = my_robot_pkg.controller:main',
    ],
},
```
This allows anyone to run your code by typing `ros2 run my_robot_pkg g1_controller`.

## 5. Practical Scenario: The "Paranoid" Node Skeleton

Let's look at a professional skeleton for a humanoid control node.

```python
# controller.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class G1MasterController(Node):
    def __init__(self):
        super().__init__('g1_master')
        # 1. State Initialization
        self.last_update = self.get_clock().now()
        
        # 2. Defensive Subscriber
        self.sub = self.create_subscription(
            JointState,
            '/g1/joint_states',
            self.joint_callback,
            10
        )
        
        self.get_logger().info("G1 Controller Package initialized.")

    def joint_callback(self, msg):
        # DEFENSIVE: Watchdog Check
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds / 1e9
        
        if dt > 0.1: # Data is more than 100ms old
            self.get_logger().warn(f"DATA STALL: Jitter detected ({dt}s)")
            # Trigger recovery behavior
            
        self.last_update = now
        # ... process joints ...

def main():
    rclpy.init()
    node = G1MasterController()
    try:
        rclpy.spin(node)
    except Exception as e:
        # Standard logging
        node.get_logger().fatal(f"CRITICAL SYSTEM FAILURE: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## 6. Critical Edge Cases: The Python Path

In large projects, you might want to share code between packages.
*   **The Problem**: Node A can't find `utils.py` in Package B.
*   **The Fix**: Use **Shared Libraries**. Place shared logic in a pure Python package and list it as an `<exec_depend>` in your node's `package.xml`.

## 7. Analytical Research: Build Optimization

*   **Python vs C++**: In research, we use Python for rapid prototyping of VLA (Vision-Language-Action) models. However, once the model is trained, we often move the "Inference Wrapper" to a C++ package using **pybind11** to reduce the latency added by the Python Global Interpreter Lock (GIL).
*   **Research Question**: How does the number of Python packages in a workspace impact the startup time of a humanoid fleet? 
    *   *Result*: Negligible, but the memory overhead of multiple Python interpreters is significant on a Jetson Orin Nano.

## 8. Multi-Level Summary

### [Beginner]
A "Package" is just a folder with rules. It ensures that when you give your code to a friend, it actually works on their machine. Use `colcon build` to "install" your code into your workspace.

### [Pro/Expert]
Software engineering in ROS 2 means managing **Namespaces** and **Parameters**. We'll learn how to launch two different "Brains" for the same robot in Lesson 4.

### [Researcher]
We are exploring **Containerized Packaging**. Using Docker to wrap your entire ROS 2 workspace ensures that your research is 100% reproducible across different versions of Ubuntu and NVIDIA drivers.

---

**Next Lesson**: [Lesson 4: Launch Files and Parameter Management](./lesson4)
