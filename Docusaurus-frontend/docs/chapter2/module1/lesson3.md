---
id: lesson3
title: Bridging Python Agents to ROS controllers using rclpy
sidebar_label: Building ROS 2 packages with Python
---

# Bridging Python Agents to ROS controllers using rclpy.

## Heading Breakdown
**Bridging Python Agents to ROS controllers using rclpy** describes the practical implementation of high-level logic in robotics. **Bridging** refers to the connection between the flexible, AI-friendly world of Python and the strict, real-time world of robot hardware. **Python Agents** are the intelligent components—the path planners, the object recognizers, the LLM interfaces. **ROS controllers** are the low-level drivers managing motors and sensors. **rclpy** (ROS Client Library for Python) is the specific library that makes this bridge possible. The importance is accessibility and ecosystem; Python has the best AI libraries (PyTorch, TensorFlow), and `rclpy` allows us to inject that intelligence directly into the robot's control loop. Real usage is writing a **Behavior Tree** in Python that sends commands to a C++ navigation stack. An example is a script that uses `numpy` to calculate a trajectory and `rclpy` to publish it to a robotic arm. This is key for **upgradable high-DoF humanoids** because it allows rapid prototyping of complex behaviors using standard Python tools.

## Training Focus: Package Management & Distribution
We focus on **reusability**. You aren't just writing scripts; you are building *packages*.
*   **Dependency Management**: Declaring exactly what your code needs (`package.xml`).
*   **Entry Points**: Configuring `setup.py` so your nodes can be run from anywhere.

## Detailed Content
### The ROS 2 Package Structure
A Python package in ROS 2 has a specific layout:
*   `package.xml`: The manifest.
*   `setup.py`: The build configuration.
*   `resource/`: Markers for the package index.
*   `module_name/`: The actual Python source code.

### rclpy Architecture
`rclpy` is a wrapper around `rcl` (C library).
*   **Performance**: While Python is slow, `rclpy` offloads the heavy lifting (serialization/transport) to C code.
*   **Threading**: Understanding `MultiThreadedExecutor` to handle multiple callbacks in Python without blocking.

### Industry Vocab
*   **Colcon**: The build tool (Collective Construction).
*   **Symlink Install**: A build mode where files are linked, not copied, allowing for rapid iteration.
*   **DeprecationWarning**: A common signal that an API is changing.

### Code Example: setup.py
```python
# Defensive setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@company.com',
    description='Controller package for G1 Humanoid',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'walker = my_robot_controller.walker_node:main',
        ],
    },
)
```

## Real-World Use Case: Jetson AI
On the Jetson, we often have a "Vision Agent" written in Python using PyTorch. This agent detects a person. Using `rclpy`, it creates a `PersonDetected` message and publishes it. The C++ locomotion controller subscribes to this and adjusts the robot's path. The bridge is seamless.