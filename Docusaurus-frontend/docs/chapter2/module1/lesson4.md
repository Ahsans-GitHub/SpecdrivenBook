---
title: "Lesson 4: Launch Files and Parameter Management"
sidebar_label: "Lesson 4: Launch & Params"
tags: [ros2, launch, parameters, dynamic-reconfigure, orchestration]
level: [beginner, normal, pro, advanced, research]
description: "Orchestrating multi-node systems and managing dynamic robot configurations without magic numbers."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Lesson 4: Launch Files and Parameter Management

## 1. The Orchestration Problem

A humanoid robot like the **Unitree G1** does not run one node; it runs fifty.
*   10 nodes for Joint Controllers.
*   5 nodes for Perception.
*   3 nodes for high-level Reasoning.
*   1 node for the Voice Assistant.

Opening 50 terminal tabs and typing `ros2 run...` is impossible. **Launch Files** solve this. They are the "Scripts of Scripts" that start your entire robot system with a single command. 

Furthermore, you should never hardcode values like `MAX_SPEED = 1.5` in your Python code. What if the robot is on a slippery floor? You need **Parameters**—dynamically tunable values that allow you to change the robot's behavior without recompiling or restarting nodes.

## 2. ROS 2 Parameters: Eliminating "Magic Numbers"

A **Parameter** is a typed value (string, int, float, bool) stored inside a node.

### Defensive Parameter Implementation

```python
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

class BipedController(Node):
    def __init__(self):
        super().__init__('biped_control')
        
        # DEFENSIVE: Describe the parameter and its safe bounds
        speed_desc = ParameterDescriptor(
            description="Max walking speed in m/s",
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.5, step=0.1)]
        )
        
        # Declare Parameter: Name, Default Value, Descriptor
        self.declare_parameter('max_speed', 1.0, speed_desc)
        
        # Get Value
        self.limit = self.get_parameter('max_speed').value
        
        # DEFENSIVE: Monitor for runtime changes (Dynamic Reconfigure)
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params):
        for param in params:
            if param.name == 'max_speed':
                if param.value > 1.5:
                    return SetParametersResult(successful=False, reason="Safety limit exceeded!")
                self.limit = param.value
                self.get_logger().info(f"Speed updated to {self.limit}")
        return SetParametersResult(successful=True)
```

## 3. Python Launch Files: The System Architect

In ROS 2, launch files are written in **Python**. This gives you the power of logic: "If `sim_mode` is True, load Gazebo; otherwise, start the physical motor drivers."

### Anatomy of a Launch File (`bringup.launch.py`)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Arguments (Inputs to the launch file)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 2. Node Configuration
    vision_node = Node(
        package='my_vision_pkg',
        executable='detector',
        name='g1_vision',
        parameters=[{'use_sim_time': use_sim_time}],
        # DEFENSIVE: Respawn if it crashes
        respawn=True,
        respawn_delay=2.0
    )
    
    # 3. Aggregation
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        vision_node
    ])
```

### Defensive Launch Features
*   **Respawn**: If a node crashes (e.g., OOM on the Jetson), the launch system will automatically restart it.
*   **Namespaces**: You can launch the same "Arm Control" node twice, once in the `/left` namespace and once in the `/right` namespace. This prevents topic name collisions.

## 4. YAML Files: The Global Config

For a humanoid, you might have hundreds of PID gains. Putting these in a launch file is messy. We use **YAML Config Files**.

```yaml
# config/g1_params.yaml
g1_controller:
  ros__parameters:
    max_speed: 1.2
    pid: [10.0, 0.1, 1.0]
    safety_enabled: true
```
You can load this file into your node in the launch script:
`parameters=[os.path.join(get_package_share_directory('my_pkg'), 'config', 'g1_params.yaml')]`

## 5. Critical Edge Cases: Parameter Jitter

When you change a parameter at runtime (e.g., tuning balance while the robot is walking):
*   **The Trap**: The parameter update happens in a different thread than the control loop. If you are halfway through a math calculation and the value changes, the robot will fall.
*   **The Fix**: Use **Atomic Updates**. Read the parameter value once at the start of your calculation and use that copy for the rest of the loop.

## 6. Analytical Research: Deterministic Orchestration

In research, we care about **Event Ordering**. 
*   **The Problem**: Launch files start nodes in a random order (Parallel). What if Node A needs Node B to be ready?
*   **Research Solution: Event Handlers**. ROS 2 Launch allows you to write "OnProcessStart" or "OnProcessExit" handlers. 
    *   *Example*: "Wait until the LIDAR node is finished calibrating before starting the Walker node."

## 7. Multi-Level Summary

### [Beginner]
A Launch file is like a "Startup List." It's one file that turns on all the robot's parts. Parameters are like the "Settings" menu in a game—they let you change how things work without changing the code.

### [Pro/Expert]
System orchestration is about **Dependency Management**. We use Launch Arguments to toggle between "Sim" and "Real" modes, and YAML files to manage the thousands of variables required for humanoid gait control.

### [Researcher]
The challenge is **Distributed Configuration**. In a fleet of 100 humanoids, how do we update the `max_speed` parameter on all of them simultaneously? We are studying **Global Parameter Stores** using Etcd or Redis integrated into the ROS 2 graph.

---

**Summary of Module 1**: You have mastered the nervous system. You can build nodes, move data, and orchestrate systems. You are a Robotic Software Engineer. In Module 2, we leave the terminal and enter the **Virtual World** of Simulation.

**Next Module**: [Chapter 3: Robot Simulation](../chapter3/module2-overview)
