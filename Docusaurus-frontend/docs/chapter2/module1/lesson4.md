---
id: lesson4
title: "Advanced Integration"
slug: /chapter2/module1/lesson4
---

# Lesson 4: Advanced Integration - Launch Files and Parameter Management (Weeks 3-5 Heaviness)

Having established a solid understanding of ROS 2 communication primitives (nodes, topics, services, actions) and how to describe your robot with URDF, this final lesson of Module 1 focuses on bringing these components together into cohesive, manageable systems. We'll explore **Launch Files** – the powerful orchestration tool in ROS 2 – and delve into **Parameter Management**, essential for configuring your robot's behavior without recompiling code.

This lesson represents a significant step towards developing real-world humanoid applications, emphasizing system integration, configuration best practices, and the robust deployment of complex robotic architectures.

## 4.1 ROS 2 Launch Files: Orchestrating Complex Systems

As robotic systems grow in complexity, manually starting each node, setting parameters, and configuring visualization tools becomes impractical. ROS 2 **Launch Files** provide a declarative way to define how a collection of nodes and other processes should be started, configured, and managed. They are written in Python (or XML, but Python is preferred for its flexibility).

Launch files allow you to:
*   **Start multiple nodes**: Launch several nodes simultaneously or in a defined sequence.
*   **Set parameters**: Pass configuration values to nodes.
*   **Remap topics/services**: Change the names of topics or services to avoid conflicts or adapt to specific setups.
*   **Include other launch files**: Build modular launch configurations.
*   **Conditional execution**: Start nodes or apply configurations based on conditions (e.g., simulation vs. real hardware).
*   **Respawn nodes**: Automatically restart nodes if they crash.

### Basic Python Launch File Structure

```python
# ~/ros2_ws/src/my_humanoid_bringup/launch/humanoid_control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Get package share directory for including other launch files or URDF paths
    my_humanoid_description_dir = get_package_share_directory('my_humanoid_description')
    
    # Example: Include a robot_description launch file
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_humanoid_description_dir, 'launch', 'display.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # Example: Launch a controller node
    joint_state_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Example: Launch a custom Python agent node
    ai_agent_node = Node(
        package='my_py_pkg',
        executable='ai_behavior_node',
        name='humanoid_ai_agent',
        parameters=[{
            'debug_mode': True,
            'movement_speed': 0.5
        }],
        output='screen',
        # Set remapping for topics
        remappings=[
            ('/cmd_vel', '/humanoid/cmd_vel'),
            ('/scan', '/humanoid/lidar_scan')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_description_launch,
        joint_state_controller_node,
        ai_agent_node,
        # ... potentially many more nodes for perception, planning, etc.
    ])
```

To run this launch file:
```bash
ros2 launch my_humanoid_bringup humanoid_control.launch.py use_sim_time:=true
```

## 4.2 Parameter Management: Dynamic Configuration

**Parameters** in ROS 2 are dynamic configuration values that nodes can expose. They allow you to change a node's behavior at runtime without modifying and recompiling its source code. This is invaluable for tuning a robot's performance in different environments or for adapting to varying task requirements.

Key features of ROS 2 parameters:
*   **Node-specific**: Each node maintains its own set of parameters.
*   **Type-safe**: Parameters have defined types (e.g., integer, float, string, boolean).
*   **Dynamic updates**: Parameters can be read, set, and updated at runtime using command-line tools or programmatically.
*   **YAML configuration**: Parameters can be loaded from YAML files during launch.

### Example: Using and Setting Parameters in Python

```python
# ~/ros2_ws/src/my_py_pkg/my_py_pkg/configurable_node.py
import rclpy
from rclpy.node import Node

class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with default values and descriptions
        self.declare_parameter('movement_speed', 0.5)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('robot_id', 'humanoid_alpha')

        # Get parameter values
        self.movement_speed = self.get_parameter('movement_speed').get_parameter_value().double_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        self.get_logger().info(f'ConfigurableNode initialized with:')
        self.get_logger().info(f'  Movement Speed: {self.movement_speed}')
        self.get_logger().info(f'  Debug Mode: {self.debug_mode}')
        self.get_logger().info(f'  Robot ID: {self.robot_id}')

        # Create a timer to demonstrate parameter updates (optional)
        self.create_timer(5.0, self.timer_callback)
        self.get_logger().info('To change parameters, use: ros2 param set /configurable_node movement_speed 0.7')

    def timer_callback(self):
        # Refresh parameter values (if they were changed externally)
        self.movement_speed = self.get_parameter('movement_speed').get_parameter_value().double_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.get_logger().info(f'Timer: Current Speed: {self.movement_speed}, Debug: {self.debug_mode}')


def main(args=None):
    rclpy.init(args=args)
    node = ConfigurableNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Running with YAML parameters**:
You can define parameters in a YAML file:
```yaml
# ~/ros2_ws/src/my_py_pkg/config/my_params.yaml
configurable_node:
  ros__parameters:
    movement_speed: 0.75
    debug_mode: true
    robot_id: "humanoid_beta"
```
And load them in a launch file or directly:
```bash
ros2 run my_py_pkg configurable_node --ros-args --params-file src/my_py_pkg/config/my_params.yaml
```
Or directly using `ros2 param set`:
```bash
ros2 param set /configurable_node movement_speed 0.8
```

## 4.3 Strata-Specific Insights

### Beginner: Getting Started with System Control

*   **Focus**: Understand that launch files simplify starting your robot system. Learn to identify which nodes are being launched and how to pass simple arguments to them (like `use_sim_time`).
*   **Hands-on**: Run the `humanoid_control.launch.py` (after creating a dummy `ai_behavior_node` in `my_py_pkg`) and observe all the nodes starting. Use `ros2 node list` and `ros2 topic list`.

### Researcher: Optimizing Deployment and Cyber-Resilience

*   **Complex Inter-Node Dependencies**: For humanoids, understanding and managing complex dependencies (e.g., a locomotion controller depending on a state estimator, which depends on sensor fusion) within launch files is critical. Explore `RegisterEventHandler` and `EmitEvent` for advanced sequencing and error handling in Python launch files.
*   **Security Configuration**: As of 2025 ROS updates, launch files can now directly integrate security configurations for nodes (e.g., specifying signed policy files for secure DDS). This ensures that critical systems start with the correct security posture.
*   **Dynamic Parameter Reconfiguration**: Investigate how parameters can be dynamically updated in response to environmental changes or mission requirements. For humanoids, this could mean adjusting gait parameters based on terrain analysis or adapting manipulation strategies based on object properties. Explore programmatic parameter clients in Python for advanced control.
*   **Deployment Strategies (Weeks 3-5 Heaviness)**:
    *   **Containerization (Docker/Podman)**: Package entire ROS 2 applications, including launch files and dependencies, into Docker containers for consistent deployment across different host OS (Ubuntu, Windows via WSL, macOS). This ensures identical environments, mitigating "works on my machine" issues.
    *   **Orchestration (Kubernetes/Edge Deployments)**: For fleet management or large-scale humanoid deployments, understand how ROS 2 launch files can be integrated into higher-level orchestration systems like Kubernetes. This is particularly relevant for managing computational resources on edge devices.
    *   **Centralized Logging and Monitoring**: Integrate `log_output='screen'` with external logging solutions and monitoring tools (e.g., Prometheus, Grafana) to provide comprehensive insights into your humanoid's behavior during complex operations.

## 4.4 Error Safety and Critical Scenarios

*   **Launch File Errors**: Syntax errors in Python launch files can prevent the entire system from starting. Use `python -m compileall <launch_file.py>` for basic syntax checking. ROS 2 provides informative error messages during launch failures.
*   **Parameter Overrides**: Be aware of the order of precedence for parameters (defaults, CLI arguments, YAML files, runtime sets). Incorrect overrides can lead to unexpected robot behavior. Always verify parameters using `ros2 param get <node_name> <param_name>`.
*   **Dependency Conflicts (Revisited)**: Launch files are often the point where dependency conflicts manifest. Ensure all necessary packages are built and sourced. For GPU-dependent nodes, verify CUDA/OpenCL paths and driver versions before launching.
*   **Secure DDS Integration**: When security is enabled, misconfigured launch files can prevent nodes from communicating due to authentication or access control failures. Always test secure configurations thoroughly. Implement programmatic checks within nodes to verify security context (e.g., `rclpy.get_security_context()`).

### Quiz: Test Your Understanding

1.  What is the primary purpose of a ROS 2 launch file?
    a) To write new ROS 2 nodes
    b) To compile ROS 2 packages
    c) To orchestrate the startup and configuration of multiple ROS 2 nodes and processes
    d) To manage hardware drivers

2.  Which of the following is NOT a capability of ROS 2 launch files?
    a) Remapping topic names
    b) Programmatically changing a node's source code at runtime
    c) Including other launch files
    d) Setting parameters for nodes

3.  Why is parameter management particularly important for humanoid robotics?
    a) To make the robot look more appealing
    b) To dynamically adjust robot behavior without recompilation
    c) To reduce the number of nodes
    d) To standardize message types

4.  You've created a launch file for your humanoid, but a specific node isn't starting with the parameters you expect. Describe your troubleshooting steps. (Open-ended)

---
**Word Count**: ~2400 lexemes.
