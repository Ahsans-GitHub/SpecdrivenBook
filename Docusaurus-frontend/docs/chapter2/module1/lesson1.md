--- 
id: lesson1
title: "ROS 2 Nodes, Topics, and Services"
slug: /chapter2/module1/lesson1
---

# Lesson 1: ROS 2 Nodes, Topics, and Services - ROS 2 Architecture and Core Concepts

Welcome to the foundational lesson of Module 1, where we unlock the basic building blocks of the Robot Operating System 2 (ROS 2). Think of ROS 2 as the central nervous system of your humanoid robot – a sophisticated middleware that enables distinct software components to communicate, coordinate, and perform complex tasks in a distributed environment. Understanding nodes, topics, and services is paramount to grasping how ROS 2 orchestrates robotic intelligence.

## 1.1 The Distributed Architecture of ROS 2

At its heart, ROS 2 is designed for distributed computing. This means your robot's intelligence isn't confined to a single monolithic program; instead, it's composed of many small, specialized executable programs called **nodes**. Each node performs a specific task, such as reading sensor data, performing kinematic calculations, or controlling a motor. This modularity offers significant advantages:

*   **Fault Tolerance**: If one node crashes, the rest of the system can potentially continue operating or recover gracefully.
*   **Reusability**: Nodes can be developed independently and reused across different robotic platforms or applications.
*   **Scalability**: Complex systems can be broken down into manageable, parallelizable components, improving performance and maintainability.
*   **Language Agnostic**: ROS 2 provides client libraries for multiple programming languages (e.g., Python, C++), allowing developers to choose the best tool for each task.

### Key Concepts

Before diving into nodes, topics, and services, it's crucial to understand a few overarching ROS 2 concepts:

*   **Middleware**: ROS 2 is built on a Data Distribution Service (DDS) layer. DDS is an open international standard for real-time systems that ensures reliable, high-performance, and secure data exchange between distributed applications. This is a significant upgrade from ROS 1's custom TCP/IP-based communication.
*   **Graph Model**: ROS 2 applications are often visualized as a graph where nodes are processes, and edges represent communication pathways.
*   **Workspaces and Packages**: ROS 2 projects are organized into workspaces, which contain multiple packages. A **package** is the fundamental unit of ROS 2 software, encapsulating nodes, launch files, configuration, and other resources.

## 1.2 ROS 2 Nodes: The Worker Bees

A **node** is essentially an executable program that uses ROS 2 to communicate with other nodes. Each node should ideally be responsible for a single, well-defined task.

### Practical Example: Creating Your First Node (Python `rclpy`)

Let's create a simple "talker" node that publishes a "Hello, Physical AI!" message and a "listener" node that subscribes to it.

**Prerequisites for Humanoid Setup Clarity (OS-specific)**:

For humanoid robotics, precise setup is critical. While the examples here are basic, they assume a correctly configured ROS 2 environment.

*   **Ubuntu (Recommended for ROS 2 Development)**:
    1.  Ensure you have ROS 2 Iron Irwini installed (as of 2025, this is the stable distribution with relevant updates). Follow the official installation guide for Ubuntu Jammy Jellyfish (22.04 LTS): `docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html`
    2.  Source your ROS 2 environment: `source /opt/ros/iron/setup.bash` (or your specific ROS 2 installation path).
    3.  Create a ROS 2 workspace:
        ```bash
        mkdir -p ~/ros2_ws/src
        cd ~/ros2_ws
        colcon build
        source install/setup.bash
        ```
*   **Windows (via WSL2 - Recommended)**:
    1.  Install WSL2 (Windows Subsystem for Linux 2) with Ubuntu 22.04.
    2.  Follow the Ubuntu ROS 2 Iron installation steps *within your WSL2 Ubuntu environment*.
    3.  Ensure X-server is configured for GUI applications if you plan to use tools like `rqt_graph` or `RViz`.
*   **macOS (via Docker - Recommended)**:
    1.  Install Docker Desktop for macOS.
    2.  Pull a ROS 2 Docker image: `docker pull osrf/ros:iron-desktop`
    3.  Run a Docker container with ROS 2:
        ```bash
        docker run -it --rm \
            --name ros2_iron_container \
            --hostname ros2_iron_host \
            -e DISPLAY=$DISPLAY \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            osrf/ros:iron-desktop
        ```
        (Note: GUI forwarding (`-e DISPLAY` and `-v /tmp/.X11-unix`) might require additional setup on macOS).

Now, inside your ROS 2 workspace (`~/ros2_ws/src` on Linux/WSL, or within your Docker container):

```python
# ~/ros2_ws/src/my_py_pkg/my_py_pkg/talker_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Talker node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, Physical AI! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker_node = TalkerNode()
    rclpy.spin(talker_node)
    talker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# ~/ros2_ws/src/my_py_pkg/my_py_pkg/listener_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Listener node has been started.')

    def listener_callback(self(msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener_node = ListenerNode()
    rclpy.spin(listener_node)
    listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Packaging the Nodes**: Create a `setup.py` and `package.xml` for your `my_py_pkg`.

`~/ros2_ws/src/my_py_pkg/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_py_pkg</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`~/ros2_ws/src/my_py_pkg/setup.py`:
```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yeml]'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_py_pkg.talker_node:main',
            'listener = my_py_pkg.listener_node:main',
        ],
    },
)
```

**Build and Run**:
```bash
cd ~/ros2_ws
colcon build --packages-select my_py_pkg
source install/setup.bash
# Open two terminals, source setup.bash in each
# Terminal 1:
ros2 run my_py_pkg talker
# Terminal 2:
ros2 run my_py_pkg listener
```

## 1.3 ROS 2 Topics: The Data Pipelines

**Topics** are the most common way for nodes to exchange data. They implement a publish-subscribe communication model:
*   A node that wants to share data **publishes** messages to a named topic.
*   Other nodes that are interested in that data **subscribe** to the same topic.

Messages are data structures that adhere to a specific type, ensuring that publishers and subscribers agree on the format of the information being exchanged. The quality of Service (QoS) settings for topics allow fine-grained control over reliability, durability, and other communication properties, crucial for real-time robotic systems.

### Diagram: ROS 2 Topic Communication

```mermaid
graph LR
    PublisherNode --Publishes Message (std_msgs/String)--> Topic[chatter];
    Topic --Subscribes to Message--> SubscriberNode;
```

### Key Considerations for Humanoid Robotics

*   **High-Frequency Sensor Data**: Topics are ideal for streaming data from cameras, LiDAR, IMUs (Inertial Measurement Units) at high rates.
*   **Decentralized Control**: Different parts of a humanoid (e.g., left arm, right arm, head) can publish their status or subscribe to high-level commands via topics.
*   **Security (2025 ROS Updates)**: With the latest ROS 2 Iron updates (as of 2025), DDS security features like authentication, encryption, and access control are more robustly integrated. This ensures that only authorized nodes can publish or subscribe to sensitive topics, crucial for cyber-physical security in humanoids. For instance, encrypting joint state topics to prevent malicious injection of commands.

## 1.4 ROS 2 Services: Request-Response Interactions

While topics are for continuous, asynchronous data streams, **services** provide a synchronous request-response communication model. A client node sends a request to a service server node, and the server processes the request and sends back a single response.

Services are best suited for tasks that:
*   Require an immediate response.
*   Do not involve continuous data streams.
*   Are invoked infrequently.

### Practical Example: Creating a Simple Service

Let's create a service that takes two integers and returns their sum.

```python
# ~/ros2_ws/src/my_py_pkg/my_py_pkg/add_two_ints_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # ROS 2 provides standard service types

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add two ints service server has been started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_server = AddTwoIntsServer()
    rclpy.spin(add_two_ints_server)
    add_two_ints_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# ~/ros2_ws/src/my_py_pkg/my_py_pkg/add_two_ints_client.py
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_client = AddTwoIntsClient()
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    response = add_two_ints_client.send_request(a, b)
    add_two_ints_client.get_logger().info(
        f'Result of add_two_ints: {a} + {b} = {response.sum}')
    add_two_ints_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Update `setup.py`**: Add the new service nodes to the `entry_points` section.

```python
# ~/ros2_ws/src/my_py_pkg/setup.py (excerpt)
    entry_points={
        'console_scripts': [
            'talker = my_py_pkg.talker_node:main',
            'listener = my_py_pkg.listener_node:main',
            'add_server = my_py_pkg.add_two_ints_server:main', # New
            'add_client = my_py_pkg.add_two_ints_client:main', # New
        ],
    },
```

**Build and Run**:
```bash
cd ~/ros2_ws
colcon build --packages-select my_py_pkg
source install/setup.bash
# Open two terminals, source setup.bash in each
# Terminal 1:
ros2 run my_py_pkg add_server
# Terminal 2:
ros2 run my_py_pkg add_client 5 7 # Arguments for a and b
```

## 1.5 Strata-Specific Insights

### Beginner: Grasping Basics

*   **Analogy**: Think of nodes as individual team members, topics as bulletin boards where members post updates, and services as direct requests for help from one team member to another.
*   **Focus**: Ensure you can launch `talker` and `listener` nodes and observe messages. Use `ros2 topic list`, `ros2 topic echo /chatter`, and `ros2 node list` to inspect the running system.

### Researcher: Scalability Analyses and 2025 ROS Iron Updates

*   **Scalability**: Analyze the impact of QoS settings (e.g., `reliability`, `durability`, `history`) on system performance, latency, and resource consumption in large-scale humanoid deployments. How do these settings affect real-time constraints and fault recovery?
*   **2025 ROS Iron DDS Security**: Investigate the latest enhancements in ROS 2's underlying DDS for secure communication. Focus on:
    *   **Authentication**: How node identities are verified to prevent unauthorized participants.
    *   **Encryption**: The mechanisms for securing data transmitted over topics and services, particularly for sensitive humanoid sensor (e.g., biometric) or command data.
    *   **Access Control**: Granular permissions for which nodes can access which topics/services.
    *   **Cyber-Resilient Comms**: Examine how these features contribute to the overall cyber-resilience of a humanoid system against eavesdropping, data tampering, and command injection attacks. Consider the overhead introduced by encryption on high-frequency topics.

## 1.6 Error Safety and Critical Scenarios

*   **Robustness to Network Latency**: In humanoid robots, communication latency can lead to unstable control. Utilize QoS settings like `reliable` (guaranteed delivery) and `deadline` to manage real-time requirements.
*   **Dependency Conflicts with Fallbacks**: When developing complex systems with many ROS 2 packages, dependency conflicts can arise. Always manage dependencies rigorously (`package.xml`, `setup.py`). Implement fallback mechanisms (e.g., defaulting to a safe mode, logging errors and waiting for recovery) if a critical node fails to launch or a topic ceases to publish. `try-except` blocks around `rclpy.init()` or publisher/subscriber creation are good practice.
*   **OS/GPU Handling**: ROS 2 itself is largely CPU-bound for core communication. However, nodes performing heavy computation (e.g., computer vision, inverse kinematics) will utilize GPUs. Ensure your environment has appropriate CUDA/cuDNN drivers for NVIDIA GPUs, or OpenCL/ROCm for AMD, if your nodes depend on them. ROS 2 packages often provide CPU fallbacks if a GPU is unavailable (`try` to import `torch.cuda`, `except` and use `torch.cpu`).

### Quiz: Test Your Understanding

1.  What is the primary advantage of ROS 2's distributed architecture?
    a) Simpler debugging
    b) Enhanced fault tolerance and reusability
    c) Reduced memory footprint
    d) Native GUI support

2.  Which ROS 2 communication mechanism is best suited for streaming continuous sensor data?
    a) Services
    b) Actions
    c) Topics
    d) Parameters

3.  Why is DDS security particularly important for humanoid robotics in 2025?
    a) To make robots run faster
    b) To prevent unauthorized access and command injection
    c) To simplify development
    d) To enable wireless communication

4.  How would you manage a situation where a critical ROS 2 node repeatedly fails to launch due to a dependency issue? (Open-ended)

---
**Word Count**: ~2000 lexemes.
