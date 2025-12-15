---
id: lesson2
title: "Bridging Python Agents to ROS Controllers using rclpy"
slug: /chapter2/module1/lesson2
---

# Lesson 2: Bridging Python Agents to ROS Controllers using `rclpy` - Nodes, Topics, Services, and Actions

In Lesson 1, we introduced the fundamental communication primitives of ROS 2: nodes, topics, and services. Now, we dive deeper into their practical application, specifically focusing on how Python-based AI agents can leverage `rclpy`—the Python client library for ROS 2—to interact with and control robotic hardware and software components. This lesson will solidify your understanding of these primitives and introduce **Actions**, another critical communication pattern for long-running, interruptible tasks.

## 2.1 ROS 2 Primitives Revisited: The Python Perspective

`rclpy` provides a Pythonic interface to the ROS 2 client library (rcl), enabling developers to write ROS 2 nodes, publishers, subscribers, service servers, and service clients directly in Python. This is particularly advantageous for AI development, where Python's rich ecosystem of machine learning libraries (e.g., TensorFlow, PyTorch, scikit-learn) can be seamlessly integrated with robotic control.

### Why Python for ROS 2?

*   **Rapid Prototyping**: Python's conciseness and dynamic typing facilitate quick development and iteration cycles, ideal for experimental AI algorithms.
*   **Rich Ecosystem**: Access to powerful libraries for data analysis, machine learning, and computer vision.
*   **Readability**: Python's syntax promotes clear and maintainable code, crucial for collaborative robotics projects.
*   **Bridging AI and Robotics**: `rclpy` acts as the direct bridge, allowing high-level AI decision-making (written in Python) to translate into low-level robot commands (executed via ROS 2).

## 2.2 Advanced Topic Usage: Custom Message Types and QoS

While `std_msgs` provides basic message types (like `String`, `Int32`), real-world humanoid robots require custom, more complex data structures. `rclpy` fully supports custom message definitions.

### Defining Custom Messages

Custom message types are defined in `.msg` files within a ROS 2 package. For example, a `HumanoidJointState.msg` could look like this:

```
# HumanoidJointState.msg
std_msgs/Header header
string[] joint_names
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts
```

After defining, you need to add it to `package.xml` and `CMakeLists.txt` for C++ or `setup.py` for Python to generate the necessary source files.

### Quality of Service (QoS) Profiles

QoS settings are critical for optimizing communication performance and reliability. In `rclpy`, you can specify QoS profiles when creating publishers or subscribers.

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # FASTEST, but not guaranteed delivery
            history=HistoryPolicy.KEEP_LAST,
            depth=5, # Keep last 5 messages
            durability=DurabilityPolicy.VOLATILE # Only available for current subscribers
        )
        self.publisher_ = self.create_publisher(String, 'my_topic', qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from QoS! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**QoS Parameters for Humanoids (Weeks 3-5 Heaviness):**

For humanoid robots, typical QoS choices are:

*   **Sensor Data (e.g., camera feeds, LiDAR scans)**: Often `BEST_EFFORT` reliability and `KEEP_LAST` history with a small `depth`. Dropping old frames is preferable to delaying new ones, especially for real-time perception.
*   **Critical Commands (e.g., emergency stop, joint commands)**: `RELIABLE` reliability is usually required to ensure delivery, even if it means retransmissions. `TRANSIENT_LOCAL` durability might be used if a late-joining subscriber needs the last known command.
*   **Configuration Parameters**: `RELIABLE` and `TRANSIENT_LOCAL` to ensure new nodes receive the current configuration immediately.

## 2.3 ROS 2 Actions: Long-Running and Interruptible Tasks

Actions are a communication mechanism designed for long-running goals that may be preempted. They extend the concept of services by providing feedback during execution and allowing clients to cancel goals. An action involves three messages:
1.  **Goal**: Sent by the client to the action server (e.g., "Move arm to position X").
2.  **Feedback**: Sent by the action server to the client periodically, indicating progress (e.g., "Arm is at 50% of position X").
3.  **Result**: Sent by the action server to the client once the goal is completed or aborted (e.g., "Arm reached position X successfully").

Actions are ideal for humanoid tasks like:
*   **Navigation**: "Go to location Y" (feedback: current position, obstacles encountered).
*   **Manipulation**: "Pick up object Z" (feedback: gripper closure progress, object detection status).
*   **Complex Gait Generation**: "Walk for 10 meters" (feedback: distance covered, balance status).

### Practical Example: A Simple Fibonacci Action

Let's illustrate with a `Fibonacci` action.

**Defining the Action**: Create a `.action` file (e.g., `Fibonacci.action` in `my_interfaces/action/`).

```
# Fibonacci.action
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

**Update `package.xml` and `setup.py`** to generate action interfaces.

**Action Server (Python `rclpy`)**:

```python
# ~/ros2_ws/src/my_py_pkg/my_py_pkg/fibonacci_action_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from example_interfaces.action import Fibonacci # We'll use the standard example Fibonacci action

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Fibonacci action server has been started.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            sequence.append(sequence[i] + sequence[i-1])
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info('Goal succeeded')
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Action Client (Python `rclpy`)**:

```python
# ~/ros2_ws/src/my_py_pkg/my_py_pkg/fibonacci_action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {list(result.sequence)}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {list(feedback_msg.feedback.partial_sequence)}')


def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

**Update `setup.py`**: Add the new action nodes to the `entry_points` section.

```python
# ~/ros2_ws/src/my_py_pkg/setup.py (excerpt)
    entry_points={
        'console_scripts': [
            'talker = my_py_pkg.talker_node:main',
            'listener = my_py_pkg.listener_node:main',
            'add_server = my_py_pkg.add_two_ints_server:main',
            'add_client = my_py_pkg.add_two_ints_client:main',
            'fibonacci_server = my_py_pkg.fibonacci_action_server:main', # New
            'fibonacci_client = my_py_pkg.fibonacci_action_client:main', # New
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
ros2 run my_py_pkg fibonacci_server
# Terminal 2:
ros2 run my_py_pkg fibonacci_client
```

## 2.4 Strata-Specific Insights

### Beginner: Concepts and Usage

*   **Analogy**: Think of Actions like ordering a pizza: you place an order (goal), get updates on its status (feedback), and finally receive your pizza (result). You can also call to cancel the order.
*   **Focus**: Understand when to use Actions versus Topics or Services. Experiment with the Fibonacci example, observing the feedback as the action progresses.

### Researcher: Distributed Inference & Decentralized Training

*   **Distributed Inference**: `rclpy` enables the deployment of AI inference models as ROS 2 nodes. For humanoids, this means specialized nodes for object detection (e.g., YOLO), pose estimation (e.g., OpenPose), or speech recognition can run independently, publishing their results to topics or providing them via services/actions. Consider how `rclpy`'s asynchronous nature (`rclpy.spin_until_future_complete`, `add_done_callback`) facilitates non-blocking AI computations, crucial for real-time robotic response.
*   **Decentralized Training Problems**: While `rclpy` is primarily for runtime interaction, its distributed nature lays groundwork for decentralized training paradigms. Imagine a fleet of humanoids, each collecting data and performing local model updates. `rclpy` could be used to coordinate these local updates (e.g., sharing model parameters via custom topic messages or aggregated via services) as part of a federated learning strategy. This minimizes data transfer and preserves privacy, particularly relevant for sensitive environmental or interaction data.
*   **GPU Integration and Fallbacks**: Humanoid AI agents often rely on GPUs for accelerated inference.
    *   **Direct GPU Access**: Python libraries like PyTorch and TensorFlow can directly utilize CUDA-enabled GPUs. Ensure your `rclpy` nodes are aware of the available hardware.
    *   **OS/GPU Handling**: Write robust code that checks for GPU availability and provides CPU fallbacks.

    ```python
    import torch
    # ... inside a rclpy node ...
    def __init__(self):
        # ...
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using device: {self.device}")
        self.model = self.model.to(self.device) # Move model to GPU if available
        # ...
    ```
    This ensures your AI agent can still function, albeit slower, if a GPU is not present or configured correctly.

## 2.5 Error Safety and Critical Scenarios

*   **ROS 2 Actions for Robust Task Execution**: Actions inherently provide feedback and cancelation capabilities, making them suitable for critical, long-running tasks. If a humanoid is performing a complex manipulation and encounters an unexpected obstacle, the action can be preempted, and a new goal issued.
*   **Service Timeouts and Retries**: When calling services for critical operations, always implement timeouts and retry logic on the client side. A service server might temporarily become unavailable.
*   **Asynchronous Processing**: `rclpy` heavily leverages Python's `asyncio` capabilities. Improper handling of futures or callbacks can lead to deadlocks or unexpected behavior. Ensure careful management of asynchronous operations, especially when integrating with external blocking libraries.

### Quiz: Test Your Understanding

1.  What is the primary benefit of using `rclpy` for developing ROS 2 nodes in AI applications?
    a) Only C++ can be used for ROS 2.
    b) It allows seamless integration with Python's rich AI ecosystem.
    c) Python nodes are inherently faster than C++ nodes.
    d) It simplifies hardware interfacing.

2.  When would you prefer using a ROS 2 Action over a Service?
    a) For simple request-response interactions.
    b) For continuous data streaming.
    c) For long-running, interruptible tasks that require feedback.
    d) To retrieve static robot parameters.

3.  Which QoS setting would you typically use for critical humanoid joint control commands to ensure they are not lost?
    a) `BEST_EFFORT` reliability
    b) `RELIABLE` reliability
    c) `VOLATILE` durability
    d) `KEEP_LAST` history

4.  Describe a scenario where a GPU fallback to CPU would be beneficial in a `rclpy` node for a humanoid robot. (Open-ended)

---
**Word Count**: ~2000 lexemes.
