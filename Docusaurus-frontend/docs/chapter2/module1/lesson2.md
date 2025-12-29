---
title: "Lesson 2: Nodes, Topics, Services, and Actions"
sidebar_label: "Lesson 2: Communication"
tags: [ros2, topics, services, actions, communication-patterns]
level: [beginner, normal, pro, advanced, research]
description: "Mastering the four patterns of robot communication and knowing exactly when to use each for humanoid control."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Lesson 2: Nodes, Topics, Services, and Actions

## 1. The Language of the Graph

In Lesson 1, we learned that ROS 2 is a decentralized graph of **Nodes**. But how do these nodes actually converse? Humanoid robots require different "Conversation Patterns" depending on whether they are streaming high-speed IMU data, asking a vision node to find a cup, or telling the legs to "Walk to the office."

There are four primary patterns in the ROS 2 language: **Topics, Services, Actions, and Parameters**. In this lesson, we dive deep into the first three.

## 2. Topics: The Broadcast (Pub/Sub)

**Topics** are the most common pattern. They are for **Continuous, Asynchronous Data Streams**.

*   **Analogy**: A Radio Station. One node broadcasts (Publishes) on a frequency, and many nodes can tune in (Subscribe).
*   **Pattern**: One-to-Many. The broadcaster doesn't know (or care) if anyone is listening.
*   **Humanoid Use Case**:
    *   **Joint States**: The robot’s knees and hips constantly publish their current angles (100Hz).
    *   **Perception**: The camera publishes raw frames (30fps).
    *   **Commands**: The gait planner publishes "target velocity" to the base controller.

### Defensive Topic Implementation

```python
from std_msgs.msg import Float64

class ElbowController(Node):
    def __init__(self):
        super().__init__('elbow_control')
        # Topic: /robot/arm/elbow/target
        self.subscription = self.create_subscription(
            Float64,
            '/robot/arm/elbow/target',
            self.listener_callback,
            10 # Queue Depth
        )

    def listener_callback(self, msg: Float64):
        # DEFENSIVE: Validate range against physical hardware stops
        # Unitree G1 Elbow limit: 0.0 to 2.5 radians
        if not (0.0 <= msg.data <= 2.5):
            self.get_logger().error(f"HARDWARE LIMIT VIOLATION: {msg.data}. Rejecting.")
            return # Fail Closed (do not move)
            
        self.apply_torque(msg.data)
```

## 3. Services: The Request (Req/Res)

**Services** are for **Instant, Synchronous Interactions**.

*   **Analogy**: A Phone Call. You ask a question, and you wait for an answer.
*   **Pattern**: One-to-One. The client waits for the server to finish the task and send a response.
*   **Humanoid Use Case**:
    *   **Calibration**: "Node A asks Node B to calibrate the IMU."
    *   **State Change**: "Set the robot to 'Sit' mode."
    *   **Calculation**: "Calculate the Inverse Kinematics for this hand position."

### Defensive Service Implementation

```python
from std_srvs.srv import SetBool

def trigger_calibration(self):
    client = self.create_client(SetBool, 'calibrate_imu')
    
    # DEFENSIVE: Never block indefinitely
    if not client.wait_for_service(timeout_sec=2.0):
        self.get_logger().error("Calibration service unavailable. Hardware risk!")
        return

    request = SetBool.Request()
    request.data = True
    
    # DEFENSIVE: Use call_async() to avoid deadlocking the balance thread
    future = client.call_async(request)
```

## 4. Actions: The Mission (Goal/Feedback/Result)

**Actions** are for **Long-running, Preemptible Tasks**.

*   **Analogy**: Ordering Pizza. You place the order (Goal). The restaurant tells you "It's in the oven... it's out for delivery" (Feedback). Finally, you get the pizza (Result).
*   **Pattern**: One-to-One with continuous feedback. Crucially, you can **Cancel** the action mid-way.
*   **Humanoid Use Case**:
    *   **Navigation**: "Walk to the kitchen." (Takes 30 seconds, provides distance-to-goal feedback).
    *   **Manipulation**: "Grasp the bottle." (Slow, high-stakes movement).

### Why use Actions instead of Services?
If you used a Service to "Walk to the kitchen," the robot's brain would be "blocked" for 30 seconds while waiting for the response. It couldn't see obstacles or stop for a human. Actions run in the background.

## 5. Critical Edge Cases: The "Service Deadlock"

A common mistake for beginners is calling a Service inside a Topic callback using a single-threaded executor.
*   **The Trap**: Callback A starts -> It calls Service B -> It waits. But the same thread is needed to process the Service B response. The system freezes.
*   **The Fix**: Use **Reentrant Groups** or **MultiThreadedExecutors**, which we will cover in Lesson 4.

## 6. Analytical Research: Interface Selection

Choosing the right pattern is a system design decision.
*   **Topic vs Service**: If data needs to be "Fresh" (like a laser scan), use a Topic. If data needs to be "Confirmed" (like a command to lock a joint), use a Service.
*   **Research Problem**: Analyzing the bandwidth impact of high-frequency Actions. Should "Balance" be an Action? 
    *   *Result*: No. Balance is too fast (500Hz). Actions have too much overhead. Balance should be a Topic or a direct hardware interface.

## 7. Multi-Level Summary

### [Beginner]
*   **Topics**: News broadcast (fast, continuous).
*   **Services**: Phone call (request and wait).
*   **Actions**: Project manager (do a task, give updates, tell me when done).

### [Pro/Expert]
Designing a humanoid means minimizing **Graph Jitter**. We use Topics for the control loops and reserved Services for "Safety Interrupts."

### [Researcher]
We are studying **Zero-Latency Orchestration**. How can we use the ROS 2 Action server to manage VLA (Vision-Language-Action) tasks where the "Feedback" is a video stream processed by a remote LLM?

---

**Next Lesson**: [Lesson 3: Building ROS 2 Packages with Python](./lesson3)
