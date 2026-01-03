---
id: lesson4
title: Capstone Project The Autonomous Humanoid
sidebar_label: Natural human-robot interaction design
---

# Capstone Project: The Autonomous Humanoid. A final project where a simulated robot receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it.

## Heading Breakdown
**Capstone Project: The Autonomous Humanoid** is the summation of the entire curriculum. **Autonomous** means the robot makes its own decisions. **Humanoid** means it does so in a human-shaped body. **A final project...** details the scope: **Voice command** (Module 4, Whisper), **Plans a path** (Module 3, Nav2), **Navigates obstacles** (Module 2, Gazebo/Physics), **Identifies an object** (Module 3, Isaac Perception), and **Manipulates it** (Module 5, VLA). The importance is **integration**; demonstrating that you can build a system greater than the sum of its parts. Real usage is the **Unitree G1** standing in a kitchen, hearing "Pour me some water," and executing the task autonomously. This is key for **upgradable high-DoF humanoids**; it proves the architecture is robust enough for real-world service.

*(Note: Sidebar refers to HRI Design, but per mapping, we cover the Capstone here).*

## Training Focus: System Integration
We focus on **glue**.
*   **State Machines**: Using `Smach` or `BehaviorTree.CPP` to manage the overall flow.
*   **Error Handling**: What happens if the cup slips?

## Detailed Content
### The Mission
1.  **Start**: Robot is at charging dock.
2.  **Trigger**: User says "Find the blue cube."
3.  **Search**: Robot explores the house (Nav2).
4.  **Detect**: Camera sees blue pixels (YOLO).
5.  **Approach**: Robot walks to table.
6.  **Grasp**: Arm reaches out (MoveIt).
7.  **Return**: Robot brings cube to user.

### Evaluation Criteria
*   **Robustness**: Does it work 10/10 times?
*   **Speed**: Is it faster than a snail?
*   **Safety**: Did it hit the wall?

### Industry Vocab
*   **System of Systems**: A complex entity built of smaller complex entities.
*   **Full Stack Robotics**: From soldering iron to neural network.
*   **Deployment**: Getting the code off your laptop and onto the robot.

### Code Example: The Main Loop
```python
# Defensive Main Loop
def main():
    rclpy.init()
    robot = RobotInterface()
    
    try:
        # 1. Wait for Voice
        command = robot.listen_for_command()
        
        # 2. Plan
        plan = robot.ask_llm(command)
        
        # 3. Execute with Watchdog
        for action in plan:
            status = robot.execute(action, timeout=10.0)
            if status == 'failed':
                robot.speak("I'm sorry, I failed to " + action.name)
                robot.go_home()
                return

        robot.speak("Task complete.")
        
    except KeyboardInterrupt:
        robot.emergency_stop()
    finally:
        robot.shutdown()
```

## Real-World Use Case: The Demo
This project is your portfolio. It demonstrates to employers that you understand the entire pipeline. You aren't just a "vision guy" or a "control guy"; you are a **Roboticist**.