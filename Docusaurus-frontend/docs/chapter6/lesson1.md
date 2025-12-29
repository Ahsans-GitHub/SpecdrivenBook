---
title: "Lesson 1: Integrating GPT Models for Conversational AI in Robots"
sidebar_label: "Lesson 1: GPT Integration"
tags: [gpt, llm, prompt-engineering, python, ros2]
level: [beginner, normal, pro, advanced, research]
description: "Connecting Large Language Models to robot control stacks for natural language reasoning."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Lesson 1: Integrating GPT Models for Conversational AI in Robots

## 1. The Robot's "System Prompt"

In traditional AI, a prompt helps generate text. In robotics, a prompt defines the robot's **Persona** and **Constraints**.

### Designing the Persona
A humanoid robot needs to know:
1.  **What it is**: "You are a Unitree G1 humanoid assistant."
2.  **What it can see**: "You have a RealSense camera and can detect: [cup, chair, person]."
3.  **What it can do**: "Available actions: walk_to(target), pick_up(object), wave()."
4.  **Safety Rules**: "Never move faster than 1.0 m/s. Do not touch humans."

## 2. Practical Scenario: The GPT-to-ROS Bridge

We use an OpenAI-style API to process human intent and turn it into **Function Calls**.

```python
import openai
import rclpy
from rclpy.node import Node

class ConversationalRobot(Node):
    def __init__(self):
        super().__init__('conv_robot')
        self.client = openai.OpenAI(api_key="YOUR_KEY")
        
    def process_command(self, user_text: str):
        # DEFENSIVE: Length limit to prevent buffer/token attacks
        if len(user_text) > 500:
            self.get_logger().warn("Command too long, truncating...")
            user_text = user_text[:500]

        response = self.client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": "You are a robot. Output only JSON function calls."},
                {"role": "user", "content": user_text}
            ],
            tools=self.get_robot_tools()
        )
        
        self.execute_tool_calls(response.choices[0].message.tool_calls)

    def get_robot_tools(self):
        return [{
            "type": "function",
            "function": {
                "name": "walk_to",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "location": {"type": "string", "enum": ["kitchen", "office", "charging_station"]}
                    }
                }
            }
        }]
```

## 3. Critical Edge Cases: Hallucinations

LLMs are non-deterministic. They might invent a command like `fly_to_moon()`.
*   **The Trap**: Sending unvalidated AI output directly to the robot's actuators.
*   **Defensive Fix**: **Schema Validation**. Use a library like `jsonschema` or `pydantic` to verify that the AI's output exactly matches your robot's capabilities before passing it to the ROS 2 Action server.

## 4. Analytical Research: Latency vs. Intelligence

*   **Cloud (GPT-4o)**: High intelligence, high latency (1-3 seconds). Good for complex task planning ("Organize the room").
*   **Edge (Llama-3-8B on Jetson)**: Lower intelligence, low latency (<200ms). Good for reactive conversation ("Stop now!") and privacy.
*   **Research**: Implementing a **Hybrid Architecture** where the Edge model handles safety and basic talk, while the Cloud model handles long-term strategy.

## 5. Defensive Programming Checklist
*   [ ] Is your API key stored in a `.env` file (not hardcoded)?
*   [ ] Do you have a timeout for the LLM request?
*   [ ] Does the robot say "I'm thinking..." or blink an LED while waiting for the cloud?
*   [ ] Is there a "Kill Switch" phrase that bypasses the AI?

---

**Summary**: GPT models give robots a semantic understanding of the world. But remember: the LLM is the **Planner**, not the **Controller**. Always verify AI intent against physical reality.
