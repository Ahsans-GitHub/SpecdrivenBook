---
id: lesson1
title: Focus The convergence of LLMs and Robotics
sidebar_label: Humanoid robot kinematics and dynamics
---

# Focus: The convergence of LLMs and Robotics.

## Heading Breakdown
**Focus: The convergence of LLMs and Robotics** marks the paradigm shift from explicit programming to semantic understanding. **Convergence** implies the merging of two previously separate fields: Natural Language Processing (LLMs) and Control Theory (Robotics). **LLMs** (Large Language Models) provide the reasoning engine, capable of understanding context, ambiguity, and intent. **Robotics** provides the physical agency. The importance is **generality**; traditional robots are specialists (welders, vacuums), but an LLM-powered robot is a generalist. Real usage involves asking a **Unitree G1** to "Make me a sandwich." The LLM breaks this down into steps (find bread, find knife, etc.) which the robot executes. This is key for **upgradable high-DoF humanoids** because the "software update" is often just a better system prompt or a more capable model (GPT-4 to GPT-5).

*(Note: Sidebar refers to Kinematics, but per mapping, we cover LLM Convergence here).*

## Training Focus: Prompt Engineering
We focus on **instructions**.
*   **Chain of Thought**: Asking the robot to "think out loud" before acting.
*   **Context Window**: Managing the robot's short-term memory of the conversation.

## Detailed Content
### The VLA Stack
*   **Vision**: CLIP / SigLIP.
*   **Language**: Llama 3 / GPT-4o.
*   **Action**: RT-2 / OpenVLA.

### Grounding
The hardest problem. How to map the word "Apple" to the specific pixels of the apple on the table.
*   **Open Vocabulary Detection**: Finding objects the robot has never seen before.

### Industry Vocab
*   **Hallucination**: When the robot invents facts.
*   **Zero-Shot**: Doing a task without training.
*   **Embodiment**: The concept that intelligence requires a body.

### Code Example: OpenAI Wrapper
```python
# Defensive LLM Interface
import openai

def get_robot_action(user_command):
    system_prompt = """
    You are a robot assistant. 
    Output ONLY valid JSON. 
    Available actions: ["move", "grab", "inspect"].
    Example: {"action": "move", "target": "kitchen"}
    """
    
    try:
        response = openai.ChatCompletion.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_command}
            ],
            temperature=0.0 # Deterministic output
        )
        return response.choices[0].message.content
    except Exception as e:
        return '{"action": "error", "reason": "API Failure"}'
```

## Real-World Use Case: The Butler
We ask the G1: "I spilled my coffee." A classical robot does nothing. An LLM-robot infers: "Spilled coffee implies a mess. A mess needs cleaning. I should look for a towel." It then generates a plan to find a towel and wipe the floor.