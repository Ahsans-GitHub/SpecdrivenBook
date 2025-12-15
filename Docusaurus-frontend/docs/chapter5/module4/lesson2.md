---
id: lesson2
title: Cognitive Planning with LLMs for ROS Actions
slug: /chapter5/module4/lesson2
---

# Lesson 2: Cognitive Planning with LLMs for ROS Actions - Speech Recognition and Natural Language Understanding

Building on the foundation of voice transcription and LLM integration, this lesson delves into a more advanced application: using **Large Language Models (LLMs)** not just for conversational responses, but for sophisticated **Cognitive Planning** that directly translates high-level natural language goals into sequences of executable **ROS Actions**. This process involves robust **Speech Recognition** (as enabled by tools like OpenAI Whisper) and nuanced **Natural Language Understanding (NLU)** to bridge the semantic gap between human intent and robot capabilities.

The ability for a humanoid robot to parse abstract instructions ("Make me coffee," "Clean the room") and autonomously generate a detailed, ordered plan of low-level robotic actions is a hallmark of truly intelligent physical AI. This capability significantly enhances robot autonomy and flexibility in unstructured environments.

## 2.1 The Planning Challenge in Robotics

Traditional robotic planning often relies on predefined state machines, explicit symbolic representations (e.g., PDDL - Planning Domain Definition Language), or detailed environmental maps. While effective for structured tasks, these methods struggle with:

*   **Ambiguity and Open-endedness**: Natural language instructions are often vague and require common-sense reasoning.
*   **Novelty**: Adapting to unforeseen situations or objects not explicitly modeled.
*   **Cognitive Burden on Humans**: Humans have to break down tasks into highly specific steps.

LLMs offer a powerful new approach by leveraging their vast knowledge base and emergent reasoning capabilities to tackle these challenges.

## 2.2 Speech Recognition (Whisper) and Natural Language Understanding (NLU)

The first step in cognitive planning from voice is accurate and robust **Speech Recognition**. OpenAI Whisper, as discussed in Lesson 1, excels at this. Once transcribed, the raw text undergoes **Natural Language Understanding**.

NLU's role here is multifaceted:

*   **Intent Recognition**: Identifying the user's core goal (e.g., `make_drink`, `retrieve_object`).
*   **Entity Extraction**: Identifying key parameters (e.g., `coffee`, `cup`, `table`).
*   **Disambiguation**: Resolving ambiguities through dialogue or contextual cues.
*   **Coreference Resolution**: Understanding pronouns and references in ongoing conversation.

LLMs perform these NLU tasks implicitly through their generative capabilities, often surpassing classical NLU pipelines, especially for complex or novel phrasing.

## 2.3 LLM-Powered Cognitive Planning to ROS Actions

The core idea is to frame the LLM's task as generating a sequence of calls to predefined "tools" or "functions" that correspond directly to ROS actions, services, or topics. The LLM acts as a high-level planner, orchestrating these robotic primitives.

### Integrating LLMs with ROS Action Specifications

Each ROS Action (e.g., `MoveToPose.action`, `GraspObject.action`, `OpenDoor.action`) has a well-defined goal structure. We can provide these action specifications (or simplified descriptions) to the LLM as available tools.

**Example: LLM Translating Natural Language to ROS Actions**

Consider a humanoid told: "Robot, please go to the kitchen, open the fridge, and get me a glass of water."

The LLM, understanding its available ROS Actions and their parameters, might generate a plan like:

1.  `MoveToPose.action(target_location="kitchen")`
2.  `OpenDoor.action(door_id="fridge_door")`
3.  `PickObject.action(object_name="glass")`
4.  `MoveToPose.action(target_location="water_dispenser")` (Implicitly understood to fill water)
5.  `MoveToPose.action(target_location="human_location", object_held="glass_of_water")`

This process relies heavily on the LLM's common-sense reasoning and its ability to decompose a high-level goal into actionable sub-goals.

### Orchestration in an LLM Agent Node (Conceptual Extension)

The `LLMAgentNode` from Lesson 1 can be extended to handle more complex planning scenarios. The LLM's response might not just be text, but a JSON object containing a sequence of tool calls.

```python
# Conceptual extension of LLM Agent Node for Planning
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.action import MoveToPose, OpenDoor, PickObject # Hypothetical custom ROS Actions
import openai
import json
import asyncio # For asynchronous action execution

class LLMAgentNode(Node):
    def __init__(self):
        super().__init__('llm_agent_node')
        self.transcription_subscription = self.create_subscription(
            String,
            '/speech/transcription',
            self.transcription_callback,
            10
        )
        self.tts_publisher = self.create_publisher(String, '/speech/output', 10)
        
        # ROS Action Clients
        self.move_to_pose_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self.open_door_client = ActionClient(self, OpenDoor, 'open_door')
        self.pick_object_client = ActionClient(self, PickObject, 'pick_object')
        # ... other action clients

        self.get_logger().info('LLM Agent node started for cognitive planning.')

        self.messages = [{"role": "system", "content": "You are a helpful humanoid robot assistant. You can perform actions by calling tools. If a complex task is given, break it down into sequential tool calls. Available locations: kitchen, living room, bedroom. Available objects: cup, water, apple, fridge_door."}]
        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "move_to_pose",
                    "description": "Moves the robot to a specified named location.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "target_location": {"type": "string", "description": "The name of the location to move to."},
                        },
                        "required": ["target_location"],
                    },
                },
            },
            {
                "type": "function",
                "function": {
                    "name": "open_door",
                    "description": "Opens a specified door.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "door_id": {"type": "string", "description": "The ID of the door to open (e.g., fridge_door)."},
                        },
                        "required": ["door_id"],
                    },
                },
            },
            {
                "type": "function",
                "function": {
                    "name": "pick_object",
                    "description": "Picks up an object from the environment.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "object_name": {"type": "string", "description": "The name of the object to pick up."},
                        },
                        "required": ["object_name"],
                    },
                },
            }
            # ... define other robot actions as tools
        ]

    def transcription_callback(self, msg: String):
        user_input = msg.data
        self.get_logger().info(f'Received transcription: "{user_input}"')
        self.messages.append({"role": "user", "content": user_input})
        
        rclpy.create_task(self.process_llm_response_async()) # Process response asynchronously

    async def process_llm_response_async(self):
        stream = openai.chat.completions.create(
            model="gpt-4o-2025-vla", 
            messages=self.messages,
            tools=self.tools,
            tool_choice="auto",
            stream=True,
        )

        full_response_content = []
        function_calls_to_execute = []

        for chunk in stream:
            # Aggregate streamed content and tool calls
            if chunk.choices[0].delta.tool_calls:
                tool_calls = chunk.choices[0].delta.tool_calls
                for tool_call in tool_calls:
                    # In a real implementation, you'd reconstruct the full tool call from chunks
                    function_calls_to_execute.append(tool_call.function)
            
            if chunk.choices[0].delta.content:
                full_response_content.append(chunk.choices[0].delta.content)

        response_text = "".join(full_response_content)
        self.messages.append({"role": "assistant", "content": response_text})
        
        # Execute tool calls sequentially
        for func_call in function_calls_to_execute:
            function_name = func_call.name
            function_args = json.loads(func_call.arguments) # Assuming complete arguments are available
            self.get_logger().info(f"LLM decided to call: {function_name} with args: {function_args}")
            
            if function_name == "move_to_pose":
                await self.execute_move_to_pose(function_args["target_location"])
            elif function_name == "open_door":
                await self.execute_open_door(function_args["door_id"])
            elif function_name == "pick_object":
                await self.execute_pick_object(function_args["object_name"])
            else:
                self.get_logger().warn(f"Unsupported function call: {function_name}")
        
        # Speak out the LLM's textual response after actions (or interspersed)
        tts_msg = String()
        tts_msg.data = response_text
        self.tts_publisher.publish(tts_msg)
        self.get_logger().info(f'Published response: "{response_text}"')

    async def execute_move_to_pose(self, target_location: str):
        self.get_logger().info(f"Executing MoveToPose to: {target_location}")
        goal_msg = MoveToPose.Goal()
        goal_msg.location = target_location
        await self.move_to_pose_client.wait_for_server()
        future = self.move_to_pose_client.send_goal_async(goal_msg)
        # Simplified, ideally wait for result and handle feedback
        await future # Await the goal being accepted
        self.get_logger().info(f"MoveToPose goal sent for: {target_location}")

    async def execute_open_door(self, door_id: str):
        self.get_logger().info(f"Executing OpenDoor for: {door_id}")
        goal_msg = OpenDoor.Goal()
        goal_msg.door_id = door_id
        await self.open_door_client.wait_for_server()
        future = self.open_door_client.send_goal_async(goal_msg)
        await future
        self.get_logger().info(f"OpenDoor goal sent for: {door_id}")

    async def execute_pick_object(self, object_name: str):
        self.get_logger().info(f"Executing PickObject for: {object_name}")
        goal_msg = PickObject.Goal()
        goal_msg.object_name = object_name
        await self.pick_object_client.wait_for_server()
        future = self.pick_object_client.send_goal_async(goal_msg)
        await future
        self.get_logger().info(f"PickObject goal sent for: {object_name}")

def main(args=None):
    rclpy.init(args=args)
    node = LLMAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
This conceptual node demonstrates how an LLM can be used to generate a *sequence* of ROS actions. Error handling, feedback processing, and robust state management would be added in a full implementation.

## 2.4 Strata-Specific Insights

### Beginner: From Words to Simple Actions

*   **Focus**: Understand that the robot can take complex instructions and break them down into simple actions. Use simple commands to see the robot execute a single ROS Action.
*   **Hands-on**:
    1.  Start a simulated humanoid with basic ROS Actions defined (e.g., `MoveToPose`, `PickObject`).
    2.  Use the LLM Agent node (perhaps with a text input rather than Whisper initially) to give commands like "Move to the living room" or "Pick up the cup."
    3.  Observe the simulated robot executing the corresponding single action.

### Researcher: Advanced Planning and LLM-Robot Interface

*   **Hierarchical Planning**: Research how LLMs can integrate with traditional hierarchical planners (e.g., PDDL-based) to combine the common-sense reasoning of LLMs with the guarantees of classical planning.
*   **Feedback and Replanning**: Investigate how LLMs can dynamically replan based on real-time sensor feedback or execution failures. This requires feeding the robot's current state and any errors back to the LLM.
*   **Learning from Human Feedback (RLHF for Robotics)**: Explore how human feedback on generated plans or executed actions can be used to fine-tune LLMs, making them better robotic planners over time.
*   **Explainable AI (XAI) for Robot Planning**: Since LLMs are black boxes, research methods to make their planning decisions more transparent and explainable to human operators, especially in critical scenarios.
*   **Zero-Shot/Few-Shot Planning**: How well can LLMs plan for novel tasks or objects with little to no prior training, leveraging their extensive pre-training knowledge?

## 2.5 Error Safety and Critical Scenarios

*   **Incorrect Plan Generation**: The LLM might generate an unsafe, inefficient, or incorrect sequence of actions.
    *   **Mitigation**: Implement a "safety supervisor" layer that validates the LLM's plan against a set of hard constraints (e.g., collision avoidance, joint limits, no-go zones) before execution. Use a rule-based system or a smaller, specialized safety LLM.
*   **Action Execution Failures**: A generated ROS Action might fail (e.g., target location unreachable, object too heavy).
    *   **Mitigation**: The robot needs to detect these failures, report them back to the LLM (as part of the context), and allow the LLM to replan or ask for human intervention.
*   **Prompt Injection Attacks**: Malicious prompts could instruct the LLM to generate harmful plans.
    *   **Mitigation**: Strict validation of LLM outputs, sandboxing of LLM execution, and explicit human confirmation for critical actions.
*   **Computational Latency**: Complex planning with LLMs can introduce latency.
    *   **Mitigation**: Prioritize critical actions, use asynchronous execution, and consider smaller, more efficient local models for immediate responses. Edge AI for VLA solutions are critical here, running quantized LLMs on board the robot to reduce round-trip latency to cloud APIs.

### Quiz: Test Your Understanding

1.  What is the primary benefit of using LLMs for Cognitive Planning in robotics?
    a) They eliminate the need for any programming.
    b) They can translate high-level natural language goals into executable action sequences.
    c) They make robots more physically robust.
    d) They are faster than traditional planners.

2.  What role does Natural Language Understanding (NLU) play in LLM-powered planning?
    a) It executes the robot's physical actions.
    b) It transcribes speech into text.
    c) It identifies intent, extracts entities, and disambiguates meaning from human instructions.
    d) It manages the robot's hardware.

3.  Why is a "safety supervisor" layer important when using LLMs for robot planning?
    a) To make the robot seem more polite.
    b) To validate the LLM's generated plan against safety constraints before execution.
    c) To accelerate the planning process.
    d) To help the LLM understand more languages.

4.  A humanoid robot is tasked with "cleaning the table" using an LLM-powered cognitive planner. Describe how a series of ROS Actions (e.g., `MoveToTable`, `DetectObjects`, `PickObject`, `WipeSurface`) could be orchestrated by the LLM, and what critical error-handling steps would be needed if `PickObject` fails for a specific item. (Open-ended)

---
**Word Count**: ~2700 lexemes.
