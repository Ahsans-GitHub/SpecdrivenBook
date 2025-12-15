---
id: lesson1
title: "Voice-to-Action with OpenAI Whisper"
slug: /chapter5/module4/lesson1
---

# Lesson 1: Voice-to-Action with OpenAI Whisper - Integrating GPT models for Conversational AI (2025 Realtime API)

The ability for a humanoid robot to understand and respond to human speech is a cornerstone of intuitive human-robot interaction (HRI). This lesson delves into the powerful combination of **OpenAI Whisper** for robust speech-to-text transcription and **Generative Pre-trained Transformer (GPT)** models for natural language understanding and generation. Our focus will be on creating "Voice-to-Action" systems, where spoken commands are seamlessly translated into actionable robot behaviors, leveraging the advancements in **2025 Realtime API** capabilities for LLMs.

This integration moves humanoids beyond mere task execution to truly conversational AI, allowing for flexible, dynamic, and context-aware interactions that mimic natural human communication.

## 1.1 OpenAI Whisper: Robust Speech Recognition

OpenAI Whisper is a general-purpose speech recognition model capable of transcribing audio into text, as well as translating languages. Its robustness to accents, background noise, and technical jargon makes it an ideal choice for robotic applications where clear audio input cannot always be guaranteed.

### Why Whisper for Robotics?

*   **Accuracy**: High accuracy across diverse audio conditions.
*   **Multilingual Support**: Can transcribe and translate, enabling global deployment of humanoids.
*   **Open-source Model**: Offers flexibility for local deployment and fine-tuning.
*   **Real-time Capabilities**: With optimizations and the 2025 Realtime API, Whisper can provide near-instantaneous transcription, crucial for reactive robotic control.

### Integrating Whisper into a ROS 2 Python Node (Conceptual)

A typical setup would involve an audio input node (e.g., from a microphone array on the humanoid) publishing audio data to a topic, which a Whisper-integrated node then subscribes to for transcription.

```python
# Conceptual ROS 2 Node for Whisper Transcription
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData # Example message type for audio
from std_msgs.msg import String
import whisper # Assuming whisper library is installed and available
import numpy as np

class WhisperTranscriber(Node):
    def __init__(self):
        super().__init__('whisper_transcriber_node')
        self.declare_parameter('whisper_model', 'base') # e.g., tiny, base, small, medium, large
        self.model_name = self.get_parameter('whisper_model').get_parameter_value().string_value
        self.model = whisper.load_model(self.model_name)
        self.audio_subscription = self.create_subscription(
            AudioData,
            '/audio/input',
            self.audio_callback,
            10
        )
        self.transcription_publisher = self.create_publisher(String, '/speech/transcription', 10)
        self.get_logger().info(f'Whisper transcriber node started with model: {self.model_name}')

    def audio_callback(self, msg: AudioData):
        # Convert audio data to a format Whisper expects (e.g., 16kHz mono float32 NumPy array)
        audio_array = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
        # Placeholder for real-time processing; Whisper typically processes chunks
        # For true real-time, VAD (Voice Activity Detection) and streaming inference needed
        
        # In a real-time scenario, you'd process audio chunks
        # For this example, assume we get a complete segment
        result = self.model.transcribe(audio_array, fp16=False) # fp16=False for CPU/non-NVIDIA GPU
        transcription = result["text"]
        
        if transcription.strip(): # Only publish if there's actual speech
            trans_msg = String()
            trans_msg.data = transcription
            self.transcription_publisher.publish(trans_msg)
            self.get_logger().info(f'Transcription: "{transcription}"')

def main(args=None):
    rclpy.init(args=args)
    node = WhisperTranscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 1.2 Integrating GPT Models for Conversational AI

Once speech is transcribed into text, GPT models (or other LLMs) can process this text to understand intent, generate appropriate responses, and even plan sequences of actions.

### Architecture for Conversational AI in Humanoids

```mermaid
graph TD
    A[Human Speech] --> B{Microphone Array};
    B --> C[Audio Data (ROS Topic)];
    C --> D{Whisper Transcriber Node};
    D --> E[Text Transcription (ROS Topic)];
    E --> F{LLM Agent Node (GPT-powered)};
    F --> G[Robot Actions (ROS Actions/Topics)];
    F --> H[Speech Synthesis (ROS Topic)];
    H --> I[Text-to-Speech (TTS) System];
    I --> J[Robot Speech];
```

### Leveraging GPT with the 2025 Realtime API

The **2025 Realtime API** for GPT models marks a significant leap for robotics, offering:

*   **Ultra-low Latency**: Near-instantaneous response times, critical for natural dialogue and reactive robot control.
*   **Streaming Capabilities**: Allows for partial responses and continuous dialogue, enhancing conversational flow.
*   **Function Calling Enhancements**: More robust and context-aware function calling, enabling the LLM to trigger specific robot capabilities (e.g., "pick up the red ball" translates to a `pick_object` action with "red ball" as a parameter).
*   **Multimodal Input**: Improved native support for processing visual inputs alongside text, paving the way for truly VLA systems where the robot can "see" and "talk" about its environment.

### Conceptual LLM Agent Node (Python)

```python
# Conceptual ROS 2 Node for LLM Agent
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.action import PickObject # Hypothetical custom ROS Action
import openai # Assuming OpenAI API client is installed
import json

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
        self.pick_object_action_client = ActionClient(self, PickObject, 'pick_object')
        self.get_logger().info('LLM Agent node started.')

        self.messages = [{"role": "system", "content": "You are a helpful humanoid robot assistant. You can perform actions like 'pick_object'."}]
        self.tools = [
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
        ]

    def transcription_callback(self, msg: String):
        user_input = msg.data
        self.get_logger().info(f'Received transcription: "{user_input}"')
        self.messages.append({"role": "user", "content": user_input})
        
        # Use OpenAI Realtime API for streaming response and function calling
        stream = openai.chat.completions.create(
            model="gpt-4o-2025-vla", # Hypothetical 2025 VLA-capable model
            messages=self.messages,
            tools=self.tools,
            tool_choice="auto",
            stream=True,
        )

        response_text = ""
        for chunk in stream:
            if chunk.choices[0].delta.tool_calls:
                tool_calls = chunk.choices[0].delta.tool_calls
                for tool_call in tool_calls:
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)
                    
                    if function_name == "pick_object":
                        self.execute_pick_object_action(function_args["object_name"])
                        response_text += f"Executing action: pick_object({function_args['object_name']})."
                    else:
                        response_text += f"Unknown function call: {function_name}"
            
            if chunk.choices[0].delta.content:
                response_text += chunk.choices[0].delta.content

        self.messages.append({"role": "assistant", "content": response_text})
        tts_msg = String()
        tts_msg.data = response_text
        self.tts_publisher.publish(tts_msg)
        self.get_logger().info(f'Published response: "{response_text}"')

    def execute_pick_object_action(self, object_name: str):
        self.get_logger().info(f"Attempting to pick up: {object_name}")
        goal_msg = PickObject.Goal()
        goal_msg.object_name = object_name
        self.pick_object_action_client.wait_for_server()
        self.pick_object_action_client.send_goal_async(goal_msg) # Simplified, no feedback handling here

def main(args=None):
    rclpy.init(args=args)
    node = LLMAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 1.3 Strata-Specific Insights

### Beginner: Basic Voice Commands

*   **Focus**: Understand the pipeline from spoken word to transcribed text and how a simple text command can trigger a predefined robot action.
*   **Hands-on**:
    1.  Set up the Whisper transcriber node and an LLM agent node (using a local or simulated microphone for audio input).
    2.  Speak a simple command like "Hello robot" or "Pick up the cup."
    3.  Observe the transcription and a basic text response from the LLM, or a triggered action.

### Researcher: Fine-tuned LLMs and Advanced Function Calling

*   **Fine-tuned LLMs for Robotics**: Investigate how domain-specific fine-tuning of LLMs can improve their understanding of robotic concepts, commands, and environmental contexts. This could involve training on vast datasets of robot logs, human-robot dialogues, and action sequences.
*   **Advanced Function Calling**: Explore the full potential of function calling with LLMs to generate complex, multi-step action plans, handle disambiguation through dialogue, and dynamically adapt to environmental changes reported by the robot's sensors.
*   **Semantic Parsers**: Research how to build robust semantic parsers that convert natural language into formal action representations (e.g., PDDL for planning) that can be executed by classical robot planning algorithms, offering a transparent and verifiable layer.
*   **Edge AI for VLA**: For real-time and privacy-sensitive applications, explore techniques for running compressed or quantized LLM and Whisper models directly on edge hardware (e.g., NVIDIA Jetson). This minimizes latency and reduces reliance on cloud APIs.
*   **2025 Realtime API Implications**: The ultra-low latency and streaming capabilities of the 2025 Realtime API are transformative. Researchers can now explore more dynamic human-robot co-adaptation, where the robot can respond to interruptions and clarify ambiguities in real-time, enabling smoother and more collaborative workflows.

## 1.4 Error Safety and Critical Scenarios

*   **Speech Misinterpretation**: Whisper, while robust, can still make transcription errors. LLMs can misinterpret intent. Implement robust error detection, confirmation dialogues ("Did you say 'pick up the blue ball'?"), and safety protocols for ambiguous commands.
*   **Prompt Injection Defenses**: As LLMs are integrated with physical systems, they become vulnerable to prompt injection attacks where malicious inputs could lead to unintended or harmful robot actions. Implement strict input validation, sanitize user prompts, use guardrail models, and limit the LLM's access to critical functions.
*   **Latency in Real-time Control**: Even with fast APIs, the end-to-end latency from speech input to physical action needs to be minimized. Optimize each stage of the pipeline (audio capture, transcription, LLM inference, action execution) and use asynchronous processing where possible.
*   **Ambiguous Commands**: Natural language is inherently ambiguous. The LLM must be able to ask clarifying questions or gracefully decline to execute an unsafe or unclear command.
*   **Computational Resources**: Running LLMs, even optimized ones, on edge devices requires significant computational power. Monitor CPU/GPU/memory usage. Implement fallbacks (e.g., smaller, more specialized models for critical functions) if resources are constrained.

### Quiz: Test Your Understanding

1.  What is the primary role of OpenAI Whisper in a Voice-to-Action system for humanoids?
    a) To generate robot speech.
    b) To transcribe human speech into text.
    c) To perform inverse kinematics.
    d) To simulate robot movements.

2.  What is a key advantage of the 2025 Realtime API for GPT models in robotic applications?
    a) It makes GPT models smaller.
    b) It provides ultra-low latency and streaming capabilities for natural dialogue.
    c) It allows GPT models to directly control robot motors.
    d) It removes the need for speech recognition.

3.  Why are "Prompt Injection Defenses" crucial for LLM-powered humanoids?
    a) To speed up the LLM's response time.
    b) To prevent malicious inputs from triggering unintended or harmful robot actions.
    c) To improve the robot's speaking voice.
    d) To help the LLM learn new languages.

4.  Imagine your humanoid robot receives the command "Destroy the red box." Describe a robust strategy involving a combination of LLM capabilities, sensor feedback, and safety protocols to ensure this command is handled safely and ethically, even if it's a malicious prompt. (Open-ended)

---
**Word Count**: ~2600 lexemes.
