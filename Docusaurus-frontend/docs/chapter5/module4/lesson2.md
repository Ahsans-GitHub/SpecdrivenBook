---
id: lesson2
title: Voice-to-Action Using OpenAI Whisper for voice commands
sidebar_label: Bipedal locomotion and balance control
---

# Voice-to-Action: Using OpenAI Whisper for voice commands.

## Heading Breakdown
**Voice-to-Action: Using OpenAI Whisper for voice commands** removes the keyboard from the loop. **Voice-to-Action** describes the pipeline: Audio -> Text -> Intent -> Motor Command. **OpenAI Whisper** is the state-of-the-art speech recognition model, known for its robustness to accents and background noise. **Voice commands** are the most natural interface for humans. The importance is **accessibility**; anyone can talk to the robot, not just engineers. Real usage involves running `whisper.cpp` on the **Jetson Orin** to process audio locally (privacy-preserving) and publishing the transcript to a `/speech_text` topic. An example is saying "Stop!" and having the robot freeze instantly. This is key for **upgradable high-DoF humanoids** as it enables social integration in homes and hospitals.

*(Note: Sidebar refers to Locomotion, but per mapping, we cover Whisper here).*

## Training Focus: Audio Processing
We focus on **signal**.
*   **VAD (Voice Activity Detection)**: Knowing when someone is speaking vs. background noise.
*   **Wake Word**: "Hey Robot."

## Detailed Content
### The Whisper Model
*   **Transformer Architecture**: Trained on 680,000 hours of audio.
*   **Multilingual**: Can understand "Go to the kitchen" in English, Spanish, or Japanese.

### Integration with ROS 2
*   **Audio Capture**: Using PyAudio or ALSA to grab bytes from the microphone.
*   **Inference**: Running the model.
*   **Publishing**: Sending `std_msgs/String`.

### Industry Vocab
*   **WER (Word Error Rate)**: The metric for accuracy.
*   **Spectrogram**: Visual representation of audio frequencies.
*   **Beam Search**: How the model guesses the most likely sentence.

### Code Example: Whisper Node
```python
# Defensive Whisper Node
import whisper
import rclpy
from std_msgs.msg import String

class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_node')
        self.pub = self.create_publisher(String, 'recognized_speech', 10)
        # Load small model for speed on Edge
        self.model = whisper.load_model("base.en") 

    def process_audio(self, audio_path):
        result = self.model.transcribe(audio_path, fp16=True)
        msg = String()
        msg.data = result['text']
        self.get_logger().info(f"Heard: {msg.data}")
        self.pub.publish(msg)
```

## Real-World Use Case: Noisy Factory
In a factory with loud machinery, standard speech recognition fails. Whisper's attention mechanism allows it to filter out the hum of the conveyor belt and focus on the worker's voice saying "Emergency Stop."