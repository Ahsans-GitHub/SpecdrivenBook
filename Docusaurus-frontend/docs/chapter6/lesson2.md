---
title: "Lesson 2: Speech Recognition and Natural Language Understanding"
sidebar_label: "Lesson 2: Speech & NLU"
tags: [whisper, stt, nlp, robotics, jetson]
level: [beginner, normal, pro, advanced, research]
description: "Implementing robust voice interaction using OpenAI Whisper and local NLU techniques."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Lesson 2: Speech Recognition and Natural Language Understanding

## 1. The Audio Pipeline

Getting a robot to "hear" involves several steps:
1.  **Audio Capture**: Recording from the Unitree G1's microphones.
2.  **Noise Suppression**: Filtering out the whirring of the robot's own motors.
3.  **VAD (Voice Activity Detection)**: Detecting when a human is actually talking.
4.  **STT (Speech-to-Text)**: Converting sound waves into strings.

## 2. OpenAI Whisper on the Edge

**Whisper** is the state-of-the-art for robotic STT. For real-time use, we use `whisper.cpp` or `faster-whisper`.

### Performance Strata
*   **Tiny/Base Model**: Runs at 5x real-time on a Jetson Orin. High speed, moderate accuracy. Best for commands.
*   **Large Model**: Very accurate but slow. Best for transcribing long research discussions.

### Defensive Voice Processing
Microphones on robots are often close to fans and servos.
*   **Spectral Subtraction**: We record the robot's "idle noise" and subtract it from the incoming audio stream to clarify the human's voice.

## 3. Practical Scenario: Implementing a "Wake Word"

We don't want the robot sending every private conversation to the LLM. We use a **Wake Word** (e.g., "Hey Robot").

```python
import pvporcupine # Example wake-word library
import whisper

def audio_loop():
    while True:
        audio_chunk = get_audio_frame()
        if detect_wake_word(audio_chunk):
            robot.play_sound("listening.wav")
            command_audio = record_until_silence()
            
            # Use Whisper for transcription
            result = model.transcribe(command_audio)
            
            # DEFENSIVE: Confidence Check
            if result['avg_logprob'] < -1.0:
                robot.say("Sorry, I didn't quite catch that. Can you repeat?")
                continue
                
            process_command(result['text'])
```

## 4. Critical Edge Cases: Verbal Ambiguity

Human: *"Go to the second door on the left."*
The robot needs to know its current position and orientation to understand "left."
*   **NLU Solution**: Using **Contextual Embeddings**. We pass the robot's current state (position, detected objects) as "Context" to the NLU engine so it can resolve spatial references.

## 5. Analytical Research: Diarization

**Diarization** is the process of identifying "Who spoke when."
*   **Use Case**: If two people give the robot conflicting commands, who should it obey?
*   **Research**: Using microphone arrays (Beamforming) to locate the speaker's position in 3D space and prioritizing commands from the "Authorized User."

## 6. Defensive Programming Checklist
*   [ ] Does the robot provide visual feedback (e.g., an LED ring) when it is listening?
*   [ ] Have you handled audio buffer overflows?
*   [ ] Is the microphone gain set to avoid clipping when the robot is near loud machinery?

---

**Summary**: Speech is the most natural way for humans to interact with humanoids. By combining Whisper with robust VAD and noise filtering, we turn a noisy robot into an attentive listener.
