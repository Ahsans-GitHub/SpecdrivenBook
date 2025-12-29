---
title: "Lesson 3: Multi-modal Interaction: Speech, Gesture, Vision"
sidebar_label: "Lesson 3: Multi-modal Fusion"
tags: [multi-modal, gesture-recognition, sensor-fusion, hri]
level: [beginner, normal, pro, advanced, research]
description: "Synthesizing audio and visual cues for intuitive human-robot communication."
---

import LevelToggle from '@site/src/components/LevelToggle';

<LevelToggle />

# Lesson 3: Multi-modal Interaction: Speech, Gesture, Vision

## 1. The Power of "And"

A single mode of communication is often ambiguous.
*   **Speech only**: *"Pick that up."* (Which one?)
*   **Vision only**: Human points at a cup. (Does he want me to wash it, fill it, or throw it away?)
*   **Multi-modal**: Human points at a cup AND says *"Fill this."* (Intent is clear).

## 2. Gesture Recognition

We use **MediaPipe** or **Isaac ROS BodyPose** to track human joints.
*   **Deictic Gestures**: Pointing. We project a ray from the human's shoulder through their fingertip into our 3D occupancy map to find the target object.
*   **Iconic Gestures**: "Stop" hand sign, "Come here" wave.

## 3. Practical Scenario: Resolving "This" and "That"

To implement this, we maintain a **Short-Term Memory** of detected objects.

```python
def handle_multimodal_request(speech_text, image_frame):
    # 1. Detect Gesture
    pointing_ray = detect_pointing_finger(image_frame)
    
    # 2. Transcribe Speech
    text = whisper.transcribe(speech_text)
    
    if "this" in text or "that" in text:
        # 3. Spatial Reasoning
        target_object = find_object_at_ray_intersection(pointing_ray)
        
        # DEFENSIVE: Check if object was actually found
        if not target_object:
            robot.say("I see you pointing, but I don't see an object there. Can you be more specific?")
            return
            
        execute_task(text, target_object)
```

## 4. Critical Edge Cases: Occlusion and Noise

What if the human points while standing behind a table? Or talks while a siren is going off?
*   **Dynamic Weighting**: If the audio signal-to-noise ratio is low, the robot relies more on visual cues (Gestures). If the human is partially occluded, the robot asks for verbal clarification.

## 5. Analytical Research: Social Robotics

Humans have expectations about personal space (**Proxemics**).
*   **Research**: Programming the **Unitree G1** to maintain a "Social Distance" (1.5m to 3m) when talking to humans, and only approaching closer if a "Handshake" or "Hand-over" gesture is detected.

## 6. Defensive Programming Checklist
*   [ ] Does the robot look at the human's face during conversation (Eye Contact)?
*   [ ] Are you filtering out "False Positive" gestures (e.g., a human scratching their head)?
*   [ ] Have you implemented a timeout for multi-modal fusion (if speech comes 10 seconds after the gesture, they are likely unrelated)?

---

**Summary**: Multi-modal interaction is about **Context**. By looking at the whole human—not just listening to their words—we build robots that feel like intelligent partners rather than awkward machines.
