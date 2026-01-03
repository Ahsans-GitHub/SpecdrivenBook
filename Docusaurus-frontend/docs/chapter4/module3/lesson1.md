---
id: lesson1
title: Focus Advanced perception and training
sidebar_label: NVIDIA Isaac SDK and Isaac Sim
---

# Focus: Advanced perception and training.

## Heading Breakdown
**Focus: Advanced perception and training** centers on the cognitive capabilities of the robot. **Advanced perception** goes beyond simple obstacle detection; it involves semantic understanding—knowing that a "chair" is something to sit on, not just a collision box. **Training** refers to the modern paradigm of Machine Learning, where behaviors are learned from data rather than hard-coded. The importance is **autonomy**; a robot that cannot understand its environment cannot operate without supervision. Real usage involves using **NVIDIA Isaac** to train a neural network to recognize industrial tools. An example is generating 10,000 synthetic images of a drill in various lighting conditions to train a YOLO model. This is key for **upgradable high-DoF humanoids** because it allows the robot's visual cortex to be updated with new object classes via software updates.

## Training Focus: The AI Pipeline
We focus on **data**.
*   **Synthetic Data**: Why we can't rely on manual labeling.
*   **Sim-to-Real**: The art of making simulated training useful in the real world.

## Detailed Content
### The Isaac Ecosystem
*   **Isaac Sim**: The simulator (Omniverse).
*   **Isaac ROS**: The runtime (Jetson).
*   **Isaac Lab**: The reinforcement learning framework.

### Perception Pipelines
Traditional CV vs. Deep Learning.
*   **Traditional**: Canny edge detection (brittle).
*   **Deep Learning**: CNNs and Transformers (robust).

### Industry Vocab
*   **Inference**: Running the model on the robot.
*   **Latency**: How long it takes to process one frame.
*   **Annotator**: The tool that labels ground truth (e.g., bounding boxes).

### Code Example: Data Generation
```python
# Defensive Synthetic Data Generation
import omni.replicator.core as rep

with rep.new_layer():
    camera = rep.create.camera(position=(0, 0, 10))
    render_product = rep.create.render_product(camera, (1024, 1024))
    
    # Randomize lighting to prevent overfitting
    def randomize_lights():
        lights = rep.create.light(light_type="Sphere", intensity=rep.distribution.uniform(500, 1500))
        return lights.node

    rep.randomizer.register(randomize_lights)
```

## Real-World Use Case: Sorting
We train a Unitree G1 to sort recyclables. We generate millions of images of crushed cans and plastic bottles in Isaac Sim. We train a segmentation model. When deployed, the robot can identify a specific brand of soda can on a messy table and pick it up.