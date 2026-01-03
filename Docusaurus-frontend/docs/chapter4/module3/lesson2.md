---
id: lesson2
title: NVIDIA Isaac Sim Photorealistic simulation and synthetic data generation
sidebar_label: AI-powered perception and manipulation
---

# NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation.

## Heading Breakdown
**NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation** introduces the tool that changes the game. **Photorealistic** means the simulation is visually indistinguishable from reality to the robot's cameras. **Synthetic data generation** is the process of programmatically creating labeled training datasets. **Isaac Sim** is built on Pixar's USD (Universal Scene Description) and NVIDIA's Omniverse, allowing for ray-traced rendering in real-time. The importance is **scalability**; we can create datasets that would take years to collect manually. Real usage involves training a **Unitree G1** to navigate a hospital. We build a digital hospital, populate it with moving doctors and patients, and train the navigation policy. This is key for **upgradable systems** because we can test new sensor configurations (e.g., placing the camera on the chest vs. the head) in simulation to optimize coverage before engineering the bracket.

## Training Focus: USD & Omniverse
We focus on **assets**.
*   **USD**: The HTML of 3D worlds. A layered, non-destructive file format.
*   **PhysX 5**: The advanced physics engine in Isaac Sim that handles deformable objects (soft bodies).

## Detailed Content
### Setting up Isaac Sim
*   **Nucleus**: The local server for sharing assets.
*   **Python API**: Controlling the simulator headless.

### Replicator
The engine for domain randomization.
*   **Texture Randomization**: Changing the floor from wood to carpet every frame.
*   **Pose Randomization**: Spawning the cup in different places.

### Industry Vocab
*   **Headless Mode**: Running simulation without a GUI (for cloud training).
*   **Ground Truth**: The perfect labels (depth, segmentation) that the simulator provides for free.
*   **RTX**: Real-Time Ray Tracing.

### Code Example: Loading USD
```python
# Defensive Asset Loading
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

def load_robot():
    usd_path = "omniverse://localhost/NVIDIA/Assets/Robots/Unitree/G1/g1.usd"
    # Check if asset exists before loading to prevent crash
    if not verify_asset(usd_path):
        raise FileNotFoundError(f"Asset not found: {usd_path}")
        
    add_reference_to_stage(usd_path=usd_path, prim_path="/World/G1")
    return Robot(prim_path="/World/G1", name="g1")
```

## Real-World Use Case: Transparent Objects
Glass is notoriously hard for depth cameras. In Isaac Sim, we simulate the refraction of light through glass. We train a neural network to infer the shape of a glass cup from the distorted background behind it, enabling the G1 to pour water without spilling.