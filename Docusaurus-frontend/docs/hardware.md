---
title: "Hardware Requirements & Setup"
sidebar_label: "Hardware Requirements"
description: "Validated hardware specifications for Physical AI development, focusing on accessible platforms like Unitree and NVIDIA Jetson."
---

# Hardware Requirements for Physical AI

To build and deploy the humanoid robotics pipelines described in this book, you need a combination of high-performance simulation workstations and energy-efficient edge compute. This guide specifies the **Hackathon-Safe** hardware stack, focusing on accessibility, community support, and reliability.

## 1. Digital Twin Workstation (Simulation)

Simulation platforms like **NVIDIA Isaac Sim** and **Gazebo** require dedicated GPU power to handle real-time physics and ray-traced rendering.

| Component | Minimum Specification | Recommended Specification |
|-----------|-----------------------|---------------------------|
| **GPU** | NVIDIA RTX 3060 (8GB VRAM) | NVIDIA RTX 4070 Ti or higher (12GB+ VRAM) |
| **CPU** | Intel i7 or AMD Ryzen 7 (8 cores) | Intel i9 or AMD Ryzen 9 (16 cores+) |
| **RAM** | 16 GB DDR4 | 32 GB - 64 GB DDR5 |
| **OS** | Ubuntu 22.04 LTS (Native) | Ubuntu 22.04 LTS or Windows 11 with WSL2 |
| **Storage** | 500 GB NVMe SSD | 1 TB NVMe SSD |

:::warning GPU Check: VRAM is King
For **Isaac Sim**, VRAM is the primary bottleneck. If your VRAM is < 12GB, you will experience frequent crashes during large-scale RL training. Aim for **16GB-24GB** for complex multi-robot simulations.
:::

## 2. Physical AI Edge Kit (On-Robot Compute)

The "Brain" inside the robot must handle high-DoF control loops and vision inference with minimal latency.

### Primary: NVIDIA Jetson Orin Series
The Jetson Orin is the industry standard for edge robotics.
*   **Jetson Orin Nano**: Good for basic navigation and low-res vision.
*   **Jetson Orin NX (16GB)**: The "Sweet Spot" for humanoid research.
*   **Jetson Orin AGX**: Required for full VLA (Vision-Language-Action) models running locally.

### Secondary: Intel RealSense D435i/D455
The "Eyes" of your robot.
*   **Function**: RGB-Depth perception.
*   **Setup**: Requires the `realsense-ros` driver.
*   **Defensive Tip**: Use the **D455** for humanoids; its wider field of view helps with balance and foot-placement visualization.

## 3. Robot Lab Tiers (Hackathon Safe)

We categorize robots into tiers based on cost, complexity, and capability. Use only these platforms for compatibility with this course.

### Tier A: Premium Research (Humanoid)
*   **Platform**: **Unitree G1** or **Unitree H1**.
*   **Cost**: $16,000 - $90,000.
*   **Capability**: Full bipedal locomotion, 23-43 DoF, advanced balance recovery.
*   **Target**: Research labs, advanced Capstone projects.

### Tier B: Mid-Range Proxy (Quadruped)
*   **Platform**: **Unitree Go2** or **Unitree AlienGo**.
*   **Cost**: $2,000 - $15,000.
*   **Capability**: 4-legged stability, excellent for testing navigation and VLA before humanoid transfer.
*   **Target**: Hackathon teams, small labs.

### Tier C: Economy / Educational (Miniature)
*   **Platform**: **Hiwonder TonyPi** or **Robotis OP3**.
*   **Cost**: $500 - $10,000.
*   **Capability**: Simplified kinematics, great for learning ROS 2 and basic gait control.
*   **Target**: Beginners, individual learners.

## 4. Architecture Summary Table

| Layer | Component | Hardware Requirement | Function |
|-------|-----------|----------------------|----------|
| **Cognitive** | LLM / VLA | RTX 4090 or Cloud (A100) | High-level reasoning and planning |
| **Perception** | Vision / VSLAM | Jetson Orin + RealSense | 3D Mapping and object detection |
| **Control** | RL Policy | Jetson Orin (Low-latency) | 500Hz balance and torque control |
| **Actuation** | Joint Motors | Unitree Dual-Motor | Physical movement execution |

## 5. Cloud OpEx & Economy Kit

### Cloud-Native Options (Non-RTX Users)
If you lack a powerful local GPU, you can use cloud instances:
*   **AWS RoboMaker**: ~ $2.50 / hour.
*   **Lambda Labs (RTX 6000)**: ~ $0.80 / hour.
*   **Estimated Cloud OpEx**: ~ $200 - $500 per quarter for active development.

### The "Economy Kit" Setup
For individual learners on a budget:
*   **Compute**: Refurbished RTX 3060 PC ($600).
*   **Edge**: Jetson Orin Nano Developer Kit ($499).
*   **Robot**: Hiwonder TonyPi ($500).
*   **Total**: ~ $1,600 for a complete Physical AI lab.

## 6. The Latency Trap & Solution

**The Trap**: Sending sensor data from the Jetson to a Cloud LLM and waiting for a motor command takes 500ms - 2000ms. In this time, a humanoid will fall.

**The Defensive Solution**:
1.  **Local Safety Envelope**: Always run the 500Hz balance loop LOCALLY on the Jetson.
2.  **Asynchronous Reasoning**: The Cloud LLM sends "Goals" (e.g., "Walk to kitchen"), while the local Jetson handles "Steps" (e.g., "Keep balancing while walking").
3.  **Heartbeat Watchdog**: If the cloud connection drops, the local controller must transition to a safe "Stand-still" or "Sit" state immediately.

---

**Summary**: Your hardware is the foundation of your Physical AI. Don't cut corners on VRAM or cooling. A stable workstation leads to a stable robot.
