<!--
    Sync Impact Report:
    - Version change: 1.0.0 -> 1.1.0
    - List of modified principles:
        - [Renamed] "Hands-On Learning Focus" -> "Hands-On & Hardware-Aware Learning"
        - [Renamed] "Content Density and Richness" -> "Analytical Depth & Research Density"
        - [Added] "Defensive Programming & Input Paranoia"
        - [Added] "Robotics-Specific Rigor & Upgradability"
    - Added sections: N/A (Updates within existing sections)
    - Removed sections: N/A
    - Templates requiring updates:
        - ✅ .specify/templates/plan-template.md
        - ✅ .specify/templates/spec-template.md
        - ✅ .specify/templates/tasks-template.md
    - Follow-up TODOs:
        - TODO(RATIFICATION_DATE): Confirm original adoption date.
-->

# Constitution for Spec-Driven Development of the "Physical AI & Humanoid Robotics" Textbook

## Vision
To create the definitive **Spec-Driven Book for Embodied AI**, bridging the gap between digital intelligence and physical application with uncompromising defensive safeguards. This textbook will serve as a secure, interactive platform for training high-tech upgradable robots—specifically the **Limx TRON2** (7-DoF arms, 70cm reach, 10kg payload, modular biped/wheeled modes)—while teaching the full stack from ROS 2 middleware and Gazebo/Unity simulations to NVIDIA Isaac perception and VLA capstones. We adopt a **Defensive Programming Specialist** mindset: we are paranoid about malicious inputs, strict about data types, and rigorous about error handling, ensuring the book is not just educational but a fortress of reliable, crash-proof robotic knowledge ready for future Artificial Super Intelligence (ASI) integration.

## Core Principles

### Defensive Programming & Input Paranoia
Assume all input is potentially malicious. Enforce precise data types and ranges for all interactive and simulation parameters.
- **Strings**: User queries must be sanitized and capped (e.g., max 2000 chars) to prevent buffer overflows.
- **Numbers**: Simulation parameters must have explicit bounds (e.g., gravity float -9.81 to 0.0; confidence scores 0.0-1.0; top_k integers 1-50).
- **Hardware Specs**: Validate inputs for hardware configs (e.g., VRAM int 12-24GB) to avoid overflow or resource exhaustion errors.
- **OS Validation**: Strictly validate file paths and commands for specific OS environments (Windows WSL string paths vs. Linux apt commands) to prevent injection attacks.

### Hands-On & Hardware-Aware Learning
Content must be actionable and hardware-specific. Explicitly target **RTX workstations, Jetson Orin kits, and Unitree/Limx proxies**. Every lesson must include "Defensive Checks" where users verify their environment (e.g., `assert torch.cuda.is_available()`) before execution. Bridge the sim-to-real gap by detailing transfer protocols for TRON2, ensuring robustness against real-world failures like sensor noise or latency.

### Analytical Depth & Research Density
Go beyond basics. Include analytical research on upgradable kinematics/dynamics, VSLAM for dynamic multi-mode navigation, and ablation studies on balance/manipulation. Content must be "dense" for RAG ingestion—richly detailed, strictly typed, and error-safe metadata to support future AI queries.

### Multi-Level Strata with Safety Gates
Cater to all levels but enforce safety gates.
- **Beginner**: Safety-first introductions, strict sandbox environments.
- **Expert**: Deep dives into TRON2 firmware and NVIDIA Isaac Gym, but with warnings about physical risks.
- **Research**: Theoretical frameworks for ASI integration and shape-shifting morphology.

### Fail-Secure Error Handling
Distinguish between user-facing and developer-facing errors.
- **User**: Friendly, non-technical messages (e.g., "Invalid input—please try shorter text").
- **Developer**: Detailed logs with timestamps, stack traces, and attack indicators (via backend middleware like FastAPI).
- **System**: Fail-closed designs—if a sensor or input is ambiguous, the system (and the code taught) must default to a safe stop state.

### Tech Stack Integrity
Build with **Docusaurus** for the frontend, strictly typed **Python** for backend logic, and **Spec-Kit Plus** for governance. Use **Context7 MCP** and **GitHub MCP** for secure deployment. Ensure all code snippets are linted, typed (mypy strict), and secure.

### Ethical & Secure Robotics
Ban assistance for disallowed activities (e.g., weaponization, surveillance). Promote ethical AI, bias mitigation in embodied systems, and physical safety protocols (e.g., kill switches, e-stop implementation).

## Success Criteria
- **Defensive Robustness**: 100% of interactive inputs have defined type/range constraints. Zero unhandled exceptions in provided code examples.
- **Content Completeness**: Full coverage of TRON2 specs (kinematics, dynamics) and the physical AI stack (ROS 2, Isaac, Gazebo).
- **Cross-OS Validation**: explicit, tested setup guides for Windows (WSL2), Linux (Ubuntu), and macOS, with injection-proof command templates.
- **RAG Readiness**: Content chunks are semantically structured and "poison-free" (sanitized) for safe ingestion by Phase 2 chatbots.
- **User Safety**: Assessment pass rates >80% on safety/security modules before unlocking advanced hardware control chapters.

## Constraints
- **Free Tier & Open Source**: Rely on free tiers (Qdrant, Neon) but never compromise security settings (e.g., enforce SSL).
- **Immutability**: Once published, core safety specs are immutable unless formally amended via ADR.
- **Hardware Realities**: Must support simulation (Isaac Sim) for users without physical TRON2 robots, but enforce "Sim-to-Real" validation checks.
- **Input Limits**: Strict caps on all user-supplied data (text length, file size, numerical ranges).

## Stakeholders
- **Learners**: Need safe, crash-proof learning paths for expensive hardware.
- **Researchers**: Need analytical depth on VLA and upgradable morphology.
- **Security Analysts**: Need assurance that the code provided does not introduce vulnerabilities.
- **Hackathon Judges**: Look for "Defensive by Design" implementation and TRON2 specificity.

## Brand Voice and Style
- **Persona**: The **Defensive Programming Specialist** & **Robotics Architect**.
- **Tone**: Empowering but Paranoid. "Build it, but secure it first." Expert, precise, and vigilant.
- **Style**: High-density technical writing. Use callouts for "Security Alert" and "Hardware Check". Explicitly state data types in text (e.g., "Set the `gravity` float to...").

## Mission
To build the world's most secure, strictly-typed, and analytically deep textbook for Physical AI, training the next generation of engineers to master the **Limx TRON2** and **NVIDIA Isaac** stack without compromising on safety, security, or rigour.

## Architecture Principles
- **Docusaurus Frontend**: With client-side validation for all forms/inputs.
- **Input Validation Layer**: All code examples include Pydantic models or equivalent validation.
- **Error Logging**: Standardized format for simulated error reporting.
- **RAG-Ready Structure**: Markdown headers correspond to semantic vectors; strict metadata schema.

## Risks & Mitigation
- **Malicious Input**: Mitigate via strict type enforcement and sanitization (HTML escape, length limits).
- **Hardware Damage**: Mitigate via mandatory "Sim-First" workflows and software E-stops in all code.
- **Sim-to-Real Failure**: Mitigate via domain randomization tutorials and latency ablation studies.

## Definition of Done
- **Complete**: All modules (ROS 2 to Capstone) drafted with TRON2 specifics.
- **Secure**: All inputs defined with types/ranges; error handling strategies documented.
- **Upgradable**: Content addresses modular upgrades (biped to wheeled).
- **Deployed**: Docusaurus site live with strict CSP (Content Security Policy) headers.

## Governance
This constitution serves as the secure root of trust for the project. Any deviations from defensive coding standards must be documented in an ADR.

**Version**: 1.1.0 | **Ratified**: TODO(RATIFICATION_DATE): Determine original adoption date. | **Last Amended**: 2025-12-29