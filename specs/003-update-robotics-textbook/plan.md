# Implementation Plan: Update Physical AI & Humanoid Robotics Textbook (Refined)

**Branch**: `003-update-robotics-textbook` | **Date**: 2025-12-29 | **Spec**: [specs/003-update-robotics-textbook/spec.md](spec.md)

## Summary

This plan executes the strictly defined restructuring of the textbook. It involves creating a specific hierarchy of Markdown files, implementing the `LevelToggle` and `Quiz` React components, and generating deep, defensive content for 7 chapters. The content focus is shifted to "Hackathon-Safe" hardware (Unitree G1/Go2), explicitly excluding Limx TRON2.

## Technical Context

**Language/Version**: TypeScript 5.x (Components), MDX (Content).
**Framework**: Docusaurus 3.x.
**State Management**: `localStorage` for Level Toggle.
**Components**:
- `LevelToggle`: Global persistence, renders on every lesson.
- `Quiz`: Async loader, per-chapter data, feedback logic.

## Project Structure

### Documentation

```text
specs/003-update-robotics-textbook/
├── spec.md
├── plan.md
└── tasks.md
```

### Source Code (Target Structure)

```text
Docusaurus-frontend/
├── docs/
│   ├── chapter1/                  # Intro (Weeks 1-2)
│   │   ├── overview.md
│   │   ├── lesson1.md
│   │   ├── ...
│   │   └── lesson4.md
│   ├── chapter2/                  # ROS 2 (Weeks 3-5)
│   │   ├── module1-overview.md
│   │   ├── module1/
│   │   │   ├── lesson1.md
│   │   │   └── ...
│   │   └── quiz.md
│   ├── chapter3/                  # Simulation (Weeks 6-7)
│   │   ├── module2-overview.md
│   │   ├── module2/
│   │   │   └── [4 lessons]
│   │   └── quiz.md
│   ├── chapter4/                  # Isaac (Weeks 8-10)
│   │   ├── module3-overview.md
│   │   ├── module3/
│   │   │   └── [4 lessons]
│   │   └── quiz.md
│   ├── chapter5/                  # VLA/Capstone (Weeks 11-13)
│   │   ├── module4-overview.md
│   │   ├── module4/
│   │   │   └── [4 lessons]
│   │   └── quiz.md
│   ├── chapter6/                  # Conversational (Week 13)
│   │   ├── overview.md
│   │   ├── lesson1.md
│   │   ├── lesson2.md
│   │   └── lesson3.md
│   ├── chapter7/                  # Assessments
│   │   ├── overview.md
│   │   ├── assessment1.md
│   │   ├── ...
│   │   ├── assessment4.md
│   │   └── quiz.md
│   └── hardware.md
├── src/
│   ├── components/
│   │   ├── LevelToggle/
│   │   └── Quiz/
```

## Constitution Check

*   [x] **Hands-On**: Retained (Code examples, Sim setups).
*   [x] **Multi-Level**: Retained (Toggle on all lessons).
*   [x] **Content Density**: Retained (>2000 words/lesson).
*   [x] **Hardware-Aware**: Updated to Unitree G1/Go2 (Hackathon Safe).
*   [x] **Structure**: strictly aligned with new user prompt.

## Complexity Tracking

| Violation | Why Needed |
|-----------|------------|
| Mixed Hierarchy (Direct vs Module) | Explicitly requested by user to match course flow (Intro vs Technical Modules). |
