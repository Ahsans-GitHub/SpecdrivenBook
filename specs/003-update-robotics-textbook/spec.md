# Feature Specification: Update Physical AI & Humanoid Robotics Textbook (Refined)

**Feature Branch**: `003-update-robotics-textbook`
**Created**: 2025-12-29
**Status**: Revised
**Input**: Strict restructuring instruction (No Limx TRON2, specific module/lesson hierarchy).

## User Scenarios & Testing

### User Story 1: Structured Navigation & Content Depth (P1)
**Goal**: Users can navigate a strictly defined hierarchy matching the course syllabus (Weeks 1-13), with specific deep content for each lesson.
**Why P1**: Core content structure update.
**Independent Test**: Verify file path `/docs/chapter1/lesson1.md` exists (no module folder). Verify `/docs/chapter2/module1/lesson1.md` exists (with module folder).

**Acceptance Scenarios**:
1. **Given** Chapter 1, **When** accessed, **Then** it shows 4 lessons directly (Foundations, Digital AI to Physical, Humanoid Landscape, Sensors) and an Overview. No Quiz.
2. **Given** Chapter 2, **When** accessed, **Then** it links to "Module 1 Overview" and lists 4 lessons in a `module1` subfolder.
3. **Given** Chapter 6, **When** accessed, **Then** it shows 3 lessons directly (GPT, Speech/NLU, Multi-modal). No Quiz.
4. **Given** Chapter 7, **When** accessed, **Then** it shows 4 Assessment lessons and a Quiz.

### User Story 2: Multi-Level Content Adaptation (P2)
**Goal**: Users can toggle between 5 difficulty levels (Beginner to Research) to see content tailored to their expertise on every lesson page.
**Why P2**: Constitution principle "Multi-Level Strata".
**Independent Test**: On `/docs/chapter1/lesson1`, toggle to "Research". Verify content expands with advanced theoretical depth.

**Acceptance Scenarios**:
1. **Given** any lesson page, **When** "Research" is selected, **Then** deep analytical content is displayed.
2. **Given** a page reload, **When** checking the toggle, **Then** the previous selection persists.

### User Story 3: Hackathon-Safe Hardware Guide (P3)
**Goal**: Users reference a Hardware Guide tailored to accessible/hackathon-standard robots (Unitree G1/Go2, Jetson, RealSense), avoiding proprietary/unavailable platforms like Limx TRON2.
**Why P3**: Practical applicability for target audience.
**Independent Test**: Go to `/docs/hardware`, verify Unitree and Jetson specs. Confirm NO mentions of Limx TRON2.

**Acceptance Scenarios**:
1. **Given** the hardware page, **When** reading robot specs, **Then** only Unitree G1/Go2 and compatible sensors (RealSense, LIDAR) are listed.
2. **Given** configuration notes, **When** reading, **Then** defensive checks for VRAM (Jetson Orin) are present.

### User Story 4: Dynamic Knowledge Assessment (P4)
**Goal**: Users take dynamic quizzes for Chapters 2, 3, 4, 5, and 7 to validate learning.
**Why P4**: Interactive retention.
**Independent Test**: Navigate to `/docs/chapter2/quiz`, take the quiz. Navigate to `/docs/chapter1/quiz`, verify it does NOT exist (or shows 404/redirect).

**Acceptance Scenarios**:
1. **Given** Chapter 2, 3, 4, 5, or 7, **When** the Quiz page is accessed, **Then** a 10-20 question async quiz loads with feedback.
2. **Given** Chapter 1 or 6, **When** searching for a quiz, **Then** none is available.

## Requirements

### Functional Requirements

- **FR-001**: Implement strict file hierarchy:
    - Ch 1: Direct lessons, no module folder.
    - Ch 2-5: `moduleX` subfolders with 4 lessons each.
    - Ch 6: Direct lessons (3 total), no module folder.
    - Ch 7: Direct assessment lessons (4 total).
- **FR-002**: Content depth must be >2000 words per lesson (>2500 for Ch 7 assessments).
- **FR-003**: `LevelToggle` component must be present on ALL lesson pages.
- **FR-004**: Quizzes must be implemented for Chapters 2, 3, 4, 5, 7 only.
- **FR-005**: Hardware guide must strictly exclude Limx TRON2 and focus on Unitree G1/Go2/Jetson.
- **FR-006**: Module Overviews (Ch 2-5) must be 1500-3000 words.

### Key Entities
- **Lesson**: Markdown content with `LevelToggle`.
- **Quiz**: React component loading specific chapter data.
- **Hardware Config**: Validated specs for Unitree/Jetson.

## Success Criteria
- **Structure Match**: 100% alignment with the "Corrected Book Structure" prompt.
- **Content Purity**: Zero mentions of Limx TRON2 in new content.
- **Interactive Coverage**: Toggles on all 27 lesson/assessment files; Quizzes on 5 chapters.