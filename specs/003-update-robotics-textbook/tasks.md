# Tasks: Update Physical AI & Humanoid Robotics Textbook (Refined)

**Input**: Strict restructuring instruction.
**Prerequisites**: plan.md, spec.md.

**Organization**: Grouped by Chapter/Story to ensure structural integrity.

## Phase 1: Setup & Components (Re-verification)

- [x] T001 Verify Docusaurus installation
- [x] T002 Create directory structure (Updated for new hierarchy)
- [x] T003 Create LevelToggle component structure
- [x] T004 Create LevelToggle styles
- [x] T005 Create Quiz component structure
- [x] T006 Create Quiz data structure
- [x] T007 Create Quiz styles

## Phase 2: Chapter 1 - Introduction (User Story 1)

**Goal**: Intro content, direct lessons, no quiz.

- [x] T008 [US1] Create Chapter 1 Overview (`docs/chapter1/overview.md`)
- [x] T009 [US1] Create Ch1 Lesson 1: Foundations (`docs/chapter1/lesson1.md`)
- [x] T010 [US1] Create Ch1 Lesson 2: Digital to Physical (`docs/chapter1/lesson2.md`)
- [x] T011 [US1] Create Ch1 Lesson 3: Humanoid Landscape (`docs/chapter1/lesson3.md`)
- [x] T012 [US1] Create Ch1 Lesson 4: Sensor Systems (`docs/chapter1/lesson4.md`)

## Phase 3: Chapter 2 - ROS 2 Fundamentals (User Story 1 & 4)

**Goal**: Module 1 subfolder, Quiz.

- [x] T013 [US1] Create Module 1 Overview (`docs/chapter2/module1-overview.md`)
- [x] T014 [US1] Create Ch2 Lesson 1: Architecture (`docs/chapter2/module1/lesson1.md`)
- [x] T015 [US1] Create Ch2 Lesson 2: Nodes/Topics (`docs/chapter2/module1/lesson2.md`)
- [x] T016 [US1] Create Ch2 Lesson 3: Packages (`docs/chapter2/module1/lesson3.md`)
- [x] T017 [US1] Create Ch2 Lesson 4: Launch/Params (`docs/chapter2/module1/lesson4.md`)
- [x] T018 [US4] Create Ch2 Quiz (`docs/chapter2/quiz.md`) & Populate Data

## Phase 4: Chapter 3 - Robot Simulation (User Story 1 & 4)

**Goal**: Module 2 subfolder, Quiz.

- [x] T019 [US1] Create Module 2 Overview (`docs/chapter3/module2-overview.md`)
- [x] T020 [US1] Create Ch3 Lesson 1: Gazebo Setup (`docs/chapter3/module2/lesson1.md`)
- [x] T021 [US1] Create Ch3 Lesson 2: URDF/SDF (`docs/chapter3/module2/lesson2.md`)
- [x] T022 [US1] Create Ch3 Lesson 3: Physics/Sensors (`docs/chapter3/module2/lesson3.md`)
- [x] T023 [US1] Create Ch3 Lesson 4: Unity Viz (`docs/chapter3/module2/lesson4.md`)
- [x] T024 [US4] Create Ch3 Quiz (`docs/chapter3/quiz.md`) & Populate Data

## Phase 5: Chapter 4 - NVIDIA Isaac (User Story 1 & 4)

**Goal**: Module 3 subfolder, Quiz.

- [x] T025 [US1] Create Module 3 Overview (`docs/chapter4/module3-overview.md`)
- [x] T026 [US1] Create Ch4 Lesson 1: Isaac SDK/Sim (`docs/chapter4/module3/lesson1.md`)
- [x] T027 [US1] Create Ch4 Lesson 2: Perception (`docs/chapter4/module3/lesson2.md`)
- [x] T028 [US1] Create Ch4 Lesson 3: RL Control (`docs/chapter4/module3/lesson3.md`)
- [x] T029 [US1] Create Ch4 Lesson 4: Sim-to-Real (`docs/chapter4/module3/lesson4.md`)
- [x] T030 [US4] Create Ch4 Quiz (`docs/chapter4/quiz.md`) & Populate Data

## Phase 6: Chapter 5 - VLA & Capstone (User Story 1 & 4)

**Goal**: Module 4 subfolder, Quiz.

- [x] T031 [US1] Create Module 4 Overview (`docs/chapter5/module4-overview.md`)
- [x] T032 [US1] Create Ch5 Lesson 1: Kinematics/Locomotion (`docs/chapter5/module4/lesson1.md`)
- [x] T033 [US1] Create Ch5 Lesson 2: Manipulation/HRI (`docs/chapter5/module4/lesson2.md`)
- [x] T034 [US1] Create Ch5 Lesson 3: Voice/Planning (`docs/chapter5/module4/lesson3.md`)
- [x] T035 [US1] Create Ch5 Lesson 4: Capstone (`docs/chapter5/module4/lesson4.md`)
- [x] T036 [US4] Create Ch5 Quiz (`docs/chapter5/quiz.md`) & Populate Data

## Phase 7: Chapter 6 - Conversational Robotics (User Story 1)

**Goal**: Direct lessons (3), No Quiz.

- [x] T037 [US1] Create Ch6 Overview (`docs/chapter6/overview.md`)
- [x] T038 [US1] Create Ch6 Lesson 1: GPT Integration (`docs/chapter6/lesson1.md`)
- [x] T039 [US1] Create Ch6 Lesson 2: Speech/NLU (`docs/chapter6/lesson2.md`)
- [x] T040 [US1] Create Ch6 Lesson 3: Multi-modal (`docs/chapter6/lesson3.md`)

## Phase 8: Chapter 7 - Assessments (User Story 1 & 4)

**Goal**: Direct assessment files, Quiz.

- [x] T041 [US1] Create Ch7 Overview (`docs/chapter7/overview.md`)
- [x] T042 [US1] Create Assessment 1: ROS 2 (`docs/chapter7/assessment1.md`)
- [x] T043 [US1] Create Assessment 2: Simulation (`docs/chapter7/assessment2.md`)
- [x] T044 [US1] Create Assessment 3: Isaac (`docs/chapter7/assessment3.md`)
- [x] T045 [US1] Create Assessment 4: VLA Capstone (`docs/chapter7/assessment4.md`)
- [x] T046 [US4] Create Ch7 Quiz (`docs/chapter7/quiz.md`) & Populate Data

## Phase 9: Hardware & Polish (User Story 3)

- [x] T047 [US3] Update Hardware Guide (`docs/hardware.md`) - Unitree/Jetson only.
- [x] T048 Verify LevelToggle on all 27 new files.
- [x] T049 Verify no Limx TRON2 references.
