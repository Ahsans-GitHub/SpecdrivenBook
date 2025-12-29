# Research: Update Physical AI & Humanoid Robotics Textbook

**Feature**: `003-update-robotics-textbook`
**Date**: 2025-12-29

## Research Tasks & Findings

### Task 1: Docusaurus MDX Component Integration for Dynamic Toggles
- **Decision**: Use Docusaurus `<BrowserOnly>` or `useEffect` for `LevelToggle` component to access `localStorage` safely during server-side rendering (SSR).
- **Rationale**: Docusaurus builds statically; direct `localStorage` access breaks build. `BrowserOnly` ensures code runs only on client.
- **Alternatives**: Custom hydration scripts (too complex/brittle).

### Task 2: Persistence Strategy for Level Selection
- **Decision**: `localStorage` key `user_skill_level` (Values: 'beginner', 'normal', 'pro', 'advanced', 'research').
- **Rationale**: Simple, persistent across sessions, GDPR compliant (functional preference).
- **Alternatives**: Cookies (overkill), URL params (not persistent across navigation).

### Task 3: Limx TRON2 Specifications
- **Decision**: Use supplied specs: 7-DoF arms, 70cm reach, 10kg payload, modular biped/wheeled.
- **Rationale**: Provided in prompt/constitution.
- **Verification**: Ensure simulation config examples (Isaac Sim) match these physical limits to prevent "sim-to-real" gaps.

### Task 4: Defensive Programming Patterns in Python/ROS 2
- **Decision**: Use `pydantic` for data validation where applicable, or standard `if/raise ValueError` blocks in educational code snippets.
- **Rationale**: Teaches "Paranoia" principle without requiring heavy external dependencies in every snippet.
- **Example**:
  ```python
  def set_velocity(v: float):
      if not (0.0 <= v <= 1.5):
          raise ValueError(f"Velocity {v} out of safe range (0.0-1.5 m/s)")
  ```

### Task 5: RAG Metadata Strategy
- **Decision**: Frontmatter `tags` and `level` fields.
- **Rationale**: Compatible with standard Markdown parsers and Docusaurus.
- **Schema**:
  ```yaml
  ---
  title: Lesson 1
  tags: [ros2, beginner, hardware-check]
  level: [beginner, normal, pro, advanced, research]
  ---
  ```

## Unknowns Resolved
- **Persistence**: `localStorage` confirmed suitable.
- **Hardware Specs**: Limx TRON2 specs taken as canonical from prompt.
- **SSR Safety**: `BrowserOnly` wrapper identified as solution.

## Best Practices
- **MDX**: Keep logic out of MDX; import components.
- **Accessibility**: Ensure toggles are keyboard accessible.
- **Fail-Closed**: If JS fails, show "Normal" content (static default in MDX).
