# Contracts: Update Physical AI & Humanoid Robotics Textbook

**Feature**: `003-update-robotics-textbook`
**Date**: 2025-12-29

## Internal Component Contracts

Since this is a Docusaurus static site update, "API Contracts" refer to the interface between MDX content and React Components.

### 1. LevelToggle Component

**Usage**: `<LevelToggle />` (Placed globally in Layout or per page)

**Props**: None (Reads/Writes `localStorage`)

**Events**:
- `onLevelChange(newLevel: string)`: Dispatched to update `LevelContent` visibility.

### 2. LevelContent Component (Optional Helper)

**Usage**: Wraps content specific to a level.

```jsx
<LevelContent levels={['research', 'advanced']}>
  ### Advanced Kinematics
  ...
</LevelContent>
```

**Props**:
- `levels`: `string[]` - List of levels where this content is visible.

### 3. Quiz Component

**Usage**: `<Quiz chapterId="1" />`

**Props**:
- `chapterId`: `string` - ID to load specific questions from `data.ts`.

**Behavior**:
- Loads questions asynchronously (simulated).
- Renders `QuizUI`.

## Data Contracts (Content Standards)

**MDX Files**:
- MUST have `h2` headings for "Defensive Programming", "Hardware Implications" in Overviews.
- MUST contain `<code>` blocks with explicit type hints.

**Hardware Specs**:
- Limx TRON2: `7-DoF`, `70cm reach`, `10kg payload`.
- VRAM Safe Range: `12-24 GB`.
