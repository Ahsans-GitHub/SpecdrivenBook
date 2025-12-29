# Data Model: Update Physical AI & Humanoid Robotics Textbook

**Feature**: `003-update-robotics-textbook`
**Date**: 2025-12-29

## Entities

### User Preferences (Client-Side)
Stored in Browser `localStorage`.

| Key | Type | Valid Values | Default | Description |
|-----|------|--------------|---------|-------------|
| `user_skill_level` | string | `beginner`, `normal`, `pro`, `advanced`, `research` | `normal` | The user's selected difficulty level. |
| `quiz_scores` | JSON string | Map of `chapter_id` -> `{ score: number, date: string }` | `{}` | Local record of quiz performance. |

### Content Metadata (Frontmatter)
YAML Frontmatter in MDX files.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `title` | string | Yes | Title of the lesson/overview. |
| `sidebar_label` | string | No | Short title for navigation. |
| `tags` | string[] | No | Keywords for RAG/Search (e.g., `ros2`, `tron2`). |
| `level` | string[] | Yes | Supported levels in this file (usually all 5). |
| `description` | string | Yes | Meta description for SEO and RAG summary. |

### Quiz Data Structure (Internal Component Data)
TypeScript Interface for `data.ts` in Quiz component.

```typescript
interface Question {
  id: string;
  text: string;
  options: string[]; // List of possible answers
  correctIndex: number;
  feedback: {
    correct: string; // Explanation why right
    incorrect: string; // Explanation why wrong
  };
}

interface QuizData {
  chapterId: string;
  questions: Question[];
}
```

## Validation Rules

- **Level Toggle**:
  - Must default to `normal` if key is missing or invalid.
  - Updates trigger immediate re-render of content visibility.
- **Quiz**:
  - `correctIndex` must be within `options` bounds.
  - Feedback must be non-empty strings.

## State Transitions

- **Level Selection**:
  - `User clicks Toggle(L)` -> `localStorage.setItem('user_skill_level', L)` -> `Component State Updates` -> `UI Renders Level L`.
- **Quiz Flow**:
  - `Start` -> `Q1 Display` -> `User Selects` -> `Feedback Display` -> `Next` -> ... -> `Score Display`.
