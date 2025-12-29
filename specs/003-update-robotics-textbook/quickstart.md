# Quickstart: Update Physical AI & Humanoid Robotics Textbook

**Feature**: `003-update-robotics-textbook`
**Date**: 2025-12-29

## Overview
This feature enriches the Phase 1 Docusaurus textbook with deep, hardware-aligned content (Limx TRON2), multi-level toggles, and dynamic quizzes. It follows a defensive programming approach.

## Setup Instructions

1.  **Prerequisites**:
    *   Node.js 18+ installed.
    *   Docusaurus project initialized in `Docusaurus-frontend/`.

2.  **Installation**:
    ```bash
    cd Docusaurus-frontend
    npm install
    ```

3.  **Running Locally**:
    ```bash
    npm start
    ```
    Access at `http://localhost:3000`.

## Feature Usage

### 1. Multi-Level Content
*   Navigate to any Lesson page (e.g., `/docs/chapter2/module1/lesson1`).
*   Locate the **Level Toggle** at the top right.
*   Select a level (e.g., "Research").
*   Verify content sections tagged for that level appear.
*   Refresh page; selection should persist.

### 2. Module Overviews
*   Navigate to a Chapter (e.g., Chapter 2).
*   Click the **Module Overview** link in the sidebar.
*   Read the "Defensive Programming Notes" and "Hardware Implications" sections.

### 3. Dynamic Quizzes
*   Navigate to `/docs/chapterX/quiz` (where X is 1-7).
*   Answer questions.
*   Review immediate feedback and final score.

## Development Guide

### Adding a New Lesson
1.  Create `.md` file in `docs/chapterX/moduleY/`.
2.  Add frontmatter:
    ```yaml
    ---
    title: My Lesson
    tags: [ros2]
    level: [beginner, normal, pro, advanced, research]
    ---
    ```
3.  Use the `LevelContent` component (if implemented) or structural markers to denote level-specific text.

### Editing Hardware Specs
*   Edit `docs/hardware.md`.
*   Ensure Limx TRON2 specs (7-DoF, 10kg payload) are accurate.
*   **WARNING**: Do not change VRAM safety ranges without ADR approval.

## Troubleshooting

*   **Toggle not working**: Check browser console for `localStorage` errors. Ensure JavaScript is enabled.
*   **Build fails**: Verify all React components are valid and imported correctly in MDX.
*   **Content missing**: Check frontmatter `sidebar_position` or `level` tags.
