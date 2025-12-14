# Data Model for "Physical AI & Humanoid Robotics" Textbook

This data model describes the key entities that constitute the "Physical AI & Humanoid Robotics" textbook content, primarily focusing on their structure and relationships for content generation and organization within the Docusaurus framework.

## Entities:

### 1. Textbook Content (Root Entity)

Represents the entire body of knowledge of the textbook. This is an aggregation of chapters and metadata.

*   **Attributes**:
    *   `id`: Unique identifier for the textbook (e.g., "physical-ai-robotics-textbook").
    *   `title`: "Physical AI & Humanoid Robotics"
    *   `description`: Overall description of the textbook.
    *   `chapters`: A collection of `Chapter` entities.
    *   `metadata`: Global metadata for RAG (e.g., `frontmatter` for search index).
    *   `config`: Configuration settings for Docusaurus (e.g., `docusaurus.config.ts`, `sidebars.ts`).

### 2. Chapter

A major organizational unit within the textbook, containing an introduction, multiple lessons, and an end-of-chapter summary/assessment. Maps to a top-level directory and an `_category_.json` in Docusaurus.

*   **Attributes**:
    *   `id`: Unique identifier (e.g., "chapter-1-intro").
    *   `title`: Title of the chapter (e.g., "Introduction to Physical AI").
    *   `slug`: URL-friendly identifier.
    *   `order`: Numerical order for display.
    *   `learningObjectives`: List of learning outcomes for the chapter, tiered by audience level (Beginner, Intermediate, Expert, Researcher).
    *   `introduction`: Markdown content for the chapter's introduction, setting context and relevance.
    *   `lessons`: A collection of `Lesson` entities.
    *   `assessments`: Markdown content for chapter-end assessments (e.g., quizzes, exercises).
    *   `keyTakeaways`: Summary of main points.
    *   `capstoneIntegrationLink`: Link or reference to how this chapter integrates with the capstone project.
    *   `frontmatter`: Metadata for Docusaurus and RAG (e.g., `title`, `tags`, `description`).
    *   `filepath`: Relative path to the Markdown file (e.g., `docs/chapter1/index.md`).

### 3. Lesson

A sub-unit within a chapter, focusing on specific topics. Maps to a Markdown file within a chapter's directory in Docusaurus.

*   **Attributes**:
    *   `id`: Unique identifier (e.g., "chapter-1-lesson-1-what-is-physical-ai").
    *   `title`: Title of the lesson.
    *   `slug`: URL-friendly identifier.
    *   `order`: Numerical order for display within the chapter.
    *   `theory`: Markdown content explaining core concepts.
    *   `handsOnPractice`: Markdown content including code snippets (Python/ROS 2), simulation guides (Gazebo/Unity/Isaac), and exercises.
    *   `advancedTopics`: Markdown content for in-depth analysis and complex concepts.
    *   `researchInsights`: Markdown content discussing current research, open problems, and future directions.
    *   `codeSnippets`: Embedded code blocks with language highlighting.
    *   `diagramDescriptions`: Textual descriptions for diagrams (to be rendered via Mermaid/PlantUML or as images).
    *   `caseStudies`: Real-world examples demonstrating concepts.
    *   `quizzes`: Interactive elements for self-assessment (multiple-choice, short-answer questions with solutions/explanations).
    *   `summaries`: Auto-generated summaries of the lesson.
    *   `boosters`: Learning tips or heuristic aphorisms.
    *   `troubleshooting`: Common issues and solutions.
    *   `frontmatter`: Metadata for Docusaurus and RAG.
    *   `filepath`: Relative path to the Markdown file (e.g., `docs/chapter1/lesson1.md`).

### 4. User Profile (Implicit Entity for Personalization)

Represents the user's background and preferences, used to tailor content delivery (e.g., skipping basic explanations for experts). This is an implicit entity, not stored within the textbook content itself, but rather used by the Docusaurus frontend/backend (Phase 2) for personalization.

*   **Attributes**:
    *   `pythonProficiency`: (e.g., Beginner, Intermediate, Expert).
    *   `roboticsExperience`: (e.g., None, Basic, Advanced).
    *   `hardwareAccess`: (e.g., None, Jetson Orin, RTX GPU).
    *   `preferredLanguage`: (e.g., English, Urdu).
    *   `sessionHistory`: (e.g., Chapters completed, quizzes taken).
