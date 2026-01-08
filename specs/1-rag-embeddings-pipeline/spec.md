# Feature Specification: RAG Embeddings Pipeline

**Feature Branch**: `1-rag-embeddings-pipeline`
**Created**: 2026-01-04
**Status**: Draft
**Input**: User description: "Deploy website URLs, generate embeddings, and store them in a vector database for the RAG Chatbot

Target audience: Developers and educators building or using the Physical AI textbook's interactive features

Focus: Seamless deployment of the Docusaurus site to obtain accessible URLs, efficient generation of embeddings using Cohere models from the book's Markdown content, and secure storage in Qdrant Cloud Free Tier for semantic search readiness

Success criteria:
- Docusaurus site deployed to GitHub Pages or similar, with verifiable public URLs for all chapters/modules/lessons
- Embeddings generated for 100% of book content (chunked Markdown files), using Cohere's free-tier models (e.g., embed-multilingual-v3.0) for high-quality vector representations
- All embeddings uploaded to Qdrant with metadata (e.g., file path, section, title, tags for levels/chapters), enabling hybrid search testing
- Pipeline tested end-to-end: From content download/parsing to Qdrant query, confirming >90% recall on sample book queries
- Documentation includes setup steps, .env examples for keys, and error-handling code

Constraints:
- Use free tiers only: Cohere free API, Qdrant Cloud Free Tier (limited collections/points)
- Embeddings are stored and indexed in Qdrant successfully
- Deployment: Vercel URLs ; no paid hosting
- Timeline: Complete within 3-6 tasks
- Sources/Tools: Phase 1 Docusaurus repo; Python for scripting (FastAPI prep); no additional paid APIs

Not building:
- Full RAG query/response logic (reserved for later specs)
- Frontend integration or UI elements (focus on backend data prep)
- Personalization/auth features (e.g., Better-Auth/Neon)
- Advanced MoE agents/skills (Claude setup for Phase 2 overall)"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Deploy Docusaurus Site for Content Access (Priority: P1)

As a developer or educator, I want to access the Physical AI textbook content via public URLs so that I can reference specific chapters, modules, and lessons in the RAG system.

**Why this priority**: Without accessible URLs, the entire RAG system cannot function since it needs public content to generate embeddings from.

**Independent Test**: Can be fully tested by deploying the Docusaurus site to GitHub Pages and verifying that all chapters/modules/lessons have accessible public URLs.

**Acceptance Scenarios**:

1. **Given** a completed Docusaurus site, **When** I visit the deployed site, **Then** I can access all chapters, modules, and lessons via public URLs
2. **Given** the deployed Docusaurus site, **When** I share a URL to a specific lesson, **Then** the recipient can access that content without authentication

---

### User Story 2 - Generate Embeddings from Book Content (Priority: P1)

As a system administrator, I want to generate high-quality embeddings from all book content so that the RAG system can perform semantic searches effectively.

**Why this priority**: This is the core functionality that enables semantic search capabilities for the RAG chatbot.

**Independent Test**: Can be fully tested by processing all Markdown content files and generating embeddings that represent the semantic meaning of the content.

**Acceptance Scenarios**:

1. **Given** book content in Markdown format, **When** I run the embedding generation process, **Then** embeddings are created for 100% of the content
2. **Given** the embedding generation process, **When** it processes chunked Markdown files, **Then** high-quality vector representations are created using Cohere's embed-english-v3.0 model

---

### User Story 3 - Store Embeddings in Vector Database (Priority: P1)

As a developer building the RAG system, I want to store embeddings in Qdrant with proper metadata so that semantic search can be performed efficiently.

**Why this priority**: Without proper storage and indexing of embeddings, the RAG system cannot perform semantic searches.

**Independent Test**: Can be fully tested by uploading embeddings to Qdrant and verifying they can be retrieved with proper metadata.

**Acceptance Scenarios**:

1. **Given** generated embeddings with metadata, **When** I upload them to Qdrant, **Then** they are stored successfully with file path, section, title, and level tags
2. **Given** embeddings stored in Qdrant, **When** I query the database, **Then** I can retrieve relevant content based on semantic similarity

---

### User Story 4 - End-to-End Pipeline Testing (Priority: P2)

As a quality assurance engineer, I want to test the complete pipeline from content parsing to Qdrant query so that I can verify the system meets the required recall threshold.

**Why this priority**: This ensures the entire system works as intended and meets the performance requirements.

**Independent Test**: Can be fully tested by running a complete end-to-end test from content download to Qdrant query and measuring recall performance.

**Acceptance Scenarios**:

1. **Given** the complete pipeline, **When** I run end-to-end tests with sample book queries, **Then** the system achieves >90% recall on relevant content
2. **Given** the pipeline components, **When** I test error handling, **Then** appropriate error messages and fallbacks are provided

---

### Edge Cases

- What happens when Cohere API rate limits are reached during embedding generation?
- How does the system handle corrupted or malformed Markdown files during content parsing?
- What occurs when Qdrant Cloud Free Tier storage limits are approached?
- How does the system handle changes to the Docusaurus site structure after initial deployment?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST deploy the Docusaurus site to Vercel URLs or similar service with publicly accessible URLs for all chapters, modules, and lessons
- **FR-002**: System MUST generate embeddings for 100% of book content (chunked Markdown files) using an appropriate embedding model
- **FR-003**: System MUST store embeddings in a vector database with metadata including file path, section, title, and level tags
- **FR-004**: System MUST provide documentation with setup steps, .env examples for API keys, and error-handling code
- **FR-005**: System MUST test the end-to-end pipeline and confirm >90% recall on sample book queries
- **FR-006**: System MUST handle rate limits and usage constraints of free-tier services appropriately with retry mechanisms and graceful degradation
- **FR-007**: System MUST handle error conditions gracefully with appropriate logging and fallback mechanisms

### Key Entities

- **Book Content**: Represents the Physical AI textbook content in Markdown format, organized into chapters, modules, and lessons with hierarchical structure
- **Embeddings**: Vector representations of content chunks that capture semantic meaning for similarity search, containing metadata for content identification
- **Vector Database**: Collection storing embeddings with metadata for efficient semantic search capabilities
- **Content URLs**: Publicly accessible URLs pointing to specific chapters, modules, and lessons in the deployed Docusaurus site

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Docusaurus site is successfully deployed to GitHub Pages with verifiable public URLs for all chapters, modules, and lessons
- **SC-002**: Embeddings are generated for 100% of book content using an appropriate embedding service
- **SC-003**: All embeddings are successfully uploaded to the vector database with complete metadata (file path, section, title, level tags)
- **SC-004**: End-to-end pipeline testing confirms >90% recall on sample book queries from content download to vector database query
- **SC-005**: Complete documentation is provided including setup steps, .env examples, and error-handling code