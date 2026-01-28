# Implementation Plan: RAG Agent Backend

**Feature**: RAG Agent Backend Implementation
**Branch**: 3-rag-agent-backend
**Created**: 2026-01-16
**Status**: Draft

## Technical Context

This plan outlines the implementation of the backend RAG agent system that integrates with OpenAI Agents SDK, Cohere embeddings, Qdrant vector database, and Neon Postgres for conversation history. The system will handle user queries, retrieve relevant content from the Physical AI textbook, and generate contextual responses with source citations. It will support selected text context biasing, multi-turn conversations, and adaptive responses.

### Current State
- Basic retrieval system exists (Qdrant integration)
- Cohere embeddings are available
- OpenAI/alternative LLM integration exists
- Need to implement RAG agent with conversation management
- Need to add selected text context biasing

### Target State
- RAG agent processes queries with retrieved context
- Selected text biases retrieval toward relevant content
- Multi-turn conversations maintain context
- Adaptive responses based on conversation history
- Proper source citations with metadata
- Conversation history stored in Neon Postgres

### Technology Stack
- **Backend**: Python, FastAPI, OpenAI/DeepSeek/Alternative LLMs
- **Vector DB**: Qdrant for content retrieval
- **Embeddings**: Cohere for vector generation
- **History**: Neon Postgres for conversation persistence
- **Agents**: OpenAI Agents SDK (if available) or custom implementation

## Constitution Check

### Compliance Verification

**Defensive Programming & Input Parana**:
- All user inputs validated and sanitized
- Query length limits enforced (max 2000 chars)
- Selected text context validated for length and content
- Parameter ranges validated (top_k, similarity, temperature)

**Fail-Secure Error Handling**:
- API endpoints have proper error handling
- Frontend displays user-friendly error messages
- Network failures handled gracefully with retries

**Tech Stack Integrity**:
- Using FastAPI with proper typing
- Following security best practices for API communication
- Proper dependency management with uv

## Phase 0: Research & Discovery

### Research Tasks

#### 0.1 OpenAI Agents SDK Integration Research
**Decision**: Determine if OpenAI Agents SDK is available, fallback to custom implementation if not
**Rationale**: Need to decide on agent implementation approach based on SDK availability
**Alternatives considered**: Custom agent implementation, LangChain, CrewAI
**Outcome**: Use OpenAI Agents SDK if available, fallback to custom implementation

#### 0.2 Selected Text Biasing Strategy Research
**Decision**: Implement selected text as additional context and query modifier
**Rationale**: Bias retrieval toward content semantically related to selected text
**Alternatives considered**: Different biasing approaches, reranking strategies
**Outcome**: Combine query with selected text context for enhanced retrieval

#### 0.3 Conversation Management Research
**Decision**: Use session-based conversation management with Neon Postgres storage
**Rationale**: Need persistent conversation history for multi-turn interactions
**Alternatives considered**: In-memory storage, file-based storage
**Outcome**: Neon Postgres for persistent, scalable conversation history

#### 0.4 LLM Provider Strategy Research
**Decision**: Support multiple LLM providers with fallback mechanisms
**Rationale**: Need to handle API availability and cost considerations
**Alternatives considered**: Single provider vs. multi-provider approach
**Outcome**: Multiple provider support with intelligent fallbacks

## Phase 1: Design & Contracts

### 1.1 Data Model Design

#### ChatRequest Entity
- **Fields**:
  - `query`: string (max 2000 chars) - User's main query
  - `selected_text`: string (max 2000 chars, optional) - Selected text context
  - `session_id`: string (UUID, optional) - Conversation session identifier
  - `top_k`: integer (1-20) - Number of results to retrieve (default: 3)
  - `min_similarity`: float (0.0-1.0) - Minimum similarity threshold (default: 0.4)
  - `temperature`: float (0.0-2.0) - Generation temperature (default: 0.7)

#### ChatResponse Entity
- **Fields**:
  - `response`: string - Generated response from the agent
  - `sources`: array of RetrievedChunk - Retrieved content with citations
  - `session_id`: string (UUID) - Session identifier
  - `metadata`: object - Additional metadata for UI enhancements
  - `retrieval_time`: float - Time taken for retrieval phase
  - `generation_time`: float - Time taken for generation phase

#### RetrievedChunk Entity
- **Fields**:
  - `id`: string - Unique identifier for the chunk
  - `title`: string - Title of the source document
  - `url`: string - URL of the source
  - `content`: string - Content of the retrieved chunk
  - `section`: string - Section of the textbook
  - `tags`: array of strings - Associated tags
  - `score`: float - Relevance score
  - `similarity`: float - Similarity score to the query

### 1.2 API Contract Design

#### POST /chat
**Description**: Process user query through RAG agent with optional selected text context
**Request Body**:
```json
{
  "query": "string (max 2000)",
  "selected_text": "string (max 2000, optional)",
  "session_id": "string (UUID, optional)",
  "top_k": "integer (1-20, default: 3)",
  "min_similarity": "float (0.0-1.0, default: 0.4)",
  "temperature": "float (0.0-2.0, default: 0.7)"
}
```
**Response**:
```json
{
  "response": "string",
  "sources": [
    {
      "id": "string",
      "title": "string",
      "url": "string",
      "content": "string",
      "section": "string",
      "tags": ["string"],
      "score": "float",
      "similarity": "float"
    }
  ],
  "session_id": "string (UUID)",
  "metadata": {
    "adaptive_prompt_hint": "string",
    "confidence_score": "float",
    "retrieval_success": "boolean",
    "adaptive_prompts": ["string"],
    "ui_enhancement_metadata": {
      "has_visualization_opportunities": "boolean",
      "suggest_follow_up_questions": "boolean",
      "suggest_related_topics": "boolean",
      "suggest_content_format": "string"
    }
  },
  "retrieval_time": "float",
  "generation_time": "float"
}
```
**Error Responses**:
- 400: Invalid input format or length
- 422: Unprocessable entity (validation errors)
- 500: Internal server error

#### GET /health
**Description**: Check backend health status
**Response**:
```json
{
  "status": "string (healthy|degraded|unhealthy)",
  "timestamp": "string (ISO 8601)",
  "services": {
    "qdrant": "string (connected|disconnected)",
    "llm_provider": "string (connected|disconnected)",
    "postgres": "string (connected|disconnected)"
  }
}
```

### 1.3 Quickstart Guide

#### Backend Setup
1. Navigate to RAG-backend directory
2. Install dependencies with `uv pip install -e .`
3. Set up environment variables (Qdrant, Cohere, LLM provider keys)
4. Run with `uvicorn main:app --reload`

#### Agent Integration
1. Verify Qdrant connection to textbook content
2. Test retrieval with sample queries
3. Validate LLM provider connectivity
4. Test end-to-end RAG flow with citations

## Phase 2: Implementation Strategy

### 2.1 Agent Core Implementation

#### 2.1.1 RAG Agent Class
- Implement core agent logic with retrieval and generation
- Add selected text context biasing functionality
- Create conversation state management
- Implement error handling and fallback mechanisms

#### 2.1.2 Retrieval Enhancement
- Update retrieval system to incorporate selected text
- Implement semantic biasing toward selected context
- Add confidence scoring for retrieved results
- Create metadata enrichment for UI features

#### 2.1.3 Generation Enhancement
- Integrate retrieved context into LLM prompts
- Add citation generation for sources
- Implement adaptive response formatting
- Create metadata for UI enhancements

### 2.2 Backend API Implementation

#### 2.2.1 Chat Endpoint Enhancement
- Accept selected_text parameter in requests
- Pass context to RAG agent
- Return enriched responses with sources and metadata
- Implement session management

#### 2.2.2 Session Management
- Create conversation session tracking
- Implement history persistence with Neon Postgres
- Add session state management (ACTIVE, INACTIVE, ENDED)
- Create session cleanup for expired sessions

## Phase 3: Testing Strategy

### 3.1 Unit Tests
- Test individual agent components with mocked dependencies
- Validate retrieval accuracy with selected text context
- Verify conversation state management
- Test error handling scenarios

### 3.2 Integration Tests
- Test end-to-end RAG flow with real dependencies
- Validate selected text biasing effectiveness
- Verify multi-turn conversation context maintenance
- Test API endpoint functionality

### 3.3 End-to-End Tests
- Test complete user flows with realistic scenarios
- Validate response quality and citation accuracy
- Test performance under various load conditions
- Verify error recovery and fallback mechanisms

## Phase 4: Deployment & Documentation

### 4.1 Local Deployment
- Document local setup with environment configuration
- Create development environment guidelines
- Add troubleshooting for common deployment issues

### 4.2 Production Considerations
- Outline production deployment requirements
- Document scaling considerations
- Add monitoring and observability setup

## Success Criteria Verification

- [ ] Query responses generated within 10 seconds with >90% success rate
- [ ] Retrieved content has >85% relevance to user queries
- [ ] Selected text context biasing improves response relevance by >20%
- [ ] Multi-turn conversations maintain coherence across 5+ exchanges
- [ ] Source citations are accurate and complete in 95%+ of responses
- [ ] System handles 100+ concurrent sessions without performance degradation
- [ ] Error handling manages 99%+ of failure scenarios gracefully
- [ ] API endpoints maintain 99.5%+ uptime during testing