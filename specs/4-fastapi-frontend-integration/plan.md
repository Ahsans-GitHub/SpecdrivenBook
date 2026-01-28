# Implementation Plan: FastAPI Frontend Integration

**Feature**: FastAPI Frontend Integration
**Branch**: 4-fastapi-frontend-integration
**Created**: 2026-01-16
**Status**: Draft

## Technical Context

This plan outlines the integration of the FastAPI backend with a frontend interface for the Physical AI textbook RAG Chatbot. The implementation will establish a local connection between the FastAPI backend agent and the frontend (Docusaurus/React-based book site), enabling the embedded chatbot to handle user queries, retrieve from Qdrant, generate responses via the agent, and display with a futuristic UI feel using ChatKit SDK.

### Current State
- FastAPI backend exists from spec 3 (RAG-backend/)
- Need to establish CORS support for frontend communication
- Need to create frontend components for chatbot integration
- Need to implement user-selected text context handling
- Need to integrate ChatKit SDK for enhanced UI

### Target State
- Local connection established between frontend and backend
- Chatbot widget embedded in book site
- Multi-turn conversation support with history
- Selected text context handling
- Futuristic UI with ChatKit SDK integration

### Technology Stack
- **Backend**: Python, FastAPI, Uvicorn
- **Frontend**: JavaScript/React, Docusaurus
- **Dependencies**: uv for Python, npm/yarn for JS
- **Integration**: ChatKit SDK
- **Databases**: Neon Postgres (for history), Qdrant (for retrieval)

## Constitution Check

### Compliance Verification

**Defensive Programming & Input Parana**:
- All user inputs will be validated and sanitized
- Query length will be limited to prevent buffer overflows
- Selected text context will be validated for length and content

**Fail-Secure Error Handling**:
- API endpoints will have proper error handling
- Frontend will display user-friendly error messages
- Network failures will be handled gracefully

**Tech Stack Integrity**:
- Using FastAPI for backend with proper typing
- Using React/Docusaurus for frontend
- Following security best practices for API communication

## Phase 0: Research & Discovery

### Research Tasks

#### 0.1 CORS Configuration Research
**Decision**: Configure FastAPI CORS middleware for local development
**Rationale**: Enable communication between frontend (localhost:3000) and backend (localhost:8000)
**Alternatives considered**: Different CORS configurations, proxy setup
**Outcome**: Standard CORS middleware with wildcard for local development

#### 0.2 ChatKit SDK Integration Research
**Decision**: Integrate ChatKit SDK for enhanced UI elements
**Rationale**: Provides adaptive prompts, real-time feedback, and visualizations
**Alternatives considered**: Custom chat UI, react-chatbot-kit
**Outcome**: ChatKit SDK chosen for futuristic AI feel

#### 0.3 Multi-turn Conversation Strategy
**Decision**: Implement conversation history using Neon Postgres if available, otherwise client-side fallback
**Rationale**: Maintains context across exchanges for coherent responses while handling cases where database is not available
**Alternatives considered**: Client-side storage only (localStorage/sessionStorage), in-memory storage
**Outcome**: Hybrid approach with server-side preferred and client-side fallback for resilience

#### 0.4 Selected Text Context Handling
**Decision**: Capture selected text and pass as context parameter in API calls
**Rationale**: Enables context-aware queries based on highlighted content with clean separation of concerns
**Alternatives considered**: Different context passing mechanisms, real-time context extraction
**Outcome**: Query parameter approach for simplicity and clarity with proper validation

## Phase 1: Design & Contracts

### 1.1 Data Model Design

#### Query Entity
- **Fields**:
  - `query_text`: string (max 2000 chars) - Main query from user
  - `selected_context`: string (max 5000 chars, optional) - Selected text from book content
  - `conversation_id`: string (UUID, optional) - For multi-turn conversations
  - `timestamp`: datetime - When query was submitted
  - `metadata`: object (optional) - Additional context like source page, user preferences

#### Response Entity
- **Fields**:
  - `response_text`: string - Generated response from agent
  - `sources`: array of strings - Citations and references
  - `conversation_id`: string (UUID) - Associated conversation
  - `timestamp`: datetime - When response was generated
  - `metadata`: object - Confidence scores, processing time, retrieved chunks count

#### Conversation Session Entity
- **Fields**:
  - `session_id`: string (UUID) - Unique session identifier
  - `user_id`: string (optional) - User identifier if available
  - `created_at`: datetime - Session creation time
  - `last_interaction`: datetime - Last activity in session
  - `active`: boolean - Whether session is currently active

#### Selected Text Context Entity
- **Fields**:
  - `context_id`: string (UUID) - Unique context identifier
  - `text_content`: string - The selected text
  - `source_page`: string - Page/chapter where text was selected
  - `selection_bounds`: object - Start/end positions in text
  - `timestamp`: datetime - When selection was captured

### 1.2 API Contract Design

#### POST /chat
**Description**: Process user query and return AI-generated response
**Request Body**:
```json
{
  "query": "string (max 2000)",
  "selected_text": "string (max 5000, optional)",
  "session_id": "string (UUID, optional)",
  "top_k": "integer (default: 3)",
  "min_similarity": "float (default: 0.4)",
  "temperature": "float (default: 0.7)"
}
```
**Response**:
```json
{
  "response": "string",
  "sources": ["string"],
  "session_id": "string (UUID)",
  "metadata": {
    "confidence_score": "float (0.0-1.0)",
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
  "status": "string",
  "timestamp": "string",
  "services": {
    "qdrant": "string (connected/disconnected)",
    "postgres": "string (connected/disconnected)"
  }
}
```

### 1.3 Quickstart Guide

#### Backend Setup
1. Navigate to RAG-backend directory
2. Install dependencies with `uv`
3. Set up environment variables (Neon Postgres, Qdrant, OpenAI keys)
4. Run with `uvicorn main:app --reload --host 0.0.0.0 --port 8000`

#### Frontend Setup
1. Navigate to Docusaurus-frontend directory
2. Install dependencies with `npm install`
3. Install ChatKit SDK: `npm install @chatscope/chat-ui-kit-react @chatscope/chat-ui-kit-styles`
4. Configure API endpoints to point to backend
5. Run development server: `npm run dev`

## Phase 2: Implementation Strategy

### 2.1 Backend Modifications

#### 2.1.1 CORS Middleware Addition
- Add CORS middleware to FastAPI app
- Configure for local development (localhost:3000)
- Include credentials support for session handling

#### 2.1.2 Enhanced /chat Endpoint
- Accept selected text context parameter
- Implement conversation history tracking
- Add metadata to response
- Include source citations

#### 2.1.3 Input Validation Layer
- Implement Pydantic models for request/response validation
- Add length limits for query and selected text
- Sanitize inputs to prevent injection attacks

### 2.2 Frontend Components

#### 2.2.1 Chat Widget Component
- Implement chat interface with message history
- Add ability to capture selected text
- Integrate with ChatKit SDK for enhanced UI
- Display responses with citations

#### 2.2.2 API Service Layer
- Create service for communicating with backend
- Handle loading states and error states
- Manage conversation context
- Implement retry logic for failed requests

#### 2.2.3 Selected Text Handler
- Add event listeners for text selection
- Extract and format selected content
- Pass context to backend with queries

### 2.3 Integration Points

#### 2.3.1 Frontend-Backend Communication
- Establish API endpoints for chat functionality
- Implement proper error handling
- Add loading indicators for real-time feedback

#### 2.3.2 ChatKit SDK Integration
- Configure adaptive prompts
- Implement real-time feedback mechanisms
- Add visualization elements based on metadata

## Phase 3: Testing Strategy

### 3.1 Unit Tests
- Test backend endpoint functionality
- Validate input sanitization
- Test error handling scenarios

### 3.2 Integration Tests
- Test frontend-backend communication
- Verify selected text context handling
- Validate multi-turn conversation flow

### 3.3 End-to-End Tests
- Test complete user flows
- Verify ChatKit UI integration
- Validate error recovery scenarios

## Phase 4: Deployment & Documentation

### 4.1 Local Setup Instructions
- Detailed setup guide for developers
- Environment configuration steps
- Troubleshooting common issues

### 4.2 Error Handling Documentation
- Network failure scenarios
- Invalid input handling
- Graceful degradation procedures

## Success Criteria Verification

- [ ] Local connection established between frontend and backend
- [ ] API calls to /chat endpoint successful
- [ ] Selected text context passed to backend
- [ ] Responses rendered with sources and metadata
- [ ] Multi-turn conversations maintain context
- [ ] ChatKit SDK provides adaptive UI elements
- [ ] Error handling implemented for network failures
- [ ] Setup documentation complete and accurate

## Risk Assessment

### Technical Risks
- CORS configuration issues during local development
- ChatKit SDK integration complexity
- Database connection stability

### Mitigation Strategies
- Thorough testing of CORS settings
- Progressive enhancement approach for ChatKit features
- Connection pooling and retry mechanisms for databases