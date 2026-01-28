"""
Data models for the RAG Agent Backend
This module defines Pydantic models for API requests, responses, and internal data structures.
"""

from pydantic import BaseModel, Field, validator
from typing import List, Dict, Optional, Any
from datetime import datetime
import uuid


class RetrievedChunk(BaseModel):
    """
    Represents a chunk of content retrieved from the vector database.

    Attributes:
        id: Unique identifier for the chunk
        title: Title of the source document
        url: URL of the source
        content: The actual content text
        section: Section of the textbook where content appears
        tags: Associated tags for categorization
        score: Relevance score from the retrieval system
        similarity: Similarity score (cosine similarity or other metric)
    """
    id: str
    title: str
    url: str
    content: str
    section: str
    tags: List[str]
    score: float
    similarity: float

    class Config:
        # Allow extra fields during development, can be restricted later
        extra = "allow"


class ChatRequest(BaseModel):
    """
    Request model for the chat endpoint.

    Attributes:
        query: The user's query/question
        selected_text: Optional selected text to bias retrieval (for context awareness)
        session_id: Optional session identifier for conversation history (future use)
        top_k: Number of top results to retrieve (default: 3, range: 1-20)
        min_similarity: Minimum similarity threshold for retrieval (default: 0.4, range: 0.0-1.0)
        temperature: Temperature for response generation (default: 0.7, range: 0.0-2.0)
    """
    query: str = Field(..., min_length=1, max_length=2000, description="User's query (1-2000 characters)")
    selected_text: str = Field("", max_length=2000, description="Selected text for context biasing (max 2000 characters)")
    session_id: Optional[str] = Field(None, description="Session identifier for conversation history")
    top_k: int = Field(3, ge=1, le=20, description="Number of results to retrieve (1-20)")
    min_similarity: float = Field(0.4, ge=0.0, le=1.0, description="Minimum similarity threshold (0.0-1.0)")
    temperature: float = Field(0.7, ge=0.0, le=2.0, description="Generation temperature (0.0-2.0)")

    @validator('query')
    def validate_query(cls, v):
        """Validate query length and content."""
        if len(v.strip()) == 0:
            raise ValueError('Query cannot be empty or whitespace only')
        if len(v) > 2000:
            raise ValueError('Query must be at most 2000 characters')
        return v.strip()

    @validator('selected_text')
    def validate_selected_text(cls, v):
        """Validate selected text length."""
        if len(v) > 2000:
            raise ValueError('Selected text must be at most 2000 characters')
        return v

    @validator('top_k')
    def validate_top_k(cls, v):
        """Validate top_k range."""
        if v < 1 or v > 20:
            raise ValueError('top_k must be between 1 and 20')
        return v

    @validator('min_similarity')
    def validate_min_similarity(cls, v):
        """Validate min_similarity range."""
        if v < 0.0 or v > 1.0:
            raise ValueError('min_similarity must be between 0.0 and 1.0')
        return v

    @validator('temperature')
    def validate_temperature(cls, v):
        """Validate temperature range."""
        if v < 0.0 or v > 2.0:
            raise ValueError('temperature must be between 0.0 and 2.0')
        return v


class ChatResponse(BaseModel):
    """
    Response model for the chat endpoint.

    Attributes:
        response: The agent's response to the query
        sources: List of retrieved chunks used to generate the response
        session_id: Session identifier (for future multi-turn support)
        metadata: Additional metadata for UI enhancements and analytics
        retrieval_time: Time taken for the retrieval phase
        generation_time: Time taken for the generation phase
    """
    response: str
    sources: List[RetrievedChunk]
    session_id: str
    metadata: Dict[str, Any]
    retrieval_time: float
    generation_time: float


class HealthResponse(BaseModel):
    """
    Response model for the health check endpoint.

    Attributes:
        status: Overall health status (healthy, degraded, unhealthy)
        timestamp: ISO 8601 timestamp of the check
        services: Status of individual services (qdrant, agent, etc.)
    """
    status: str
    timestamp: str
    services: Dict[str, str]


class StatusResponse(BaseModel):
    """
    Response model for the detailed status endpoint.

    Attributes:
        status: Overall system status
        vector_db_status: Detailed status of the vector database
        agent_status: Detailed status of the agent system
        uptime: System uptime in seconds
    """
    status: str
    vector_db_status: Dict[str, Any]
    agent_status: Dict[str, Any]
    uptime: float


class ErrorMessage(BaseModel):
    """
    Standard error response format.

    Attributes:
        error: Error message
        error_type: Type of error (validation_error, retrieval_error, generation_error, etc.)
        details: Additional error details (optional)
    """
    error: str
    error_type: str
    details: Optional[Dict[str, Any]] = None




class ChatMessage(BaseModel):
    """
    Model for individual chat messages.

    Attributes:
        id: Unique message identifier
        message: Content of the message
        sender: Who sent the message ('user' or 'assistant')
        timestamp: When the message was sent
        sources: Retrieved sources associated with the message (for assistant responses)
    """
    id: str
    message: str
    sender: str
    timestamp: datetime
    sources: Optional[List[RetrievedChunk]] = None

    @validator('sender')
    def validate_sender(cls, v):
        """Validate sender is either 'user' or 'assistant'."""
        if v not in ['user', 'assistant']:
            raise ValueError("Sender must be either 'user' or 'assistant'")
        return v


class ConversationSession(BaseModel):
    """
    Model for conversation session management.

    Attributes:
        session_id: Unique session identifier
        created_at: When the session was created
        last_activity: When the session had last activity
        messages: List of messages in the session
        metadata: Additional session metadata
    """
    session_id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    created_at: datetime = Field(default_factory=datetime.utcnow)
    last_activity: datetime = Field(default_factory=datetime.utcnow)
    messages: List[ChatMessage] = Field(default_factory=list)
    metadata: Dict[str, Any] = Field(default_factory=dict)


class AgentState(BaseModel):
    """
    Model for tracking agent state during processing.

    Attributes:
        query: Original user query
        processed_query: Query after preprocessing (e.g., with selected text incorporated)
        retrieved_chunks: Chunks retrieved from the vector database
        context: Formatted context for the LLM
        response: Final response from the agent
        retrieval_time: Time taken for retrieval phase
        generation_time: Time taken for generation phase
    """
    query: str
    processed_query: str
    retrieved_chunks: List[RetrievedChunk]
    context: str
    response: str
    retrieval_time: float
    generation_time: float