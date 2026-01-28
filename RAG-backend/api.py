"""
FastAPI Backend for RAG Agent
This module implements the FastAPI backend that connects the frontend to the RAG agent,
handling chat queries, health checks, and status information.
"""

import asyncio
import time
import logging
import os
from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

# Import the RAG agent
from agent import RAGAgent
from models import RetrievedChunk
# Import Neon database integration
from neon_db import neon_manager


# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Configure logging
import os
# Create logs directory if it doesn't exist
os.makedirs("logs", exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler("logs/api.log")
    ]
)
logger = logging.getLogger(__name__)


# Pydantic models for request/response
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
    sources: List[Dict[str, Any]]
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


# Initialize FastAPI app
app = FastAPI(
    title="Physical AI RAG Agent API",
    description="REST API for the Physical AI & Humanoid Robotics textbook RAG agent",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# Add rate limiting exception handler
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Add CORS middleware to allow frontend connections
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, restrict this to your frontend domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize the RAG agent globally
rag_agent = RAGAgent()


@app.post("/chat",
          response_model=ChatResponse,
          summary="Process a chat query through the RAG agent",
          description="Main endpoint for the RAG chatbot. Accepts user queries, retrieves relevant content from the vector database, processes it through the agent, and returns a contextual response with sources.",
          )
@limiter.limit("10/minute")
async def chat_endpoint(request: Request, chat_request: ChatRequest) -> ChatResponse:
    """
    Process a chat query through the RAG agent.

    Args:
        request: Request object for rate limiting
        chat_request: ChatRequest containing the query and parameters

    Returns:
        ChatResponse with the agent's response, sources, and metadata
    """
    start_time = time.time()
    logger.info(f"Received chat request: query_length={len(chat_request.query)}, session_id={chat_request.session_id}")

    try:
        # Process the query through the RAG agent
        result = await rag_agent.process_query(
            query=chat_request.query,
            session_id=chat_request.session_id,
            selected_text=chat_request.selected_text,
            top_k=chat_request.top_k,
            min_similarity=chat_request.min_similarity,
            temperature=chat_request.temperature
        )

        processing_time = time.time() - start_time
        logger.info(f"Chat request completed successfully in {processing_time:.2f}s, session_id={result['session_id']}")

        # Convert sources to the expected format
        converted_sources = []
        for source in result.get('sources', []):
            converted_sources.append({
                "id": source.get('id', ''),
                "title": source.get('title', ''),
                "url": source.get('url', ''),
                "content": source.get('content', ''),
                "section": source.get('section', ''),
                "tags": source.get('tags', []),
                "score": source.get('score', 0.0),
                "similarity": source.get('similarity', 0.0)
            })

        # Create the response object
        response = ChatResponse(
            response=result['response'],
            sources=converted_sources,
            session_id=result['session_id'],
            metadata=result['metadata'],
            retrieval_time=result.get('retrieval_time', 0.0),
            generation_time=result.get('generation_time', 0.0)
        )

        return response

    except ValueError as ve:
        # Handle validation errors
        logger.error(f"Validation error in chat endpoint: {str(ve)}")
        raise HTTPException(
            status_code=400,
            detail={
                "error": str(ve),
                "error_type": "validation_error",
                "details": {"message": str(ve)}
            }
        )
    except Exception as e:
        # Handle other errors
        logger.error(f"Error in chat endpoint: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail={
                "error": "An unexpected error occurred while processing your request",
                "error_type": "internal_error",
                "details": {"message": str(e)}
            }
        )


@app.get("/health",
         response_model=HealthResponse,
         summary="Health check endpoint",
         description="Verify the service is running and all dependencies are accessible")
async def health_check() -> HealthResponse:
    """
    Health check endpoint to verify the service is running and all dependencies are accessible.

    Returns:
        Health status information
    """
    try:
        # Check if the retriever is working
        retriever_status = rag_agent.retriever.check_collection_status()
        retriever_ok = retriever_status.get('has_content', False) or retriever_status.get('total_documents', 0) >= 0

        # Check if the agent is initialized
        agent_ok = rag_agent.client is not None

        # Determine overall status
        overall_status = "healthy"
        if not retriever_ok or not agent_ok:
            overall_status = "degraded"

        return HealthResponse(
            status=overall_status,
            timestamp=time.strftime('%Y-%m-%dT%H:%M:%SZ'),
            services={
                "qdrant": "available" if retriever_ok else "unavailable",
                "agent": "available" if agent_ok else "unavailable"
            }
        )
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        return HealthResponse(
            status="unhealthy",
            timestamp=time.strftime('%Y-%m-%dT%H:%M:%SZ'),
            services={
                "qdrant": "unavailable",
                "agent": "unavailable"
            }
        )


@app.get("/status",
         response_model=StatusResponse,
         summary="Detailed status information",
         description="Detailed status information about the RAG system, including vector database statistics and agent readiness")
async def status_endpoint() -> StatusResponse:
    """
    Detailed status information about the RAG system, including vector database statistics and agent readiness.

    Returns:
        Status information
    """
    start_time = time.time()

    try:
        # Get vector database status
        vector_db_status = rag_agent.retriever.check_collection_status()

        # Get agent status
        agent_status = {
            "initialized": rag_agent.client is not None,
            "model": getattr(rag_agent, 'model_name', 'unknown')
        }

        # Calculate uptime (time since app started)
        uptime = time.time() - start_time  # This would be actual uptime in a real implementation

        return StatusResponse(
            status="ready",
            vector_db_status=vector_db_status,
            agent_status=agent_status,
            uptime=uptime
        )
    except Exception as e:
        logger.error(f"Status check failed: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail={
                "error": "Could not retrieve status information",
                "error_type": "internal_error",
                "details": {"message": str(e)}
            }
        )


@app.get("/",
         summary="Root endpoint",
         description="Root endpoint that provides basic information about the API")
async def root():
    """
    Root endpoint that provides basic information about the API.

    Returns:
        Dictionary with API information
    """
    return {
        "message": "Physical AI RAG Agent API",
        "version": "1.0.0",
        "description": "REST API for the Physical AI & Humanoid Robotics textbook RAG agent",
        "endpoints": {
            "POST /chat": "Main chat endpoint",
            "GET /health": "Health check",
            "GET /status": "Detailed status information",
            "GET /docs": "API documentation (Swagger UI)",
            "GET /redoc": "API documentation (Redoc)",
            "POST /history/save": "Save chat history to Neon database",
            "GET /history": "Retrieve chat history from Neon database"
        }
    }


# History endpoints
@app.post("/history/save",
          summary="Save chat interaction to Neon database",
          description="Saves a chat interaction to the Neon database for history tracking")
async def save_history_endpoint(request_data: dict):
    """
    Save a chat interaction to the Neon database.

    Args:
        request_data: Dictionary containing query, response, sources, and userId

    Returns:
        Success status
    """
    try:
        query = request_data.get('query', '')
        response = request_data.get('response', '')
        sources = request_data.get('sources', [])
        user_id = request_data.get('userId', 'anonymous')
        session_id = request_data.get('sessionId')

        if not query or not response:
            raise HTTPException(status_code=400, detail="Query and response are required")

        # Save to Neon database
        await neon_manager.save_chat_interaction(query, response, user_id, sources, session_id)

        return {"success": True, "message": "Chat history saved successfully"}
    except Exception as e:
        logger.error(f"Error saving history: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/history",
         summary="Retrieve chat history from Neon database",
         description="Retrieves chat history for a specific user from the Neon database")
async def get_history_endpoint(user_id: str = "anonymous", limit: int = 50):
    """
    Retrieve chat history for a specific user from the Neon database.

    Args:
        user_id: ID of the user to retrieve history for
        limit: Maximum number of records to return

    Returns:
        List of chat history records
    """
    try:
        history = await neon_manager.get_user_history(user_id, limit)

        return {
            "success": True,
            "history": history
        }
    except Exception as e:
        logger.error(f"Error retrieving history: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


# Event handlers
@app.on_event("startup")
async def startup_event():
    """Handle startup events."""
    logger.info("Starting up RAG Agent API...")
    # Initialize the Neon database connection
    await neon_manager.initialize()


@app.on_event("shutdown")
async def shutdown_event():
    """Handle shutdown events."""
    logger.info("Shutting down RAG Agent API...")


# Error handlers
@app.exception_handler(404)
async def not_found_handler(request, exc):
    """Handle 404 errors."""
    return JSONResponse(
        status_code=404,
        content={
            "error": "Endpoint not found",
            "error_type": "not_found",
            "details": {"path": str(request.url)}
        }
    )


@app.exception_handler(500)
async def internal_error_handler(request, exc):
    """Handle 500 errors."""
    logger.error(f"Internal server error: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": "An internal server error occurred",
            "error_type": "internal_error",
            "details": {"message": "Please contact the administrator"}
        }
    )


if __name__ == "__main__":
    import uvicorn
    # For development/testing purposes
    uvicorn.run(app, host="0.0.0.0", port=8000)