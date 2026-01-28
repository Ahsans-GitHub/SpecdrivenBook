# Quickstart Guide: RAG Agent Backend Implementation

## Overview
This guide will help you quickly set up and run the RAG Agent Backend for the Physical AI textbook. The system integrates OpenAI Agents SDK (or custom implementation), Cohere embeddings, Qdrant vector database, and Neon Postgres for conversation history.

## Prerequisites
- Python 3.11+
- uv package manager
- Access to Qdrant vector database with Physical AI textbook content
- API key for OpenAI (or alternative LLM provider like DeepSeek via OpenRouter)
- API key for Cohere for embeddings

## Setup

### 1. Navigate to the RAG-backend Directory
```bash
cd RAG-backend
```

### 2. Install Dependencies with uv
```bash
uv pip install -e .
```

### 3. Set Up Environment Variables
Create a `.env` file in the RAG-backend directory with your API keys:
```env
# Qdrant Configuration
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=physical_ai_textbook

# API Keys
COHERE_API_KEY=your_cohere_api_key
OPENAI_API_KEY=your_openai_api_key  # Or DEEPSEEK_API_KEY, OPENROUTER_API_KEY

# Model Configuration (optional)
MODEL_NAME=gpt-3.5-turbo  # Or deepseek/deepseek-r1-distill-llama-70b via OpenRouter
EMBEDDING_MODEL=embed-multilingual-v3.0

# Neon Postgres Configuration (optional, for conversation history)
NEON_POSTGRES_URL=your_neon_postgres_connection_string

# Application Configuration
CHUNK_SIZE=1000
OVERLAP_SIZE=200
DEBUG=true
```

### 4. Run the Backend Server
```bash
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

## API Usage

### 1. Test the Health Endpoint
```bash
curl http://localhost:8000/health
```

### 2. Send a Chat Query
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is embodied AI in robotics?",
    "top_k": 3,
    "min_similarity": 0.4,
    "temperature": 0.7
  }'
```

### 3. Send a Query with Selected Text Context
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept further",
    "selected_text": "Embodied AI refers to artificial intelligence systems that interact with the physical world through sensors and actuators",
    "top_k": 3,
    "min_similarity": 0.4,
    "temperature": 0.7
  }'
```

## Key Features

### 1. RAG Agent Capabilities
- Combines retrieval with intelligent generation
- Uses OpenAI Agents SDK if available, falls back to custom implementation
- Integrates with Qdrant vector database for content retrieval

### 2. Context-Aware Queries
- Support for optional selected text to bias retrieval
- Configurable number of results (top_k)
- Adjustable similarity thresholds (min_similarity)

### 3. Rich Responses
- Source citations for all information
- Metadata for enhanced UI features
- Performance timing information
- Confidence scores and adaptive prompts

### 4. Multi-turn Conversations
- Session management with UUID-based tracking
- Conversation history maintenance
- Context preservation across exchanges

## Testing

### 1. Test Basic Functionality
```bash
python -m pytest tests/test_basic_rag.py -v
```

### 2. Test Selected Text Context
```bash
python -m pytest tests/test_selected_text_biasing.py -v
```

### 3. Test Multi-turn Conversations
```bash
python -m pytest tests/test_multi_turn_conversations.py -v
```

### 4. Run All Tests
```bash
python -m pytest tests/ -v
```

## API Endpoints

- `POST /chat` - Main query endpoint with optional selected text context
- `GET /health` - Service health check
- `GET /status` - Detailed system status
- `GET /docs` - Interactive API documentation (Swagger UI)

## Configuration Options

### Query Parameters
- `query`: The user's query (required, max 2000 chars)
- `selected_text`: Optional selected text for context biasing (max 2000 chars)
- `session_id`: Optional session identifier for conversation history
- `top_k`: Number of results to retrieve (1-20, default: 3)
- `min_similarity`: Minimum similarity threshold (0.0-1.0, default: 0.4)
- `temperature`: Generation temperature (0.0-2.0, default: 0.7)

### Response Format
The API returns responses with:
- `response`: The agent's response to the query
- `sources`: List of retrieved chunks with citations
- `session_id`: Session identifier for conversation continuity
- `metadata`: Additional information for UI enhancements
- `retrieval_time`: Time taken for the retrieval phase
- `generation_time`: Time taken for the generation phase

## Troubleshooting

### Common Issues

1. **API Key Issues**
   - Verify all required API keys are set in the .env file
   - Check that API key permissions are correct

2. **Qdrant Connection Issues**
   - Verify QDRANT_URL and QDRANT_API_KEY are correct
   - Check that the collection exists and has content

3. **Agent SDK Availability**
   - If OpenAI Agents SDK is not available, the system uses a custom implementation
   - Check logs for any SDK availability warnings

4. **Memory Issues**
   - Large selected text contexts may cause memory issues
   - Limit selected text to 2000 characters maximum

### Performance Considerations
- Response times typically under 10 seconds
- Retrieval performance depends on vector database speed
- LLM provider response times may vary based on load

## Next Steps

1. Populate your Qdrant database with Physical AI textbook content
2. Test queries through the API
3. Integrate with your frontend application
4. Monitor performance and adjust parameters as needed