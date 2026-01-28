# Quickstart Guide: Test Vector Retrieval Pipeline

## Overview
This guide will help you quickly set up and run tests for the vector retrieval pipeline in the Physical AI textbook RAG system.

## Prerequisites
- Python 3.11+
- uv package manager
- Access to Qdrant vector database with Physical AI textbook content
- API keys for Cohere and your LLM provider (OpenAI, DeepSeek, etc.)

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
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=physical_ai_textbook
COHERE_API_KEY=your_cohere_api_key
OPENAI_API_KEY=your_openai_api_key  # Or DEEPSEEK_API_KEY, etc.
MODEL_NAME=gpt-3.5-turbo  # Or your preferred model
EMBEDDING_MODEL=embed-multilingual-v3.0  # Or your preferred embedding model
```

### 4. Run the Backend Server
```bash
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

## Running Tests

### 1. Accuracy Tests
To run accuracy validation tests:
```bash
python -m pytest tests/test_retrieval_accuracy.py -v
```

### 2. Performance Tests
To run performance validation tests:
```bash
python -m pytest tests/test_retrieval_performance.py -v
```

### 3. Semantic Understanding Tests
To validate semantic matching capabilities:
```bash
python -m pytest tests/test_semantic_matching.py -v
```

### 4. End-to-End Test
To run a complete test suite:
```bash
python -m pytest tests/ -k "retrieval" -v
```

## Testing API Endpoints

### 1. Test Accuracy Endpoint
```bash
curl -X POST http://localhost:8000/test/accuracy \
  -H "Content-Type: application/json" \
  -d '{
    "test_set": "physics_fundamentals",
    "top_k": 5,
    "min_similarity": 0.4
  }'
```

### 2. Test Performance Endpoint
```bash
curl -X GET "http://localhost:8000/test/performance/load?concurrent_users=10&test_duration=60"
```

### 3. Health Check
```bash
curl http://localhost:8000/health
```

## Key Features

### 1. Precision/Recall Validation
- Measures precision@K and recall@K for retrieval accuracy
- Compares retrieved results against manually validated ground truth
- Provides detailed metrics per query and aggregate statistics

### 2. Performance Benchmarking
- Measures response times under various load conditions
- Validates system performance with concurrent users
- Tracks throughput and error rates

### 3. Semantic Understanding Validation
- Tests retrieval with semantic variations of queries
- Validates that system matches meaning, not just keywords
- Includes synonym and paraphrase validation

### 4. Confidence Score Validation
- Correlates confidence scores with actual relevance
- Validates that higher confidence scores indicate better results
- Provides Spearman correlation coefficients

## Configuration

### Test Parameters
You can customize test behavior with these parameters:
- `top_k`: Number of results to retrieve (default: 5, range: 1-20)
- `min_similarity`: Minimum similarity threshold (default: 0.4, range: 0.0-1.0)
- `concurrent_users`: Number of concurrent users in performance tests (default: 10)
- `test_duration`: Duration of performance tests in seconds (default: 60)

## Troubleshooting

### Common Issues

1. **Qdrant Connection Issues**
   - Verify QDRANT_URL and QDRANT_API_KEY are correct
   - Check that the collection exists and has content

2. **API Key Issues**
   - Ensure all required API keys are set in the .env file
   - Verify API key permissions and quotas

3. **Performance Test Failures**
   - Check system resources during load testing
   - Verify concurrent user limits in API providers

### Test Results Interpretation
- Precision@K: Proportion of retrieved results that are relevant
- Recall@K: Proportion of all relevant results that were retrieved
- MRR (Mean Reciprocal Rank): Quality measure considering rank of first relevant result
- Response times: Should be under 2 seconds for 95%+ of requests

## Next Steps

1. Customize test sets with domain-specific queries
2. Add more comprehensive semantic validation tests
3. Set up automated testing pipelines
4. Monitor performance metrics over time