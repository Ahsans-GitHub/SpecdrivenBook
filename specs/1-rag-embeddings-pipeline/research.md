# Research: RAG Embeddings Pipeline

## Decision: Python Project Structure
**Rationale**: Following the constraint to create a `RAG-backend/` folder with a single `main.py` file that initializes with `uv`. This approach keeps the implementation simple and focused while meeting the requirements.
**Alternatives considered**: Multi-file structure with separate modules for fetching, processing, embedding, and storage. However, the specification specifically requests a single main.py file.

## Decision: Content Fetching Approach
**Rationale**: Using requests library with BeautifulSoup for parsing HTML content from URLs. This is the most common and reliable approach for web scraping in Python.
**Alternatives considered**: Using Selenium for JavaScript-heavy sites, or scrapy for large-scale scraping. For this use case, requests + BeautifulSoup is sufficient and lighter weight.

## Decision: Text Chunking Strategy
**Rationale**: Using semantic chunking based on document structure (headings, paragraphs) rather than fixed-length token splitting. This preserves context and meaning for better embeddings.
**Alternatives considered**: Fixed-length character/word splitting, token-based splitting. Semantic chunking is better for RAG applications as it maintains coherent content sections.

## Decision: Embedding Model Selection
**Rationale**: Using Cohere's embed-english-v3.0 model as specified in the feature requirements. This model is optimized for English text and works well for educational content.
**Alternatives considered**: OpenAI embeddings, Sentence Transformers. Cohere was specifically requested and offers good performance for the free tier.

## Decision: Qdrant Vector Database Setup
**Rationale**: Using Qdrant Cloud Free Tier as specified in the constraints. Qdrant offers good performance for semantic search and has Python client libraries.
**Alternatives considered**: Pinecone, Weaviate, ChromaDB. Qdrant was specifically requested and fits the free-tier requirement.

## Decision: Error Handling Strategy
**Rationale**: Implement comprehensive error handling with retry mechanisms for API calls and graceful degradation when services are unavailable. This aligns with the Defensive Programming principle from the constitution.
**Alternatives considered**: Basic try-catch blocks. The approach includes detailed logging, specific exception handling, and retry logic with exponential backoff.

## Decision: Environment Configuration
**Rationale**: Using python-dotenv for managing API keys and configuration values. This follows security best practices by keeping sensitive information out of the codebase.
**Alternatives considered**: Hardcoded values (not secure), command-line arguments. Environment variables are the standard approach for configuration management.

## Decision: Rate Limit Handling
**Rationale**: Implementing rate limit handling with exponential backoff and retry mechanisms to work within free-tier constraints. This ensures the system continues to function even when hitting API limits.
**Alternatives considered**: Simple delays, no rate limit handling. The chosen approach is more robust and handles the free-tier constraints properly.