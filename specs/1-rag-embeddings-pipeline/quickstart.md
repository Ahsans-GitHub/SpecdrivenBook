# Quickstart: RAG Embeddings Pipeline

## Prerequisites

- Python 3.11 or higher
- `uv` package manager installed
- Cohere API key (free tier)
- Qdrant Cloud account and API key (free tier)

## Setup

1. **Clone the repository and navigate to the RAG-backend directory:**
   ```bash
   cd RAG-backend
   ```

2. **Create a virtual environment using uv:**
   ```bash
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

3. **Install dependencies:**
   ```bash
   uv pip install requests cohere qdrant-client python-dotenv beautifulsoup4 lxml
   ```

4. **Create your environment file:**
   ```bash
   cp .env.example .env
   ```

5. **Add your API keys to the `.env` file:**
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_URL=your_qdrant_cluster_url
   ```

## Usage

### Run the full pipeline

```bash
python main.py
```

This will:
1. Fetch content from predefined URLs
2. Process and chunk the content
3. Generate embeddings using Cohere
4. Store embeddings in Qdrant vector database

### Run with custom URLs

```bash
python main.py --urls "https://example.com/page1" "https://example.com/page2"
```

## Configuration

The main.py file includes the following configurable parameters:

- `CHUNK_SIZE`: Maximum characters per content chunk (default: 1000)
- `OVERLAP_SIZE`: Overlap between chunks to maintain context (default: 200)
- `EMBEDDING_MODEL`: Cohere model to use (default: "embed-english-v3.0")
- `VECTOR_DIMENSION`: Dimension of the embedding vectors (depends on model)
- `QDRANT_COLLECTION`: Name of the Qdrant collection (default: "physical_ai_content")

## Example Output

When running successfully, you should see output similar to:

```
Starting RAG Embeddings Pipeline...
Processing URL: https://example.com/chapter1
Fetched content successfully
Created 5 content chunks
Generated embeddings for all chunks
Stored embeddings in Qdrant collection: physical_ai_content
Pipeline completed successfully
```

## Troubleshooting

**API Rate Limits**: If you encounter rate limit errors, the system will automatically retry with exponential backoff.

**Invalid URLs**: The system will skip invalid URLs and continue with valid ones, logging the errors.

**Qdrant Connection**: Ensure your Qdrant URL and API key are correct in the .env file.

## Next Steps

After successful setup:
1. Verify embeddings are stored in your Qdrant collection
2. Test similarity searches using the Qdrant dashboard
3. Customize the content URLs to match your specific content sources
4. Adjust chunk size based on your content and embedding requirements