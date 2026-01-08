# RAG Embeddings Pipeline

This project implements a pipeline to fetch content from URLs, generate embeddings using Cohere, and store them in Qdrant vector database for RAG (Retrieval Augmented Generation) applications.

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
   DEPLOY_VERCEL_URL=https://your-vercel-app-url.vercel.app
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

### Run a test of the pipeline

```bash
python main.py --test
```

## Configuration

The main.py file includes the following configurable parameters:

- `CHUNK_SIZE`: Maximum characters per content chunk (default: 1000)
- `OVERLAP_SIZE`: Overlap between chunks to maintain context (default: 200)
- `EMBEDDING_MODEL`: Cohere model to use (default: "embed-english-v3.0")
- `VECTOR_DIMENSION`: Dimension of the embedding vectors (depends on model)
- `QDRANT_COLLECTION`: Name of the Qdrant collection (default: "physical_ai_content")

## Example Usage

Here are some common usage examples:

### Process a single URL
```bash
python main.py --urls "https://example.com/content"
```

### Process multiple URLs
```bash
python main.py --urls "https://example.com/page1" "https://example.com/page2" "https://example.com/page3"
```

### Process textbook content
```bash
python main.py --urls "https://physicalaiandhumanoidrobotics.vercel.app/module1" "https://physicalaiandhumanoidrobotics.vercel.app/module2"
```

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

Pipeline completed successfully!
Total content chunks processed: 5
Total processing time: 15.23 seconds
Content processing: 8.45s
Embedding generation: 5.23s
Storage: 1.55s
```

## Troubleshooting

### Common Issues and Solutions

**API Rate Limits**: If you encounter rate limit errors, the system will automatically retry with exponential backoff.
- Solution: If issues persist, consider upgrading your Cohere API tier or reducing the number of concurrent requests.

**Invalid URLs**: The system will skip invalid URLs and continue with valid ones, logging the errors.
- Solution: Verify that all URLs are properly formatted and accessible in a browser.

**Qdrant Connection**: Ensure your Qdrant URL and API key are correct in the .env file.
- Solution: Check your Qdrant Cloud dashboard for the correct URL and API key format.

**Memory Issues**: For large documents, consider reducing the chunk size to prevent memory issues.
- Solution: Set CHUNK_SIZE to a smaller value (e.g., 500 instead of 1000) in your environment variables.

**Slow Processing**: If processing is slow, check your internet connection and API key rate limits.
- Solution: Verify your internet speed and check if you're hitting API rate limits.

### Authentication Errors

**Cohere API Error**: If you see "Invalid API key" errors:
- Verify that your COHERE_API_KEY is correct and not expired
- Check that there are no extra spaces or characters in your .env file

**Qdrant Authentication Error**: If you see "Unauthorized" or "Forbidden" errors:
- Verify that your QDRANT_API_KEY is correct
- Ensure your QDRANT_URL is properly formatted (should include protocol and port if needed)

### Network and Connectivity Issues

**Timeout Errors**: If you see timeout errors when fetching content:
- Check your internet connection
- Try increasing the timeout values in the configuration
- Some websites may block automated requests; consider adding appropriate headers

**SSL Certificate Issues**: If you encounter SSL errors:
- Ensure your Python environment has updated certificates
- In some cases, you may need to configure SSL settings in your environment

### Configuration Issues

**Missing Environment Variables**: If the application fails to start:
- Verify all required environment variables are set in your .env file
- Check that your .env file is in the correct location
- Ensure there are no typos in variable names

**Chunk Size Problems**: If you encounter errors processing large documents:
- Adjust CHUNK_SIZE to be appropriate for your content
- Ensure OVERLAP_SIZE is less than CHUNK_SIZE

### Performance Optimization

**Slow Embedding Generation**: If embedding generation is slow:
- Consider using a more powerful Cohere model if available in your tier
- Check if your network connection is stable
- Monitor your API usage to ensure you're not hitting rate limits

**Large Memory Usage**: If the process uses too much memory:
- Reduce CHUNK_SIZE to process smaller text chunks
- Process fewer URLs simultaneously
- Monitor your system's memory usage during execution

### Debugging Tips

**Enable Verbose Logging**: To get more detailed information about what's happening:
- Add logging configuration to see detailed process information
- Check the logs directory for detailed execution information

**Test Individual Components**: To isolate issues:
- Test URL fetching independently using simple requests
- Verify your Cohere and Qdrant connections separately
- Test with a small number of URLs first before processing large batches

## Performance Metrics

The pipeline logs performance metrics for each operation:
- Content processing time
- Embedding generation time
- Storage time
- Total pipeline execution time

## Next Steps

After successful setup:
1. Verify embeddings are stored in your Qdrant collection
2. Test similarity searches using the Qdrant dashboard
3. Customize the content URLs to match your specific content sources
4. Adjust chunk size based on your content and embedding requirements