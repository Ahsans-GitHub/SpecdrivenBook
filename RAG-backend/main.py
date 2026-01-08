#!/usr/bin/env python3
"""
RAG Embeddings Pipeline

This script implements a pipeline to fetch content from URLs, generate embeddings using Cohere,
and store them in Qdrant vector database for RAG (Retrieval Augmented Generation) applications.
"""

import argparse
import asyncio
import sys
import time
from typing import List, Dict, Tuple, Optional
from urllib.parse import urlparse

import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models

from config import Config
from utils.urls import is_valid_url, validate_and_filter_urls
from utils.text_processing import clean_html_content, semantic_chunk_text, extract_title_from_html
from utils.errors import ContentFetchError
from utils.retry import retry_with_backoff
from utils.logging import rag_logger


class RAGEmbeddingPipeline:
    """Main class for the RAG Embeddings Pipeline."""

    def __init__(self):
        """Initialize the pipeline with configuration and clients."""
        # Validate configuration
        Config.validate_config()

        # Initialize Cohere client
        self.cohere_client = cohere.Client(Config.COHERE_API_KEY)

        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
            timeout=10
        )

        # Initialize other components
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        })

    def fetch_content_from_url(self, url: str) -> Tuple[str, str]:
        """
        Fetch content from a URL.

        Args:
            url: The URL to fetch content from

        Returns:
            Tuple of (content, title) where content is the HTML and title is the page title
        """
        try:
            # Increase timeout to accommodate Vercel's delayed loading for dynamic routes
            response = self.session.get(url, timeout=60)
            response.raise_for_status()

            # Extract title from the content
            title = extract_title_from_html(response.text)

            rag_logger.info(f"Successfully fetched content from {url}")
            return response.text, title

        except requests.exceptions.RequestException as e:
            error_msg = f"Failed to fetch content from {url}: {str(e)}"
            rag_logger.error(error_msg)
            raise ContentFetchError(error_msg) from e

    def parse_html_content(self, html_content: str) -> str:
        """
        Parse HTML content and extract text.

        Args:
            html_content: Raw HTML content

        Returns:
            str: Cleaned text content
        """
        try:
            text = clean_html_content(html_content)
            # Sanitize the extracted text to prevent potential security issues
            sanitized_text = sanitize_text(text)
            rag_logger.info(f"Parsed HTML content, got {len(sanitized_text)} characters of text")
            return sanitized_text
        except Exception as e:
            error_msg = f"Failed to parse HTML content: {str(e)}"
            rag_logger.error(error_msg)
            raise ValueError(error_msg) from e

    def extract_content_from_html(self, html_content: str, url: str, title: str = "") -> List[Dict]:
        """
        Extract and chunk content from HTML with metadata.

        Args:
            html_content: Raw HTML content
            url: Source URL
            title: Page title

        Returns:
            List of content chunks with metadata
        """
        try:
            # Sanitize the URL to prevent potential security issues
            sanitized_url = sanitize_input(url)

            # Parse and clean the content
            text_content = self.parse_html_content(html_content)

            # Extract title if not provided and sanitize it
            if not title:
                title = extract_title_from_html(html_content)
            sanitized_title = sanitize_input(title) if title else ""

            # Chunk the text semantically
            chunks = semantic_chunk_text(
                text_content,
                chunk_size=Config.CHUNK_SIZE,
                overlap=Config.OVERLAP_SIZE
            )

            # Create content chunks with metadata
            content_chunks = []
            for i, chunk in enumerate(chunks):
                # Sanitize the chunk text
                sanitized_chunk = sanitize_text(chunk)

                chunk_data = {
                    'id': f"{url_hash(sanitized_url)}_{i}",
                    'url': sanitized_url,
                    'text': sanitized_chunk,
                    'title': sanitized_title,
                    'section': extract_section_from_url(sanitized_url),  # Extract section from URL
                    'level': 'intermediate',  # Default level, could be extracted from content
                    'tags': extract_tags_from_url(sanitized_url),  # Extract tags from URL
                    'created_at': time.time()
                }
                content_chunks.append(chunk_data)

            rag_logger.info(f"Created {len(content_chunks)} content chunks from {sanitized_url}")
            return content_chunks

        except Exception as e:
            error_msg = f"Failed to extract content from {url}: {str(e)}"
            rag_logger.error(error_msg)
            raise ValueError(error_msg) from e

    @retry_with_backoff(
        max_retries=3,
        allowed_exceptions=(requests.exceptions.RequestException, ContentFetchError)
    )
    def fetch_and_process_single_url(self, url: str) -> List[Dict]:
        """
        Fetch and process a single URL.

        Args:
            url: The URL to process

        Returns:
            List of content chunks with metadata
        """
        try:
            # Fetch content from URL
            html_content, title = self.fetch_content_from_url(url)

            # Extract and chunk content
            content_chunks = self.extract_content_from_html(html_content, url, title)

            return content_chunks

        except Exception as e:
            error_msg = f"Error processing URL {url}: {str(e)}"
            rag_logger.error(error_msg)
            raise

    def process_multiple_urls(self, urls: List[str]) -> List[Dict]:
        """
        Process multiple URLs and return all content chunks.

        Args:
            urls: List of URLs to process

        Returns:
            List of all content chunks from all URLs
        """
        start_time = time.time()
        rag_logger.info(f"Starting to process {len(urls)} URLs")

        all_chunks = []

        for i, url in enumerate(urls, 1):
            try:
                rag_logger.info(f"Processing URL {i}/{len(urls)}: {url}")
                url_start_time = time.time()
                chunks = self.fetch_and_process_single_url(url)
                url_processing_time = time.time() - url_start_time
                all_chunks.extend(chunks)
                rag_logger.info(f"Successfully processed {url} in {url_processing_time:.2f}s, got {len(chunks)} chunks")
            except Exception as e:
                rag_logger.error(f"Failed to process {url}: {str(e)}")
                continue  # Continue with other URLs

        total_time = time.time() - start_time
        rag_logger.info(f"Completed processing {len(urls)} URLs, got {len(all_chunks)} total chunks in {total_time:.2f}s")
        return all_chunks

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere.

        Args:
            texts: List of text strings to generate embeddings for

        Returns:
            List of embedding vectors
        """
        try:
            rag_logger.info(f"Starting to generate embeddings for {len(texts)} text chunks")
            start_time = time.time()

            # Generate embeddings using Cohere
            response = self.cohere_client.embed(
                texts=texts,
                model=Config.EMBEDDING_MODEL,
                input_type="search_document"
            )

            embeddings = response.embeddings
            embedding_time = time.time() - start_time
            rag_logger.info(f"Generated embeddings for {len(texts)} text chunks in {embedding_time:.2f}s")
            return embeddings

        except Exception as e:
            error_msg = f"Failed to generate embeddings: {str(e)}"
            rag_logger.error(error_msg)
            raise

    @retry_with_backoff(
        max_retries=3,
        allowed_exceptions=(Exception,)
    )
    def generate_embeddings_with_retry(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings with retry logic for handling rate limits.

        Args:
            texts: List of text strings to generate embeddings for

        Returns:
            List of embedding vectors
        """
        return self.generate_embeddings(texts)

    def process_content_chunks_with_embeddings(self, content_chunks: List[Dict]) -> List[Dict]:
        """
        Process content chunks to add embeddings.

        Args:
            content_chunks: List of content chunks with metadata

        Returns:
            List of content chunks with embeddings added
        """
        if not content_chunks:
            return []

        # Extract texts for embedding generation
        texts = [chunk['text'] for chunk in content_chunks]

        # Generate embeddings
        embeddings = self.generate_embeddings_with_retry(texts)

        # Add embeddings to content chunks
        processed_chunks = []
        for chunk, embedding in zip(content_chunks, embeddings):
            chunk_with_embedding = chunk.copy()
            chunk_with_embedding['embedding'] = embedding
            chunk_with_embedding['processed_at'] = time.time()
            processed_chunks.append(chunk_with_embedding)

            # Log embedding result
            rag_logger.log_embedding_result(
                chunk_id=chunk['id'],
                success=True,
                vector_dimension=len(embedding) if embedding else 0
            )

        rag_logger.info(f"Added embeddings to {len(processed_chunks)} content chunks")
        return processed_chunks

    def setup_qdrant_collection(self):
        """Set up the Qdrant collection for storing embeddings."""
        try:
            # Check if collection already exists
            collections = self.qdrant_client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if Config.QDRANT_COLLECTION not in collection_names:
                # Create collection with appropriate vector size
                # We'll determine the vector size from a sample embedding
                sample_embedding = self.cohere_client.embed(
                    texts=["sample"],
                    model=Config.EMBEDDING_MODEL,
                    input_type="search_document"
                )
                vector_size = len(sample_embedding.embeddings[0]) if sample_embedding.embeddings else 1024

                self.qdrant_client.create_collection(
                    collection_name=Config.QDRANT_COLLECTION,
                    vectors_config=models.VectorParams(
                        size=vector_size,
                        distance=models.Distance.COSINE
                    )
                )
                rag_logger.info(f"Created Qdrant collection: {Config.QDRANT_COLLECTION}")
            else:
                rag_logger.info(f"Qdrant collection {Config.QDRANT_COLLECTION} already exists")

        except Exception as e:
            error_msg = f"Failed to set up Qdrant collection: {str(e)}"
            rag_logger.error(error_msg)
            raise

    def store_embeddings_in_qdrant(self, content_chunks: List[Dict]) -> bool:
        """
        Store embeddings with metadata in Qdrant vector database.

        Args:
            content_chunks: List of content chunks with embeddings and metadata

        Returns:
            bool: True if storage was successful, False otherwise
        """
        if not content_chunks:
            rag_logger.info("No content chunks to store in Qdrant")
            return True

        rag_logger.info(f"Starting to store {len(content_chunks)} embeddings in Qdrant collection: {Config.QDRANT_COLLECTION}")
        start_time = time.time()

        try:
            # Prepare points for Qdrant
            points = []
            for chunk in content_chunks:
                # Sanitize all metadata before storing
                sanitized_url = sanitize_input(chunk['url']) if chunk['url'] else ''
                sanitized_title = sanitize_input(chunk['title']) if chunk['title'] else ''
                sanitized_section = sanitize_input(chunk['section']) if chunk['section'] else ''
                sanitized_level = sanitize_input(chunk['level']) if chunk['level'] else ''

                # Sanitize tags
                sanitized_tags = []
                if chunk.get('tags'):
                    for tag in chunk['tags']:
                        sanitized_tag = sanitize_input(str(tag)) if tag else ''
                        if sanitized_tag:  # Only add non-empty tags
                            sanitized_tags.append(sanitized_tag)

                # Sanitize text content and limit size
                text_content = chunk['text'] if chunk['text'] else ''
                sanitized_text = sanitize_text(text_content)
                # Limit text size in payload to prevent storage issues
                limited_text = sanitized_text[:2000] if sanitized_text else ''

                # Create payload with sanitized metadata
                payload = {
                    'url': sanitized_url,
                    'title': sanitized_title,
                    'section': sanitized_section,
                    'level': sanitized_level,
                    'tags': sanitized_tags,
                    'text': limited_text,
                    'created_at': chunk.get('created_at', time.time()),
                    'processed_at': chunk.get('processed_at', time.time())
                }

                # Create a point for Qdrant
                point = models.PointStruct(
                    id=chunk['id'],
                    vector=chunk['embedding'],
                    payload=payload
                )
                points.append(point)

            # Upload points to Qdrant
            self.qdrant_client.upsert(
                collection_name=Config.QDRANT_COLLECTION,
                points=points
            )

            total_time = time.time() - start_time
            rag_logger.info(f"Successfully stored {len(points)} embeddings in Qdrant collection: {Config.QDRANT_COLLECTION} in {total_time:.2f}s")

            # Log storage results for each chunk
            for chunk in content_chunks:
                rag_logger.log_storage_result(
                    chunk_id=chunk['id'],
                    success=True,
                    collection=Config.QDRANT_COLLECTION
                )

            return True

        except Exception as e:
            total_time = time.time() - start_time
            error_msg = f"Failed to store embeddings in Qdrant after {total_time:.2f}s: {str(e)}"
            rag_logger.error(error_msg)
            # Log failure for each chunk
            for chunk in content_chunks:
                rag_logger.log_storage_result(
                    chunk_id=chunk['id'],
                    success=False,
                    collection=Config.QDRANT_COLLECTION
                )
            raise

    def search_similar_content(self, query: str, top_k: int = 5) -> List[Dict]:
        """
        Search for similar content in the vector database.

        Args:
            query: Query text to search for
            top_k: Number of results to return

        Returns:
            List of similar content chunks with metadata
        """
        try:
            # Sanitize the query to prevent potential security issues
            sanitized_query = sanitize_text(query) if query else ""

            # Validate top_k parameter
            if top_k <= 0:
                top_k = 5  # Default to 5 if invalid
            elif top_k > 100:  # Limit to prevent excessive resource usage
                top_k = 100

            # Generate embedding for the query
            query_embedding = self.cohere_client.embed(
                texts=[sanitized_query],
                model=Config.EMBEDDING_MODEL,
                input_type="search_query"
            ).embeddings[0]

            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=Config.QDRANT_COLLECTION,
                query_vector=query_embedding,
                limit=top_k
            )

            # Format results
            results = []
            for result in search_results:
                result_data = {
                    'id': result.id,
                    'score': result.score,
                    'payload': result.payload,
                    'text': result.payload.get('text', '')
                }
                results.append(result_data)

            rag_logger.info(f"Found {len(results)} similar content items for query")
            return results

        except Exception as e:
            error_msg = f"Failed to search for similar content: {str(e)}"
            rag_logger.error(error_msg)
            return []

    def run_full_pipeline(self, urls: List[str]) -> bool:
        """
        Run the complete pipeline: fetch, process, embed, and store.

        Args:
            urls: List of URLs to process

        Returns:
            bool: True if the pipeline completed successfully, False otherwise
        """
        try:
            # Validate URLs
            valid_urls = validate_and_filter_urls(urls)
            if not valid_urls:
                print("No valid URLs provided.")
                return False

            print(f"Processing {len(valid_urls)} URLs...")
            rag_logger.info(f"Starting full pipeline with {len(valid_urls)} URLs")

            # Step 1: Process URLs to get content chunks
            content_start_time = time.time()
            all_chunks = self.process_multiple_urls(valid_urls)
            content_processing_time = log_performance_metrics(content_start_time, "Content processing")

            if not all_chunks:
                print("No content chunks were created from the provided URLs.")
                return False

            print(f"Created {len(all_chunks)} content chunks in {content_processing_time:.2f} seconds")
            rag_logger.info(f"Content processing completed with {len(all_chunks)} chunks in {content_processing_time:.2f}s")

            # Step 2: Generate embeddings for all content chunks
            embedding_start_time = time.time()
            chunks_with_embeddings = self.process_content_chunks_with_embeddings(all_chunks)
            embedding_time = log_performance_metrics(embedding_start_time, "Embedding generation")

            print(f"Generated embeddings for {len(chunks_with_embeddings)} chunks in {embedding_time:.2f} seconds")
            rag_logger.info(f"Embedding generation completed for {len(chunks_with_embeddings)} chunks in {embedding_time:.2f}s")

            # Step 3: Set up Qdrant collection and store embeddings
            storage_start_time = time.time()
            self.setup_qdrant_collection()
            storage_success = self.store_embeddings_in_qdrant(chunks_with_embeddings)
            storage_time = log_performance_metrics(storage_start_time, "Storage operation")

            if storage_success:
                print(f"Stored embeddings in Qdrant in {storage_time:.2f} seconds")
                rag_logger.info(f"Storage completed successfully in {storage_time:.2f}s")
            else:
                print("Failed to store embeddings in Qdrant")
                return False

            total_time = content_processing_time + embedding_time + storage_time
            print(f"\nPipeline completed successfully!")
            print(f"Total content chunks processed: {len(chunks_with_embeddings)}")
            print(f"Total processing time: {total_time:.2f} seconds")
            print(f"Content processing: {content_processing_time:.2f}s")
            print(f"Embedding generation: {embedding_time:.2f}s")
            print(f"Storage: {storage_time:.2f}s")

            # Log summary metrics
            rag_logger.info(f"Pipeline summary: {len(chunks_with_embeddings)} chunks processed in {total_time:.2f}s")
            rag_logger.info(f"Performance breakdown - Content processing: {content_processing_time:.2f}s, "
                          f"Embedding: {embedding_time:.2f}s, Storage: {storage_time:.2f}s")

            return True

        except Exception as e:
            error_msg = f"Pipeline failed with error: {str(e)}"
            print(error_msg)
            rag_logger.error(error_msg)
            return False


def url_hash(url: str) -> str:
    """Generate a short hash for a URL."""
    import hashlib
    return hashlib.md5(url.encode()).hexdigest()[:12]


def extract_section_from_url(url: str) -> str:
    """Extract section information from URL."""
    try:
        parsed = urlparse(url)
        path_parts = parsed.path.strip('/').split('/')
        # Look for common section indicators in the URL path
        for part in path_parts:
            if part in ['chapter', 'module', 'lesson', 'part', 'section']:
                idx = path_parts.index(part)
                if idx + 1 < len(path_parts):
                    return f"{part}-{path_parts[idx + 1]}"
        # If no specific section found, return the first meaningful path part
        for part in path_parts:
            if part and part not in ['www', 'api', 'v1', 'v2']:
                return part
        return 'unknown'
    except:
        return 'unknown'


def extract_tags_from_url(url: str) -> List[str]:
    """Extract tags from URL."""
    try:
        parsed = urlparse(url)
        path_parts = parsed.path.strip('/').split('/')
        tags = []
        # Look for common tag indicators in the URL path
        for part in path_parts:
            if len(part) > 2 and part not in ['www', 'api', 'v1', 'v2', 'index', 'home', 'about']:
                tags.append(part)
        return tags
    except:
        return []


from utils.text_processing import sanitize_text


def sanitize_input(input_text: str) -> str:
    """
    Sanitize input text to prevent potential security issues.

    Args:
        input_text: The input text to sanitize

    Returns:
        str: Sanitized input text
    """
    if not input_text:
        return ""

    return sanitize_text(input_text)


def performance_log_wrapper(func):
    """
    Decorator to log performance metrics for functions.

    Args:
        func: The function to wrap

    Returns:
        The wrapped function with performance logging
    """
    def wrapper(*args, **kwargs):
        start_time = time.time()
        rag_logger.info(f"Starting execution of {func.__name__}")
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        rag_logger.info(f"{func.__name__} completed in {execution_time:.2f} seconds")
        return result
    return wrapper


def log_performance_metrics(start_time: float, operation_name: str):
    """
    Log performance metrics for a specific operation.

    Args:
        start_time: The start time of the operation (from time.time())
        operation_name: The name of the operation being measured
    """
    end_time = time.time()
    execution_time = end_time - start_time
    rag_logger.info(f"{operation_name} completed in {execution_time:.2f} seconds")
    return execution_time


def main():
    """Main function to run the RAG embeddings pipeline."""
    parser = argparse.ArgumentParser(description='RAG Embeddings Pipeline')
    parser.add_argument('--urls', nargs='+', help='List of URLs to process')
    parser.add_argument('--config', help='Path to config file (optional)')
    parser.add_argument('--test', action='store_true', help='Run a simple test of the pipeline')
    args = parser.parse_args()

    # Initialize the pipeline
    pipeline = RAGEmbeddingPipeline()

    if args.test:
        # Run a simple test
        print("Running pipeline test...")

        # Test search functionality if there's data in the collection
        try:
            results = pipeline.search_similar_content("test query")
            print(f"Test search returned {len(results)} results")
            if results:
                print("Sample result:", results[0]['payload']['title'] if results[0]['payload'].get('title') else 'No title')
        except Exception as e:
            print(f"Test search failed: {str(e)}")

        # Test with a simple sample
        sample_urls = [
            "https://httpbin.org/html",  # Simple test page
        ]
        success = pipeline.run_full_pipeline(sample_urls)
        if success:
            print("Pipeline test completed successfully!")
        else:
            print("Pipeline test failed.")
        return

    # Define default URLs if none provided
    if not args.urls:
        print("No URLs provided, using sample URLs...")
        # In a real implementation, these would be actual URLs from the textbook
        sample_urls = [
            "https://example.com/chapter1",
            "https://example.com/chapter2",
            "https://example.com/module1",
            "https://example.com/lesson1",
            "https://example.com/part1"
        ]
        urls = sample_urls
    else:
        # Sanitize the input URLs
        urls = [sanitize_input(url) for url in args.urls if url]

    # Run the full pipeline
    success = pipeline.run_full_pipeline(urls)

    if success:
        print("\nPipeline completed successfully!")
    else:
        print("\nPipeline failed.")
        sys.exit(1)


if __name__ == "__main__":
    main()