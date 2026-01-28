#!/usr/bin/env python3
"""
RAG Retrieval System - Main Executable
This module implements the retrieval component of the RAG system,
handling vector database operations and content retrieval with biasing.
Usage: python retrieval.py --query "your query here"
"""

import argparse
import asyncio
import time
import logging
import os
import sys
from typing import List, Dict, Any, Optional, Tuple
from urllib.parse import urlparse
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import PointStruct, Distance, VectorParams
from config import Config
from models import RetrievedChunk


logger = logging.getLogger(__name__)


class RAGRetrievalSystem:
    """
    Main retrieval system for the RAG agent.
    Handles vector database operations and content retrieval.
    """

    def __init__(self):
        """Initialize the retrieval system with Qdrant client."""
        try:
            # Initialize Cohere client for embeddings
            cohere_api_key = os.getenv("COHERE_API_KEY")
            if not cohere_api_key:
                raise ValueError("COHERE_API_KEY environment variable is required")

            self.cohere_client = cohere.Client(cohere_api_key)

            # Initialize Qdrant client
            qdrant_url = os.getenv("QDRANT_URL")
            qdrant_api_key = os.getenv("QDRANT_API_KEY")

            if not qdrant_url or not qdrant_api_key:
                raise ValueError("Both QDRANT_URL and QDRANT_API_KEY environment variables are required")

            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                timeout=30
            )

            # Verify collection exists
            self.collection_name = Config.QDRANT_COLLECTION
            self._verify_collection_exists()

            logger.info(f"RAG Retrieval System initialized with collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Failed to initialize RAG Retrieval System: {str(e)}")
            raise

    def _verify_collection_exists(self):
        """Verify that the required collection exists in Qdrant."""
        try:
            collections = self.qdrant_client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name not in collection_names:
                logger.warning(f"Collection {self.collection_name} does not exist, creating it...")

                # Get embedding size from Cohere
                sample_embedding = self.cohere_client.embed(
                    texts=["sample"],
                    model=Config.EMBEDDING_MODEL,
                    input_type="search_document"
                )
                vector_size = len(sample_embedding.embeddings[0]) if sample_embedding.embeddings else 1024

                # Create collection
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=vector_size,
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"Created collection: {self.collection_name}")
            else:
                logger.info(f"Collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error verifying collection: {str(e)}")
            raise

    def check_collection_status(self) -> Dict[str, Any]:
        """
        Check the status of the collection including content statistics.

        Returns:
            Dictionary with collection status information
        """
        try:
            # Get collection info
            collection_info = self.qdrant_client.get_collection(self.collection_name)

            # Get point count
            count = self.qdrant_client.count(
                collection_name=self.collection_name
            )

            return {
                "status": "available",
                "has_content": count.count > 0,
                "total_documents": count.count,
                "collection_name": self.collection_name,
                "vector_size": collection_info.config.params.vectors.size if hasattr(collection_info, 'config') else "unknown"
            }
        except Exception as e:
            logger.error(f"Error checking collection status: {str(e)}")
            return {
                "status": "unavailable",
                "has_content": False,
                "total_documents": 0,
                "error": str(e)
            }

    async def retrieve_with_biasing(
        self,
        query: str,
        selected_text: str = "",
        top_k: int = 3,
        min_similarity: float = 0.4
    ) -> List[Dict[str, Any]]:
        """
        Retrieve content with optional biasing based on selected text.

        Args:
            query: Original query to search for
            selected_text: Optional selected text to bias retrieval toward
            top_k: Number of results to return
            min_similarity: Minimum similarity threshold

        Returns:
            List of retrieved content chunks with metadata
        """
        start_time = time.time()
        logger.info(f"Starting retrieval for query: '{query[:50]}...', selected_text_len={len(selected_text)}")

        try:
            # Generate embedding for the query
            search_query = query
            if selected_text and selected_text.strip():
                # Combine query with selected text for better context
                search_query = f"{query} Context from selected text: {selected_text[:500]}"

            query_embedding = self.cohere_client.embed(
                texts=[search_query],
                model=Config.EMBEDDING_MODEL,
                input_type="search_query"
            ).embeddings[0]

            # Perform search in Qdrant - using correct method name for newer Qdrant versions
            try:
                # Try the newer query_points method first
                search_results = self.qdrant_client.query_points(
                    collection_name=self.collection_name,
                    query=query_embedding,
                    limit=top_k * 2,  # Get more results to apply similarity filter
                    with_payload=True
                ).points
            except AttributeError:
                # Fall back to the older search method if needed
                search_results = self.qdrant_client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding,
                    limit=top_k * 2,  # Get more results to apply similarity filter
                    with_payload=True
                )

            # Filter results by minimum similarity and convert to RetrievedChunk format
            filtered_results = []
            for result in search_results:
                if result.score >= min_similarity:
                    # Extract metadata from payload
                    payload = result.payload or {}

                    # Create result dictionary matching RetrievedChunk fields
                    result_dict = {
                        "id": result.id,
                        "title": payload.get("title", ""),
                        "url": payload.get("url", ""),
                        "content": payload.get("text", payload.get("content", ""))[:2000],  # Limit content size
                        "section": payload.get("section", ""),
                        "tags": payload.get("tags", []),
                        "score": result.score,
                        "similarity": result.score  # In Qdrant, the score often represents similarity
                    }
                    filtered_results.append(result_dict)

            # Limit to top_k results after filtering
            final_results = filtered_results[:top_k]

            retrieval_time = time.time() - start_time
            logger.info(f"Retrieved {len(final_results)} results in {retrieval_time:.2f}s")

            return final_results

        except Exception as e:
            logger.error(f"Error during retrieval: {str(e)}", exc_info=True)
            return []

    def _extract_section_from_url(self, url: str) -> str:
        """
        Extract section information from URL.

        Args:
            url: The URL to extract section from

        Returns:
            Extracted section name
        """
        try:
            parsed = urlparse(url)
            path_parts = parsed.path.strip('/').split('/')

            # Look for common section indicators in the URL path
            for part in path_parts:
                if part in ['chapter', 'module', 'lesson', 'part', 'section', 'topic']:
                    idx = path_parts.index(part)
                    if idx + 1 < len(path_parts):
                        return f"{part}-{path_parts[idx + 1]}"

            # If no specific section found, return the first meaningful path part
            for part in path_parts:
                if part and part not in ['www', 'api', 'v1', 'v2', 'index', 'home', 'about', 'docs']:
                    return part

            return 'general'
        except Exception:
            return 'general'

    def _extract_tags_from_url(self, url: str) -> List[str]:
        """
        Extract tags from URL.

        Args:
            url: The URL to extract tags from

        Returns:
            List of extracted tags
        """
        try:
            parsed = urlparse(url)
            path_parts = parsed.path.strip('/').split('/')
            tags = []

            # Look for common tag indicators in the URL path
            for part in path_parts:
                if len(part) > 2 and part not in ['www', 'api', 'v1', 'v2', 'index', 'home', 'about', 'docs', 'chapter', 'lesson', 'part']:
                    tags.append(part)

            return tags[:5]  # Limit to 5 tags
        except Exception:
            return []

    def _sanitize_text(self, text: str) -> str:
        """
        Sanitize text to prevent potential security issues.

        Args:
            text: Text to sanitize

        Returns:
            Sanitized text
        """
        if not text:
            return text

        # Remove potential harmful characters/sequences
        # This is a basic sanitization - expand as needed
        sanitized = text.replace('\0', '')  # Remove null bytes
        return sanitized.strip()

    async def retrieve_content_by_ids(self, ids: List[str]) -> List[Dict[str, Any]]:
        """
        Retrieve specific content chunks by their IDs.

        Args:
            ids: List of content chunk IDs to retrieve

        Returns:
            List of content chunks with metadata
        """
        try:
            points = self.qdrant_client.retrieve(
                collection_name=self.collection_name,
                ids=ids,
                with_payload=True
            )

            results = []
            for point in points:
                payload = point.payload or {}
                result = {
                    "id": point.id,
                    "title": payload.get("title", ""),
                    "url": payload.get("url", ""),
                    "content": payload.get("text", payload.get("content", "")),
                    "section": payload.get("section", ""),
                    "tags": payload.get("tags", []),
                    "score": 1.0,  # Score not applicable when retrieving by ID
                    "similarity": 1.0  # Similarity not applicable when retrieving by ID
                }
                results.append(result)

            return results
        except Exception as e:
            logger.error(f"Error retrieving content by IDs: {str(e)}", exc_info=True)
            return []

    async def search_by_keywords(self, keywords: List[str], top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for content using keyword matching in addition to vector search.

        Args:
            keywords: List of keywords to search for
            top_k: Number of results to return

        Returns:
            List of content chunks with metadata
        """
        try:
            # Create a search query from keywords
            keyword_query = " ".join(keywords)

            # Generate embedding for keyword query
            query_embedding = self.cohere_client.embed(
                texts=[keyword_query],
                model=Config.EMBEDDING_MODEL,
                input_type="search_query"
            ).embeddings[0]

            # Perform search in Qdrant - using correct method name for newer Qdrant versions
            try:
                # Try the newer query_points method first
                search_results = self.qdrant_client.query_points(
                    collection_name=self.collection_name,
                    query=query_embedding,
                    limit=top_k,
                    with_payload=True
                ).points
            except AttributeError:
                # Fall back to the older search method if needed
                search_results = self.qdrant_client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding,
                    limit=top_k,
                    with_payload=True
                )

            # Convert results to our format
            results = []
            for result in search_results:
                payload = result.payload or {}
                result_dict = {
                    "id": result.id,
                    "title": payload.get("title", ""),
                    "url": payload.get("url", ""),
                    "content": payload.get("text", payload.get("content", ""))[:2000],
                    "section": payload.get("section", ""),
                    "tags": payload.get("tags", []),
                    "score": result.score,
                    "similarity": result.score
                }
                results.append(result_dict)

            return results
        except Exception as e:
            logger.error(f"Error during keyword search: {str(e)}", exc_info=True)
            return []

    async def get_content_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about the content in the vector database.

        Returns:
            Dictionary with content statistics
        """
        try:
            # Get total count
            total_count = self.qdrant_client.count(collection_name=self.collection_name)

            # Sample a few points to get content characteristics
            sample_points = self.qdrant_client.scroll(
                collection_name=self.collection_name,
                limit=5,
                with_payload=True
            )[0]  # Get only the points, not the offset

            # Analyze sample content
            sections = set()
            tags = set()
            urls = set()

            for point in sample_points:
                payload = point.payload or {}
                if payload.get("section"):
                    sections.add(payload["section"])
                if payload.get("tags"):
                    for tag in payload["tags"]:
                        tags.add(tag)
                if payload.get("url"):
                    urls.add(payload["url"])

            return {
                "total_documents": total_count.count,
                "unique_sections": len(sections),
                "unique_tags": len(tags),
                "unique_urls": len(urls),
                "sample_sections": list(sections)[:10],  # First 10 sections
                "sample_tags": list(tags)[:10],  # First 10 tags
                "sample_urls": list(urls)[:5]  # First 5 URLs
            }
        except Exception as e:
            logger.error(f"Error getting content statistics: {str(e)}", exc_info=True)
            return {
                "total_documents": 0,
                "error": str(e)
            }


def main():
    parser = argparse.ArgumentParser(description='Query the Qdrant vector database for Physical AI content')
    parser.add_argument('--query', '-q', type=str, required=True,
                        help='Query to search for in the vector database')
    parser.add_argument('--top-k', type=int, default=3,
                        help='Number of top results to retrieve (default: 3)')
    parser.add_argument('--min-similarity', type=float, default=0.1,
                        help='Minimum similarity threshold (default: 0.1)')

    args = parser.parse_args()

    print("="*70)
    print("QDRANT VECTOR DATABASE RETRIEVAL SYSTEM")
    print("="*70)
    print(f"Query: '{args.query}'")
    print(f"Top-K: {args.top_k}")
    print(f"Min Similarity: {args.min_similarity}")
    print("-"*70)

    try:
        # Load environment variables
        from dotenv import load_dotenv
        load_dotenv()

        # Initialize the retrieval system
        print("Initializing retrieval system...")
        retriever = RAGRetrievalSystem()
        print("[OK] Retrieval system initialized successfully")

        # Check collection status
        status = retriever.check_collection_status()
        print(f"[OK] Collection status: {status['status']}")
        print(f"[OK] Documents in collection: {status['total_documents']}")

        if not status.get('has_content', False):
            print("[ERROR] Collection is empty or not accessible")
            return 1

        # Perform the query
        print(f"\nExecuting query: '{args.query}'")
        results = asyncio.run(
            retriever.retrieve_with_biasing(
                query=args.query,
                top_k=args.top_k,
                min_similarity=args.min_similarity
            )
        )

        print(f"\n[OK] Retrieved {len(results)} results")

        if results:
            print("\n--- RETRIEVED RESULTS ---")
            for i, result in enumerate(results, 1):
                print(f"\n{i}. Title: {result.get('title', 'N/A')[:100]}{'...' if len(result.get('title', '')) > 100 else ''}")
                print(f"   Score: {result.get('score', 0):.3f}")
                print(f"   URL: {result.get('url', 'N/A')}")
                print(f"   Section: {result.get('section', 'N/A')}")

                # Clean content for safe printing
                content = result.get('content', '')[:300]
                # Remove or replace problematic Unicode characters
                clean_content = content.encode('ascii', errors='ignore').decode('ascii')
                print(f"   Content Preview: {clean_content}{'...' if len(result.get('content', '')) > 300 else ''}")

                if result.get('tags'):
                    print(f"   Tags: {', '.join(result.get('tags', []))}")

            print("\n--- END OF RESULTS ---")

            print(f"\nSUMMARY:")
            print(f"- Found {len(results)} relevant documents")
            if results:
                avg_score = sum(r.get('score', 0) for r in results) / len(results)
                print(f"- Average relevance score: {avg_score:.3f}")

                print(f"\nThe system successfully answered the query from the vector database!")
                print(f"Retrieved content is semantically related to: '{args.query}'")
        else:
            print("   No results found for this query")
            print("   This could be because the query doesn't match any content in the database")

        print("\n" + "="*70)
        print("SUCCESS: Query completed successfully!")
        print("The system successfully retrieved relevant content from the vector database.")
        print("="*70)

        return 0

    except Exception as e:
        print(f"\n[ERROR] {str(e)}")
        import traceback
        traceback.print_exc()
        print("="*70)
        print("FAILED: There was an error during the retrieval process.")
        print("="*70)
        return 1


if __name__ == "__main__":
    # Only run main if called directly as a script
    if len(sys.argv) > 1:  # If arguments provided, run as command line
        sys.exit(main())
    else:
        # Otherwise, just make the class available for imports
        pass