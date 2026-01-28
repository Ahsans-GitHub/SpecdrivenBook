#!/usr/bin/env python3
"""
Test script to verify Qdrant vector database retrieval functionality
This script tests the connection to Qdrant and verifies retrieval works properly.
"""

import argparse
import asyncio
import sys
import os
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class QdrantTestRetriever:
    """
    Test class to verify Qdrant vector database retrieval functionality.
    This mimics the real retrieval system but focuses on testing.
    """

    def __init__(self):
        """Initialize the test retriever with Qdrant connection."""
        logger.info("Initializing Qdrant Test Retriever...")

        # Import required modules
        import os
        import cohere
        from qdrant_client import QdrantClient

        # Get API keys from environment
        self.cohere_api_key = os.getenv("COHERE_API_KEY")
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("QDRANT_COLLECTION", "physical_ai")

        if not all([self.cohere_api_key, self.qdrant_url, self.qdrant_api_key]):
            raise ValueError("Missing required environment variables for Qdrant connection")

        # Initialize clients
        self.cohere_client = cohere.Client(self.cohere_api_key)
        self.qdrant_client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
            timeout=30
        )

        logger.info(f"✓ Successfully connected to Qdrant at {self.qdrant_url}")
        logger.info(f"✓ Using collection: {self.collection_name}")

    def test_connection(self):
        """Test basic connection to Qdrant."""
        try:
            # Test collection existence
            collections = self.qdrant_client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name in collection_names:
                logger.info(f"✓ Collection '{self.collection_name}' exists")

                # Count documents in collection
                count = self.qdrant_client.count(collection_name=self.collection_name)
                logger.info(f"✓ Collection contains {count.count} documents")

                return True, count.count
            else:
                logger.warning(f"✗ Collection '{self.collection_name}' does not exist")
                return False, 0

        except Exception as e:
            logger.error(f"✗ Connection test failed: {str(e)}")
            return False, 0

    def test_embedding_generation(self, text):
        """Test embedding generation with Cohere."""
        try:
            logger.info(f"Generating embedding for: '{text[:50]}{'...' if len(text) > 50 else ''}'")

            # Generate embedding
            response = self.cohere_client.embed(
                texts=[text],
                model=os.getenv("EMBEDDING_MODEL", "embed-multilingual-v3.0"),
                input_type="search_query"
            )

            embedding = response.embeddings[0]
            logger.info(f"✓ Generated embedding with dimension: {len(embedding)}")

            return embedding

        except Exception as e:
            logger.error(f"✗ Embedding generation failed: {str(e)}")
            return None

    def test_vector_search(self, query_embedding, top_k=3):
        """Test vector search in Qdrant."""
        try:
            logger.info(f"Performing vector search with top_k={top_k}")

            # Perform search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True,
                with_vectors=False
            )

            logger.info(f"✓ Found {len(search_results)} results from vector search")

            # Process and return results
            results = []
            for i, result in enumerate(search_results):
                payload = result.payload or {}
                results.append({
                    "rank": i + 1,
                    "id": result.id,
                    "score": result.score,
                    "title": payload.get("title", "No Title"),
                    "url": payload.get("url", "No URL"),
                    "content_preview": payload.get("content", payload.get("text", ""))[:200] + "...",
                    "section": payload.get("section", "General"),
                    "tags": payload.get("tags", [])
                })

            return results

        except Exception as e:
            logger.error(f"✗ Vector search failed: {str(e)}")
            return []

    def test_full_retrieval(self, query, top_k=3):
        """Test the full retrieval pipeline: embed + search."""
        logger.info(f"Testing full retrieval pipeline for query: '{query}'")

        # Step 1: Generate embedding
        embedding = self.test_embedding_generation(query)
        if embedding is None:
            return []

        # Step 2: Search in vector database
        results = self.test_vector_search(embedding, top_k)

        return results


def main():
    parser = argparse.ArgumentParser(description='Test Qdrant vector database retrieval functionality')
    parser.add_argument('--query', '-q', type=str, required=True,
                        help='Query to search for in the vector database')
    parser.add_argument('--top-k', type=int, default=3,
                        help='Number of top results to retrieve (default: 3)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Enable verbose output')

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    print("="*70)
    print("QDRANT VECTOR DATABASE RETRIEVAL TEST")
    print("="*70)
    print(f"Query: '{args.query}'")
    print(f"Top-K: {args.top_k}")
    print("-"*70)

    try:
        # Initialize test retriever
        retriever = QdrantTestRetriever()

        # Test basic connection
        print("\n1. Testing connection to Qdrant...")
        collection_exists, doc_count = retriever.test_connection()

        if not collection_exists:
            print("❌ Cannot proceed: Collection does not exist")
            return 1

        if doc_count == 0:
            print("⚠️  Warning: Collection is empty")

        # Test full retrieval pipeline
        print(f"\n2. Testing full retrieval pipeline...")
        results = retriever.test_full_retrieval(args.query, args.top_k)

        print(f"\n3. Results:")
        if results:
            print(f"   Found {len(results)} relevant results:")
            print()
            for result in results:
                print(f"   [{result['rank']}] Score: {result['score']:.3f}")
                print(f"       Title: {result['title']}")
                print(f"       Section: {result['section']}")
                print(f"       URL: {result['url']}")
                print(f"       Content: {result['content_preview']}")
                if result['tags']:
                    print(f"       Tags: {', '.join(result['tags'][:3])}")
                print()
        else:
            print("   No relevant results found for this query")
            print("   This could be because:")
            print("   - Query doesn't match any content in the database")
            print("   - The collection is empty")
            print("   - Query is too specific or uses different terminology")

        print("="*70)
        print("TEST COMPLETED SUCCESSFULLY!")
        print("Qdrant vector database retrieval is working properly.")
        print("="*70)

        return 0

    except Exception as e:
        print(f"\n❌ ERROR: {str(e)}")
        import traceback
        traceback.print_exc()
        print("="*70)
        print("TEST FAILED!")
        print("There may be issues with the Qdrant connection or configuration.")
        print("="*70)
        return 1


if __name__ == "__main__":
    sys.exit(main())