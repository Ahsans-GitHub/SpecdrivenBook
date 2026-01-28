#!/usr/bin/env python3
"""
Simple test script to verify Qdrant vector database retrieval functionality
"""

import sys
import os
import asyncio

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

def test_qdrant_connection():
    """Test Qdrant connection and retrieval."""
    print("Testing Qdrant connection and retrieval functionality...")

    try:
        import cohere
        from qdrant_client import QdrantClient

        # Get environment variables
        cohere_api_key = os.getenv("COHERE_API_KEY")
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        collection_name = os.getenv("QDRANT_COLLECTION", "physical_ai")

        print(f"Using collection: {collection_name}")

        # Initialize clients
        cohere_client = cohere.Client(cohere_api_key)
        qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            timeout=30
        )

        print("[OK] Successfully connected to Qdrant")

        # Test collection existence
        collections = qdrant_client.get_collections()
        collection_names = [col.name for col in collections.collections]

        if collection_name in collection_names:
            print(f"[OK] Collection '{collection_name}' exists")

            # Count documents in collection
            count = qdrant_client.count(collection_name=collection_name)
            print(f"[OK] Collection contains {count.count} documents")

            if count.count > 0:
                # Test embedding generation
                print("\nTesting embedding generation...")
                test_query = "What is Physical AI?"

                response = cohere_client.embed(
                    texts=[test_query],
                    model=os.getenv("EMBEDDING_MODEL", "embed-multilingual-v3.0"),
                    input_type="search_query"
                )

                embedding = response.embeddings[0]
                print(f"[OK] Generated embedding with dimension: {len(embedding)}")

                # Test vector search
                print("\nTesting vector search...")
                search_results = qdrant_client.search(
                    collection_name=collection_name,
                    query_vector=embedding,
                    limit=3,
                    with_payload=True
                )

                print(f"[OK] Found {len(search_results)} results from vector search")

                # Show sample results
                for i, result in enumerate(search_results[:2]):  # Show first 2 results
                    payload = result.payload or {}
                    print(f"\nResult {i+1}:")
                    print(f"  ID: {result.id}")
                    print(f"  Score: {result.score:.3f}")
                    print(f"  Title: {payload.get('title', 'N/A')[:60]}...")
                    print(f"  Content: {payload.get('content', payload.get('text', ''))[:100]}...")

                print("\n" + "="*60)
                print("SUCCESS: Qdrant retrieval is working correctly!")
                print("The system can successfully query the vector database.")
                print("="*60)

                return True
            else:
                print("[WARN] Collection is empty - no documents to retrieve")
                return False
        else:
            print(f"[ERROR] Collection '{collection_name}' does not exist")
            return False

    except Exception as e:
        print(f"[ERROR] Error during testing: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("="*60)
    print("QDRANT VECTOR DATABASE RETRIEVAL TEST")
    print("="*60)

    success = test_qdrant_connection()

    print("\nTest completed.")
    if success:
        print("Qdrant retrieval functionality is working properly!")
    else:
        print("Qdrant retrieval functionality has issues.")
    print("="*60)