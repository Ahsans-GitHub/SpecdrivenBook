#!/usr/bin/env python3
"""
Test script to verify Qdrant vector database retrieval functionality
Usage: python test_retrieval.py --query "your query here"
"""

import argparse
import asyncio
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from retrieval import RAGRetrievalSystem


def test_retrieval_system():
    """Test the retrieval system with a sample query."""
    print("Initializing RAG Retrieval System...")

    try:
        # Initialize the retrieval system
        retriever = RAGRetrievalSystem()
        print("✓ Retrieval system initialized successfully")

        # Check collection status
        status = retriever.check_collection_status()
        print(f"✓ Collection status: {status['status']}")
        print(f"✓ Total documents in database: {status['total_documents']}")
        print(f"✓ Content available: {status['has_content']}")

        return retriever

    except Exception as e:
        print(f"✗ Error initializing retrieval system: {str(e)}")
        return None


def test_query_retrieval(retriever, query):
    """Test query retrieval from the vector database."""
    print(f"\nSearching for: '{query}'")

    try:
        # Perform retrieval with biasing
        results = asyncio.run(
            retriever.retrieve_with_biasing(
                query=query,
                top_k=5,  # Get top 5 results
                min_similarity=0.1  # Lower threshold to get more results
            )
        )

        print(f"✓ Found {len(results)} relevant results")

        if results:
            print("\n--- Retrieved Results ---")
            for i, result in enumerate(results, 1):
                print(f"\n{i}. Title: {result.get('title', 'N/A')}")
                print(f"   URL: {result.get('url', 'N/A')}")
                print(f"   Section: {result.get('section', 'N/A')}")
                print(f"   Similarity: {result.get('similarity', 0):.3f}")
                print(f"   Content Preview: {result.get('content', '')[:200]}...")

                if result.get('tags'):
                    print(f"   Tags: {', '.join(result.get('tags', []))}")
        else:
            print("⚠ No relevant results found for this query")

        return results

    except Exception as e:
        print(f"✗ Error during query retrieval: {str(e)}")
        import traceback
        traceback.print_exc()
        return []


def main():
    parser = argparse.ArgumentParser(description='Test Qdrant vector database retrieval')
    parser.add_argument('--query', '-q', type=str, required=True,
                       help='Query to search for in the vector database')

    args = parser.parse_args()

    print("="*60)
    print("QDRANT VECTOR DATABASE RETRIEVAL TEST")
    print("="*60)

    # Test the retrieval system
    retriever = test_retrieval_system()

    if not retriever:
        print("Cannot proceed without a working retrieval system.")
        return 1

    # Test the query retrieval
    results = test_query_retrieval(retriever, args.query)

    print("\n" + "="*60)
    if results:
        print("SUCCESS: Query matched relevant content from the database!")
    else:
        print("INFO: Query did not match any content (this may be normal depending on the query)")
    print("="*60)

    return 0


if __name__ == "__main__":
    sys.exit(main())