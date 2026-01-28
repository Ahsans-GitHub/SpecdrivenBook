#!/usr/bin/env python3
"""
Test script to verify Qdrant vector database retrieval functionality
Usage: python retrieval_test.py --query "your query here"
"""

import argparse
import asyncio
import sys
import os

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

from retrieval import RAGRetrievalSystem


def main():
    parser = argparse.ArgumentParser(description='Test Qdrant vector database retrieval')
    parser.add_argument('--query', '-q', type=str, required=True,
                        help='Query to search for in the vector database')
    parser.add_argument('--top-k', type=int, default=3,
                        help='Number of top results to retrieve (default: 3)')
    parser.add_argument('--min-similarity', type=float, default=0.1,
                        help='Minimum similarity threshold (default: 0.1)')

    args = parser.parse_args()

    print("="*70)
    print("QDRANT VECTOR DATABASE RETRIEVAL TEST")
    print("="*70)
    print(f"Query: '{args.query}'")
    print(f"Top-K: {args.top_k}")
    print(f"Min Similarity: {args.min_similarity}")
    print("-"*70)

    try:
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
        print(f"\nPerforming query: '{args.query}'")
        results = asyncio.run(
            retriever.retrieve_with_biasing(
                query=args.query,
                top_k=args.top_k,
                min_similarity=args.min_similarity
            )
        )

        print(f"\n[OK] Retrieved {len(results)} results")

        if results:
            print("\n--- RESULTS ---")
            for i, result in enumerate(results, 1):
                print(f"\n{i}. Title: {result.get('title', 'N/A')}")
                print(f"   Score: {result.get('score', 0):.3f}")
                print(f"   URL: {result.get('url', 'N/A')}")
                print(f"   Section: {result.get('section', 'N/A')}")
                content = result.get('content', '')[:300]
                # Remove or replace problematic Unicode characters
                content = content.encode('ascii', errors='ignore').decode('ascii')
                print(f"   Content: {content}...")
                if result.get('tags'):
                    print(f"   Tags: {', '.join(result.get('tags', []))}")
            print("\n--- END RESULTS ---")
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
    sys.exit(main())