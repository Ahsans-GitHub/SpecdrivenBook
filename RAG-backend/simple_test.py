#!/usr/bin/env python3
"""
Simple test to verify Qdrant vector database retrieval functionality
"""

import sys
import os
import asyncio

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_retrieval():
    """Test the retrieval system."""
    print("Testing Qdrant vector database retrieval...")

    # Import inside function to handle potential errors gracefully
    try:
        from retrieval import RAGRetrievalSystem
        print("✓ Successfully imported RAGRetrievalSystem")
    except ImportError as e:
        print(f"✗ Import error: {e}")

        # Let's try to examine the file directly
        import ast
        with open('retrieval.py', 'r') as f:
            content = f.read()

        tree = ast.parse(content)
        classes = []
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                classes.append(node.name)

        print(f"Classes found in retrieval.py: {classes}")

        # Try to dynamically load the class
        exec_globals = {}
        exec(content, exec_globals)

        if 'RAGRetrievalSystem' in exec_globals:
            RAGRetrievalSystem = exec_globals['RAGRetrievalSystem']
            print("✓ Dynamically loaded RAGRetrievalSystem")
        else:
            print("✗ RAGRetrievalSystem class not found")
            return False

    try:
        # Initialize the retrieval system
        retriever = RAGRetrievalSystem()
        print("✓ Retrieval system initialized successfully")

        # Check collection status
        status = retriever.check_collection_status()
        print(f"✓ Collection status: {status}")

        if status.get('has_content', False):
            print(f"✓ Database has {status.get('total_documents', 0)} documents")

            # Test a sample query
            print("\nTesting sample query...")
            query = "What is Physical AI?"
            results = asyncio.run(retriever.retrieve_with_biasing(
                query=query,
                top_k=3,
                min_similarity=0.1
            ))

            print(f"✓ Retrieved {len(results)} results for query: '{query}'")

            if results:
                print("\nSample results:")
                for i, result in enumerate(results[:2]):  # Show first 2 results
                    print(f"  {i+1}. Title: {result.get('title', 'N/A')[:50]}...")
                    print(f"     Content preview: {result.get('content', '')[:100]}...")
                    print(f"     Similarity: {result.get('similarity', 0):.3f}")

            return True
        else:
            print("✗ Database appears to be empty")
            return False

    except Exception as e:
        print(f"✗ Error during retrieval test: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("="*60)
    print("TESTING QDRANT VECTOR DATABASE RETRIEVAL")
    print("="*60)

    success = test_retrieval()

    print("\n" + "="*60)
    if success:
        print("SUCCESS: Qdrant retrieval is working correctly!")
    else:
        print("FAILURE: Qdrant retrieval is not working properly.")
    print("="*60)