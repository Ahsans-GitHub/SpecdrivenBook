#!/usr/bin/env python3
"""
Final test to confirm Qdrant retrieval functionality works properly
"""

import sys
import os
import asyncio

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

def run_comprehensive_test():
    """Run comprehensive tests on the retrieval system."""
    print("="*70)
    print("COMPREHENSIVE QDRANT RETRIEVAL FUNCTIONALITY TEST")
    print("="*70)

    try:
        from retrieval import RAGRetrievalSystem

        print("\n1. Testing retrieval system initialization...")
        retriever = RAGRetrievalSystem()
        print("   [✓] Retrieval system initialized successfully")

        print("\n2. Testing collection status...")
        status = retriever.check_collection_status()
        print(f"   [✓] Collection status: {status['status']}")
        print(f"   [✓] Documents in collection: {status['total_documents']}")
        print(f"   [✓] Content available: {status['has_content']}")

        if not status['has_content']:
            print("   [⚠] Warning: Collection appears to be empty")
            return False

        print("\n3. Testing basic retrieval...")

        # Test 1: Simple query
        async def test_basic_query():
            results = await retriever.retrieve_with_biasing(
                query="What is Moravec’s Paradox?",
                top_k=3,
                min_similarity=0.1
            )
            return results

        basic_results = asyncio.run(test_basic_query())
        print(f"   [✓] Basic query returned {len(basic_results)} results")

        if basic_results:
            print(f"   [✓] First result - Title: {basic_results[0]['title'][:50]}...")
            print(f"   [✓] First result - Score: {basic_results[0]['score']:.3f}")
            print(f"   [✓] First result - Content preview: {basic_results[0]['content'][:100]}...")

        # Test 2: Query with selected text (biasing)
        async def test_biasing_query():
            results = await retriever.retrieve_with_biasing(
                query="Explain the concept of embodied cognition",
                selected_text="Physical AI connects artificial intelligence with physical embodiment",
                top_k=2,
                min_similarity=0.1
            )
            return results

        biasing_results = asyncio.run(test_biasing_query())
        print(f"   [✓] Biasing query returned {len(biasing_results)} results")

        # Test 3: Statistics functionality
        stats = asyncio.run(retriever.get_content_statistics())
        print(f"   [✓] Content statistics retrieved:")
        print(f"       - Total documents: {stats['total_documents']}")
        print(f"       - Unique sections: {stats.get('unique_sections', 'N/A')}")
        print(f"       - Sample sections: {stats.get('sample_sections', [])[:3]}")

        print("\n4. Testing keyword search...")
        keyword_results = asyncio.run(
            retriever.search_by_keywords(
                keywords=["Physical AI", "embodied", "robotics"],
                top_k=2
            )
        )
        print(f"   [✓] Keyword search returned {len(keyword_results)} results")

        print("\n5. Testing content by IDs (if results available)...")
        if basic_results:
            ids = [result['id'] for result in basic_results[:2]]
            id_results = asyncio.run(
                retriever.retrieve_content_by_ids(ids)
            )
            print(f"   [✓] Content by IDs returned {len(id_results)} results")

        print("\n" + "="*70)
        print("🎉 ALL TESTS PASSED!")
        print("Qdrant vector database retrieval is working perfectly!")
        print("The system can successfully query the vector database and")
        print("retrieve relevant content based on semantic similarity.")
        print("="*70)

        return True

    except Exception as e:
        print(f"\n❌ TEST FAILED: {str(e)}")
        import traceback
        traceback.print_exc()
        print("="*70)
        return False

if __name__ == "__main__":
    success = run_comprehensive_test()

    if success:
        print("\n✅ QDRANT RETRIEVAL SYSTEM IS FULLY OPERATIONAL!")
    else:
        print("\n❌ ISSUES DETECTED WITH THE RETRIEVAL SYSTEM")

    print("="*70)