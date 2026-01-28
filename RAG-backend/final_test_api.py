#!/usr/bin/env python3
"""
Final test script to verify the complete API integration between frontend and backend
"""

import asyncio
import aiohttp
import time
import sys


async def test_api_connection():
    """
    Test basic API connectivity and functionality
    """
    print("="*80)
    print("FINAL API INTEGRATION TEST")
    print("Testing backend-frontend connection through FastAPI endpoints")
    print("="*80)

    base_url = "http://localhost:8000"

    # Test 1: Health check
    print("\n1. Testing health endpoint...")
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(f"{base_url}/health") as health_resp:
                health_data = await health_resp.json()
                print(f"   SUCCESS: Health status: {health_data.get('status', 'unknown')}")
                print(f"   SUCCESS: Services: {health_data.get('services', {})}")
    except Exception as e:
        print(f"   ERROR: Health check failed: {str(e)}")
        return False

    # Test 2: Status endpoint
    print("\n2. Testing status endpoint...")
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(f"{base_url}/status") as status_resp:
                status_data = await status_resp.json()
                print(f"   SUCCESS: Status: {status_data.get('status', 'unknown')}")
                print(f"   SUCCESS: Vector DB has content: {status_data.get('vector_db_status', {}).get('has_content', False)}")
                print(f"   SUCCESS: Documents in DB: {status_data.get('vector_db_status', {}).get('total_documents', 0)}")
    except Exception as e:
        print(f"   ERROR: Status check failed: {str(e)}")
        return False

    # Test 3: Main chat functionality
    print("\n3. Testing main chat endpoint with query processing...")

    test_query = {
        "query": "What is Disembodiment?",
        "selected_text": "",
        "session_id": None,
        "top_k": 3,
        "min_similarity": 0.1,
        "temperature": 0.7
    }

    print(f"   Sending query: '{test_query['query']}'")

    try:
        start_time = time.time()

        async with aiohttp.ClientSession() as session:
            async with session.post(
                f"{base_url}/chat",
                json=test_query,
                timeout=aiohttp.ClientTimeout(total=60)  # 60 second timeout
            ) as resp:
                response_data = await resp.json()

                if resp.status != 200:
                    print(f"   ERROR: Request failed with status {resp.status}: {response_data}")
                    return False

                processing_time = time.time() - start_time

                print(f"   SUCCESS: Request completed in {processing_time:.2f}s")
                print(f"   SUCCESS: Response status: {resp.status}")
                print(f"   SUCCESS: Response length: {len(response_data.get('response', ''))} characters")
                print(f"   SUCCESS: Sources retrieved: {len(response_data.get('sources', []))}")

                # Check response structure
                required_fields = ['response', 'sources', 'session_id', 'metadata']
                missing_fields = [field for field in required_fields if field not in response_data]
                if missing_fields:
                    print(f"   ERROR: Missing fields in response: {missing_fields}")
                    return False
                else:
                    print(f"   SUCCESS: All required fields present")

                # Check sources structure
                sources = response_data.get('sources', [])
                if sources:
                    sample_source = sources[0]
                    source_required_fields = ['id', 'title', 'url', 'content', 'section', 'tags', 'score', 'similarity']
                    missing_source_fields = [field for field in source_required_fields if field not in sample_source]
                    if not missing_source_fields:
                        print(f"   SUCCESS: Source structure correct")
                    else:
                        print(f"   ERROR: Missing source fields: {missing_source_fields}")
                        return False

                # Show sample response
                response_preview = response_data['response'][:200].encode('ascii', errors='ignore').decode('ascii')
                print(f"   SUCCESS: Response preview: {response_preview}...")

                return True

    except asyncio.TimeoutError:
        print(f"   ERROR: Request timed out after 60 seconds")
        return False
    except Exception as e:
        print(f"   ERROR: Chat request failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


async def main():
    """Main function to run the integration test."""
    print("Starting final API integration test...")

    # Test the main integration
    success = await test_api_connection()

    print(f"\n{'='*80}")
    if success:
        print("SUCCESS: API integration test passed!")
        print("The FastAPI backend is properly connected to the RAG agent.")
        print("Queries flow correctly from the API endpoint through the agent.")
        print("Retrieval and generation are working end-to-end.")
        print("System is ready for frontend integration.")
    else:
        print("FAILURE: API integration test failed!")
        print("There are issues with the API integration.")
    print(f"{'='*80}")

    return success


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)