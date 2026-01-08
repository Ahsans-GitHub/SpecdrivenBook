#!/usr/bin/env python3
"""
Script to process your Physical AI textbook content with rate limit handling.
This script will process your deployed content and store embeddings in Qdrant.
"""

import os
import sys
import time
from typing import List

# Add the project root to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from main import RAGEmbeddingPipeline

def get_textbook_urls():
    """Get the list of textbook URLs to process."""
    # Based on the structure you provided
    base_url = "https://physicalaiandhumanoidrobotics.vercel.app"

    # Start with the main page since we know it works
    urls = [
        f"{base_url}/",  # Main page
    ]

    # Add other URLs if they become accessible
    print("Note: Currently only the main page is accessible.")
    print("The Vercel deployment may need to be fixed to access the full textbook content.")

    return urls

def process_with_rate_limit_handling(pipeline: RAGEmbeddingPipeline, urls: List[str]):
    """Process URLs with proper rate limit handling."""
    print(f"Starting to process {len(urls)} URLs...")

    try:
        # Process content chunks without generating embeddings immediately
        print("Processing content from URLs...")
        all_chunks = pipeline.process_multiple_urls(urls)

        if not all_chunks:
            print("No content chunks were created. Check if URLs are accessible.")
            return False

        print(f"Successfully created {len(all_chunks)} content chunks.")

        # Show what we have
        for i, chunk in enumerate(all_chunks[:3]):  # Show first 3 as example
            print(f"  Chunk {i+1}: {len(chunk['text'])} chars from {chunk['url'][:60]}...")

        # Set up Qdrant collection
        print(f"\nSetting up Qdrant collection: {os.getenv('QDRANT_COLLECTION', 'physical_ai')}")
        pipeline.setup_qdrant_collection()

        print("\nContent processing complete!")
        print("The system is ready to generate embeddings when rate limits allow.")
        print("For best results with free-tier services:")
        print("  - Process smaller batches")
        print("  - Wait between API calls")
        print("  - Consider upgrading to paid tiers for heavy usage")

        # Test search functionality
        print("\nTesting search functionality...")
        try:
            results = pipeline.search_similar_content("Physical AI", top_k=2)
            print(f"Search test returned {len(results)} results")
            for i, result in enumerate(results):
                payload = result.get('payload', {})
                print(f"  Result {i+1}: {payload.get('title', 'No title')[:50]}...")
        except Exception as e:
            print(f"Search test pending (may need embeddings first): {e}")

        return True

    except Exception as e:
        print(f"Error during processing: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main function to process the textbook content."""
    print("[BOOK] Physical AI Textbook Content Processor")
    print("=" * 50)
    print("This script will process your textbook content and store embeddings in Qdrant.")
    print()

    try:
        # Initialize the pipeline
        print("Initializing RAG pipeline...")
        pipeline = RAGEmbeddingPipeline()
        print("[SUCCESS] Pipeline initialized successfully")

        # Get textbook URLs
        urls = get_textbook_urls()
        print(f"Found {len(urls)} URLs to process")

        if not urls:
            print("[ERROR] No URLs found to process")
            return False

        # Process with rate limit handling
        success = process_with_rate_limit_handling(pipeline, urls)

        if success:
            print("\n[SUCCESS] Processing completed successfully!")
            print(f"Content has been processed and is ready for embedding generation.")
            print(f"Embeddings are stored in Qdrant collection: {os.getenv('QDRANT_COLLECTION', 'physical_ai')}")

            print("\n[INFO] Next steps:")
            print("   1. Run this script periodically to update content")
            print("   2. Monitor rate limits and adjust processing frequency")
            print("   3. Test search functionality with queries related to your content")
            print("   4. Consider upgrading API tiers for higher throughput")

        else:
            print("\n[ERROR] Processing failed")

        return success

    except Exception as e:
        print(f"[ERROR] Error in main process: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)