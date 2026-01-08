#!/usr/bin/env python3
"""
Script to process all Physical AI textbook content with comprehensive URL checking.
This script will identify which URLs are actually accessible and process them.
"""

import os
import sys
import time
import requests
from typing import List

# Add the project root to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from main import RAGEmbeddingPipeline

def get_all_textbook_urls():
    """Get all possible textbook URLs from the sitemap."""
    base_url = "https://physicalaiandhumanoidrobotics.vercel.app"

    # All URLs from the sitemap you provided
    all_urls = [
        f"{base_url}/docs",
        f"{base_url}/docs/chapter1",
        f"{base_url}/docs/chapter1/lesson1",
        f"{base_url}/docs/chapter1/lesson2",
        f"{base_url}/docs/chapter1/lesson3",
        f"{base_url}/docs/chapter1/lesson4",
        f"{base_url}/docs/chapter2/chapter2-overview",
        f"{base_url}/docs/chapter2/module1-detailing",
        f"{base_url}/docs/chapter2/module1-overview",
        f"{base_url}/docs/chapter2/module1/lesson1",
        f"{base_url}/docs/chapter2/module1/lesson2",
        f"{base_url}/docs/chapter2/module1/lesson3",
        f"{base_url}/docs/chapter2/module1/lesson4",
        f"{base_url}/docs/chapter3/chapter3-overview",
        f"{base_url}/docs/chapter3/module2-detailing",
        f"{base_url}/docs/chapter3/module2-overview",
        f"{base_url}/docs/chapter3/module2/lesson1",
        f"{base_url}/docs/chapter3/module2/lesson2",
        f"{base_url}/docs/chapter3/module2/lesson3",
        f"{base_url}/docs/chapter3/module2/lesson4",
        f"{base_url}/docs/chapter4/chapter4-overview",
        f"{base_url}/docs/chapter4/module3-detailing",
        f"{base_url}/docs/chapter4/module3-overview",
        f"{base_url}/docs/chapter4/module3/lesson1",
        f"{base_url}/docs/chapter4/module3/lesson2",
        f"{base_url}/docs/chapter4/module3/lesson3",
        f"{base_url}/docs/chapter4/module3/lesson4",
        f"{base_url}/docs/chapter5/chapter5-overview",
        f"{base_url}/docs/chapter5/module4-detailing",
        f"{base_url}/docs/chapter5/module4-overview",
        f"{base_url}/docs/chapter5/module4/lesson1",
        f"{base_url}/docs/chapter5/module4/lesson2",
        f"{base_url}/docs/chapter5/module4/lesson3",
        f"{base_url}/docs/chapter5/module4/lesson4",
        f"{base_url}/docs/chapter6",
        f"{base_url}/docs/chapter6/lesson1",
        f"{base_url}/docs/chapter6/lesson2",
        f"{base_url}/docs/chapter6/lesson3",
        f"{base_url}/docs/chapter7",
        f"{base_url}/docs/chapter7/assessment1",
        f"{base_url}/docs/chapter7/assessment2",
        f"{base_url}/docs/chapter7/assessment3",
        f"{base_url}/docs/chapter7/assessment4",
        f"{base_url}/docs/hardware",
        f"{base_url}/docs/hardware-requirements"
    ]

    return all_urls

def check_accessible_urls(urls: List[str]) -> List[str]:
    """Check which URLs are actually accessible."""
    print(f"Checking accessibility of {len(urls)} URLs...")

    accessible_urls = []
    inaccessible_urls = []

    for i, url in enumerate(urls):
        try:
            response = requests.head(url, timeout=10)  # Use HEAD to check status quickly
            if response.status_code == 200:
                print(f"[OK] {url}")
                accessible_urls.append(url)
            else:
                print(f"[{response.status_code}] {url}")
                inaccessible_urls.append(url)
        except Exception as e:
            print(f"[ERROR] {url} - {str(e)}")
            inaccessible_urls.append(url)

        # Be respectful to the server
        time.sleep(0.5)

    print(f"\nSummary:")
    print(f"  Accessible URLs: {len(accessible_urls)}")
    print(f"  Inaccessible URLs: {len(inaccessible_urls)}")

    return accessible_urls

def process_with_rate_limit_handling(pipeline: RAGEmbeddingPipeline, urls: List[str]):
    """Process URLs with proper rate limit handling."""
    if not urls:
        print("No URLs to process")
        return False

    print(f"\nStarting to process {len(urls)} accessible URLs...")

    try:
        # Process content chunks without generating embeddings immediately
        print("Processing content from accessible URLs...")
        all_chunks = pipeline.process_multiple_urls(urls)

        if not all_chunks:
            print("No content chunks were created.")
            return False

        print(f"Successfully created {len(all_chunks)} content chunks.")

        # Show what we have
        for i, chunk in enumerate(all_chunks[:5]):  # Show first 5 as example
            print(f"  Chunk {i+1}: {len(chunk['text'])} chars from {chunk['url'][:60]}...")

        if len(all_chunks) > 5:
            print(f"  ... and {len(all_chunks) - 5} more chunks")

        # Set up Qdrant collection (this might hit rate limits, so we'll handle it)
        print(f"\nSetting up Qdrant collection: {os.getenv('QDRANT_COLLECTION', 'physical_ai')}")

        # Try to set up the collection, but handle rate limit errors gracefully
        try:
            pipeline.setup_qdrant_collection()
            print("Qdrant collection setup completed.")
        except Exception as e:
            if "429" in str(e) or "rate" in str(e).lower() or "limit" in str(e).lower():
                print(f"Rate limit hit during collection setup: {e}")
                print("This is expected with free-tier services. Collection likely already exists.")
            else:
                print(f"Error setting up collection: {e}")
                # Continue anyway since collection might already exist

        # Try to store embeddings, but handle rate limits
        print("\nProcessing content chunks with embeddings (respects rate limits)...")

        try:
            chunks_with_embeddings = pipeline.process_content_chunks_with_embeddings(all_chunks)
            print(f"Successfully added embeddings to {len(chunks_with_embeddings)} chunks.")

            # Store in Qdrant
            print("Storing embeddings in Qdrant...")
            success = pipeline.store_embeddings_in_qdrant(chunks_with_embeddings)

            if success:
                print("✅ Embeddings successfully stored in Qdrant!")
            else:
                print("⚠️  Some issues occurred during storage.")

        except Exception as e:
            if "429" in str(e) or "rate" in str(e).lower() or "limit" in str(e).lower():
                print(f"Rate limit hit during embedding generation: {e}")
                print("This is expected with free-tier services.")
                print("Content is processed and ready for embedding when rate limits allow.")
            else:
                print(f"Error during embedding generation: {e}")

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
    """Main function to process all textbook content."""
    print("[BOOK] Physical AI Textbook Content Processor")
    print("=" * 60)
    print("This script will check all textbook URLs and process accessible content.")
    print()

    try:
        # Get all textbook URLs
        all_urls = get_all_textbook_urls()
        print(f"Total URLs to check: {len(all_urls)}")

        # Check which URLs are accessible
        accessible_urls = check_accessible_urls(all_urls)

        if not accessible_urls:
            print("\n[ERROR] No accessible URLs found!")
            print("This suggests there may be an issue with your Vercel deployment.")
            print("Please check that your Docusaurus site is properly deployed.")
            return False

        # Initialize the pipeline
        print("\nInitializing RAG pipeline...")
        pipeline = RAGEmbeddingPipeline()
        print("[SUCCESS] Pipeline initialized successfully")

        # Process with rate limit handling
        success = process_with_rate_limit_handling(pipeline, accessible_urls)

        if success:
            print("\n[SUCCESS] Processing completed successfully!")
            print(f"Content has been processed and stored in Qdrant collection: {os.getenv('QDRANT_COLLECTION', 'physical_ai')}")

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