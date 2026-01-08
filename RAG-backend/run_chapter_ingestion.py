#!/usr/bin/env python3
"""
Script to run the RAG pipeline with Physical AI textbook chapter URLs
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from main import RAGEmbeddingPipeline

def get_chapter_urls():
    """Get the list of chapter URLs from your deployed Vercel site."""
    base_url = "https://physicalaiandhumanoidrobotics.vercel.app"

    # Chapter URLs extracted from the sitemap
    urls = [
        f"{base_url}/docs/chapter1",
        f"{base_url}/docs/chapter1/lesson1",
        f"{base_url}/docs/chapter1/lesson2",
        f"{base_url}/docs/chapter1/lesson3",
        f"{base_url}/docs/chapter1/lesson4",
        f"{base_url}/docs/chapter2/chapter2-overview",
        f"{base_url}/docs/chapter2/module1-overview",
        f"{base_url}/docs/chapter2/module1/lesson1",
        f"{base_url}/docs/chapter2/module1/lesson2",
        f"{base_url}/docs/chapter2/module1/lesson3",
        f"{base_url}/docs/chapter2/module1/lesson4",
        f"{base_url}/docs/chapter3/chapter3-overview",
        f"{base_url}/docs/chapter3/module2-overview",
        f"{base_url}/docs/chapter3/module2/lesson1",
        f"{base_url}/docs/chapter3/module2/lesson2",
        f"{base_url}/docs/chapter3/module2/lesson3",
        f"{base_url}/docs/chapter3/module2/lesson4",
        f"{base_url}/docs/chapter4/chapter4-overview",
        f"{base_url}/docs/chapter4/module3-overview",
        f"{base_url}/docs/chapter4/module3/lesson1",
        f"{base_url}/docs/chapter4/module3/lesson2",
        f"{base_url}/docs/chapter4/module3/lesson3",
        f"{base_url}/docs/chapter4/module3/lesson4",
        f"{base_url}/docs/chapter5/chapter5-overview",
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

    return urls

def main():
    """Run the RAG pipeline with your deployed textbook URLs."""
    print("Initializing RAG Embeddings Pipeline with Physical AI textbook URLs...")

    try:
        # Initialize the pipeline
        pipeline = RAGEmbeddingPipeline()
        print("[SUCCESS] Pipeline initialized successfully")

        # Get your textbook chapter URLs
        print("\nFetching Physical AI textbook chapter URLs...")
        urls = get_chapter_urls()
        print(f"Found {len(urls)} chapter URLs to process:")
        for i, url in enumerate(urls, 1):
            print(f"  {i:2d}. {url}")

        if not urls:
            print("No URLs found.")
            return False

        print(f"\nProcessing {len(urls)} textbook URLs and storing embeddings in Qdrant collection: {os.getenv('QDRANT_COLLECTION', 'physical_ai')}")

        # Run the full pipeline
        success = pipeline.run_full_pipeline(urls)

        if success:
            print("\n[SUCCESS] Pipeline completed successfully!")
            print("Embeddings have been generated and stored in your Qdrant collection.")

            # Test a simple search to verify
            print("\nTesting search functionality...")
            try:
                results = pipeline.search_similar_content("Physical AI", top_k=3)
                print(f"Search test returned {len(results)} results")
                if results:
                    print("Sample result:", results[0]['payload']['title'] if results[0]['payload'].get('title') else 'No title available')
                    print("First result URL:", results[0]['payload']['url'])

                    # Test another search with robotics
                    results2 = pipeline.search_similar_content("robotics", top_k=3)
                    print(f"Robotics search returned {len(results2)} results")
                    if results2:
                        print("Sample robotics result:", results2[0]['payload']['title'] if results2[0]['payload'].get('title') else 'No title available')
            except Exception as e:
                print(f"Search test failed: {e}")

            return True
        else:
            print("\n[ERROR] Pipeline failed.")
            return False

    except Exception as e:
        print(f"[ERROR] Error running pipeline: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)