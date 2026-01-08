#!/usr/bin/env python3
"""
Script to run the RAG pipeline with deployed Vercel URLs
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from main import RAGEmbeddingPipeline

def get_book_urls():
    """Get the list of URLs from your deployed Vercel site."""
    # These are example URLs based on typical Docusaurus structure
    # You should update these to match your actual deployed pages
    base_url = "https://physicalaiandhumanoidrobotics.vercel.app"

    urls = [
        f"{base_url}/",
        f"{base_url}/introduction",
        f"{base_url}/overview",
        # Add more specific chapter/module URLs as needed
        # f"{base_url}/module1",
        # f"{base_url}/module2",
        # f"{base_url}/lesson1",
        # etc.
    ]

    # For a more comprehensive approach, you might want to use a sitemap
    # Or get URLs from a sitemap.xml if available
    sitemap_url = f"{base_url}/sitemap.xml"
    print(f"Consider checking {sitemap_url} for a complete list of available URLs")

    return urls

def main():
    """Run the RAG pipeline with your deployed URLs."""
    print("Initializing RAG Embeddings Pipeline...")

    try:
        # Initialize the pipeline
        pipeline = RAGEmbeddingPipeline()
        print("✓ Pipeline initialized successfully")

        # Get your deployed URLs
        print("\nFetching URLs from deployed Vercel site...")
        urls = get_book_urls()
        print(f"Found {len(urls)} URLs to process:")
        for url in urls:
            print(f"  - {url}")

        if not urls:
            print("No URLs found. Please update the get_book_urls() function with your actual URLs.")
            return False

        print(f"\nProcessing {len(urls)} URLs and storing embeddings in Qdrant collection: {pipeline.qdrant_client._get_collection_name if hasattr(pipeline.qdrant_client, '_get_collection_name') else 'physical_ai'}")

        # Run the full pipeline
        success = pipeline.run_full_pipeline(urls)

        if success:
            print("\n🎉 Pipeline completed successfully!")
            print("Embeddings have been generated and stored in your Qdrant collection.")

            # Test a simple search to verify
            print("\nTesting search functionality...")
            try:
                results = pipeline.search_similar_content("Physical AI", top_k=3)
                print(f"Search test returned {len(results)} results")
                if results:
                    print("Sample result:", results[0]['payload']['title'] if results[0]['payload'].get('title') else 'No title available')
            except Exception as e:
                print(f"Search test failed (this is normal if collection is still being populated): {e}")

            return True
        else:
            print("\n❌ Pipeline failed.")
            return False

    except Exception as e:
        print(f"❌ Error running pipeline: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)