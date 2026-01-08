#!/usr/bin/env python3
"""
Test script to ingest only the chapter content from the sitemap
"""

import requests
from xml.etree import ElementTree as ET
from main import RAGEmbeddingPipeline

def get_chapter_urls(sitemap_url):
    """Get only the chapter and hardware-related URLs from the sitemap."""
    response = requests.get(sitemap_url)
    response.raise_for_status()

    root = ET.fromstring(response.content)

    # Extract all URLs
    urls = []
    for url_element in root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}loc'):
        urls.append(url_element.text)

    # Filter for documentation URLs (chapters, lessons, hardware)
    chapter_urls = [url for url in urls
                   if '/docs/' in url and
                   ('/chapter' in url or '/hardware' in url or url == 'https://physicalaiandhumanoidrobotics.vercel.app/docs')]

    return chapter_urls

def main():
    print("Starting chapter content ingestion...")

    # Get the chapter URLs
    chapter_urls = get_chapter_urls('https://physicalaiandhumanoidrobotics.vercel.app/sitemap.xml')

    print(f"Found {len(chapter_urls)} chapter/hardware URLs to process")
    for i, url in enumerate(chapter_urls[:10]):  # Show first 10
        print(f"{i+1}. {url}")

    if len(chapter_urls) > 10:
        print(f"... and {len(chapter_urls) - 10} more URLs")

    # Initialize the RAG pipeline
    pipeline = RAGEmbeddingPipeline()

    # Process first 5 URLs as a test
    test_urls = chapter_urls[:5]
    print(f"\nTesting with first 5 URLs...")

    success = pipeline.run_full_pipeline(test_urls)

    if success:
        print(f"\nChapter content ingestion test completed successfully!")
        print(f"Processed {len(test_urls)} URLs")
    else:
        print(f"\nChapter content ingestion test failed.")
        return False

    # If the test was successful, process all chapter URLs
    print(f"\nNow processing all {len(chapter_urls)} chapter URLs...")
    success = pipeline.run_full_pipeline(chapter_urls)

    if success:
        print(f"\nComplete chapter content ingestion completed successfully!")
        print(f"Processed {len(chapter_urls)} URLs")
        return True
    else:
        print(f"\nComplete chapter content ingestion failed.")
        return False

if __name__ == "__main__":
    main()