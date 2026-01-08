#!/usr/bin/env python3
"""
Simple Sitemap Ingestion Script for Physical AI and Humanoid Robotics Textbook
"""

import requests
from xml.etree import ElementTree as ET
from main import RAGEmbeddingPipeline
import time


def fetch_sitemap_urls(sitemap_url: str):
    """Fetch and parse sitemap to get all URLs."""
    response = requests.get(sitemap_url)
    response.raise_for_status()

    root = ET.fromstring(response.content)
    urls = []
    for url_element in root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}loc'):
        urls.append(url_element.text)

    return urls


def validate_url(url: str, timeout: int = 5) -> bool:
    """
    Validate if a URL is accessible.
    """
    try:
        response = requests.head(url, timeout=timeout, allow_redirects=True)
        return response.status_code < 400
    except:
        return False


def main():
    print("Starting sitemap ingestion for Physical AI and Humanoid Robotics Textbook...")

    try:
        # Fetch all URLs from sitemap
        print("Fetching URLs from sitemap...")
        all_urls = fetch_sitemap_urls('https://physicalaiandhumanoidrobotics.vercel.app/sitemap.xml')
        print(f"Found {len(all_urls)} URLs in sitemap")

        # Filter for documentation-related URLs since those are more likely to be the textbook content
        docs_urls = [url for url in all_urls if '/docs/' in url and ('/chapter' in url or '/hardware' in url or '/module' in url)]
        print(f"Filtered to {len(docs_urls)} textbook content URLs")

        # Validate which URLs are actually working (limit to first 10 to avoid too many requests)
        working_urls = []
        for i, url in enumerate(docs_urls[:10]):  # Check first 10 URLs
            print(f"Checking URL {i+1}/10: {url}")
            if validate_url(url):
                working_urls.append(url)
                print("  Working")
            else:
                print("  Not accessible")
            time.sleep(0.2)  # Be respectful to the server

        print(f"Found {len(working_urls)} working URLs out of {len(docs_urls[:10])} checked")

        # If we have working URLs, process them
        if working_urls:
            print(f"Processing {len(working_urls)} working URLs...")

            # Initialize the RAG pipeline
            pipeline = RAGEmbeddingPipeline()

            # Process the working URLs
            success = pipeline.run_full_pipeline(working_urls)

            if success:
                print(f"Sitemap ingestion completed successfully!")
                print(f"Processed {len(working_urls)} working URLs")
                return True
            else:
                print("Sitemap ingestion failed.")
                return False
        else:
            print("No working URLs found in the documentation section.")
            print("The deployed site may have configuration issues.")

            # Try the main pages as fallback
            main_pages = [
                'https://physicalaiandhumanoidrobotics.vercel.app/',
                'https://physicalaiandhumanoidrobotics.vercel.app/docs'
            ]

            working_main_pages = []
            for page in main_pages:
                if validate_url(page):
                    working_main_pages.append(page)

            if working_main_pages:
                print(f"Found {len(working_main_pages)} main pages working, processing those...")
                pipeline = RAGEmbeddingPipeline()
                success = pipeline.run_full_pipeline(working_main_pages)
                return success
            else:
                print("No working pages found at all.")
                return False

    except Exception as e:
        print(f"Error during sitemap ingestion: {str(e)}")
        return False


if __name__ == "__main__":
    success = main()
    if success:
        print("\nData ingestion completed successfully!")
        print("The RAG system now has the available textbook content indexed.")
    else:
        print("\nData ingestion encountered issues.")
        print("Note: The deployed site may have configuration issues that need to be resolved.")