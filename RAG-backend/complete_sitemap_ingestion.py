#!/usr/bin/env python3
"""
Complete Sitemap Ingestion Script for Physical AI and Humanoid Robotics Textbook

This script performs comprehensive data ingestion from the sitemap.xml,
focusing on the actual working URLs of the Physical AI textbook.
"""

import requests
from xml.etree import ElementTree as ET
from main import RAGEmbeddingPipeline
import time
from urllib.parse import urljoin, urlparse


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

    Args:
        url: URL to validate
        timeout: Request timeout in seconds

    Returns:
        True if URL is accessible (status < 400), False otherwise
    """
    try:
        response = requests.head(url, timeout=timeout, allow_redirects=True)
        return response.status_code < 400
    except:
        # If HEAD fails, try GET with a short timeout and limited content
        try:
            response = requests.get(url, timeout=timeout, stream=True)
            # Just read a small amount to verify it's accessible
            for chunk in response.iter_content(chunk_size=1):
                break  # Just verify we can read something
            response.close()
            return response.status_code < 400
        except:
            return False


def filter_working_urls(urls, max_urls=50):
    """
    Filter URLs to only include working ones, with a limit to avoid too many requests.

    Args:
        urls: List of URLs to filter
        max_urls: Maximum number of URLs to validate (to avoid rate limiting)

    Returns:
        List of working URLs
    """
    print(f"Validating a sample of {min(max_urls, len(urls))} URLs from {len(urls)} total...")

    working_urls = []
    checked_count = 0

    for url in urls:
        if checked_count >= max_urls:
            break

        print(f"Checking: {url}")
        if validate_url(url):
            working_urls.append(url)
            print(f"  ✓ Working")
        else:
            print(f"  ✗ Not accessible")

        checked_count += 1

        # Add a small delay to be respectful to the server
        time.sleep(0.1)

    return working_urls


def main():
    print("Starting complete sitemap ingestion for Physical AI and Humanoid Robotics Textbook...")

    try:
        # Fetch all URLs from sitemap
        print("Fetching URLs from sitemap...")
        all_urls = fetch_sitemap_urls('https://physicalaiandhumanoidrobotics.vercel.app/sitemap.xml')
        print(f"Found {len(all_urls)} URLs in sitemap")

        # Filter for documentation-related URLs since those are more likely to be the textbook content
        docs_urls = [url for url in all_urls if '/docs/' in url]
        print(f"Filtered to {len(docs_urls)} documentation URLs")

        # Further filter to likely textbook content
        textbook_urls = [url for url in docs_urls
                        if any(pattern in url for pattern in
                              ['/chapter', '/lesson', '/module', '/hardware', '/overview', '/detailing'])]
        print(f"Filtered to {len(textbook_urls)} likely textbook content URLs")

        # Validate which URLs are actually working
        working_urls = filter_working_urls(textbook_urls)
        print(f"Found {len(working_urls)} working URLs")

        if not working_urls:
            print("No working URLs found. Let's try the main textbook pages...")
            # Try some known main pages
            main_pages = [
                'https://physicalaiandhumanoidrobotics.vercel.app/',
                'https://physicalaiandhumanoidrobotics.vercel.app/docs',
                'https://physicalaiandhumanoidrobotics.vercel.app/docs/hardware',
                'https://physicalaiandhumanoidrobotics.vercel.app/docs/assessments'
            ]

            working_urls = []
            for page in main_pages:
                if validate_url(page):
                    working_urls.append(page)
                    print(f"Added working main page: {page}")

        if not working_urls:
            print("No working URLs found. The site may have configuration issues.")
            print("This appears to be a Docusaurus site with a baseUrl configuration problem.")
            print("The sitemap may be outdated or the site may need to be redeployed with correct configuration.")
            return False

        print(f"Processing {len(working_urls)} working URLs...")
        for url in working_urls:
            print(f"  - {url}")

        # Initialize the RAG pipeline
        pipeline = RAGEmbeddingPipeline()

        # Process the working URLs
        success = pipeline.run_full_pipeline(working_urls)

        if success:
            print(f"\nComplete sitemap ingestion completed successfully!")
            print(f"Processed {len(working_urls)} working URLs")
            return True
        else:
            print(f"\nSitemap ingestion partially failed.")
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