#!/usr/bin/env python3
"""
Sitemap Ingestor for RAG Embeddings Pipeline

This script fetches and parses a sitemap.xml file to extract all URLs,
then feeds them to the RAG embeddings pipeline for processing.
"""

import argparse
import asyncio
import os
import sys
from typing import List
from xml.etree import ElementTree as ET

import requests
from urllib.parse import urljoin, urlparse

from main import RAGEmbeddingPipeline
from utils.logging import rag_logger


def fetch_sitemap_urls(sitemap_url: str) -> List[str]:
    """
    Fetch and parse a sitemap.xml file to extract all URLs.

    Args:
        sitemap_url: URL to the sitemap.xml file

    Returns:
        List of URLs extracted from the sitemap
    """
    try:
        rag_logger.info(f"Fetching sitemap from {sitemap_url}")
        response = requests.get(sitemap_url)
        response.raise_for_status()

        # Parse the XML content
        root = ET.fromstring(response.content)

        # Handle both regular sitemaps and sitemap indexes
        urls = []

        # Check if this is a sitemap index (contains sitemap elements)
        sitemap_elements = root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}sitemap')
        if sitemap_elements:
            # This is a sitemap index, need to fetch individual sitemaps
            for sitemap_elem in sitemap_elements:
                loc_elem = sitemap_elem.find('{http://www.sitemaps.org/schemas/sitemap/0.9}loc')
                if loc_elem is not None:
                    individual_sitemap_url = loc_elem.text
                    rag_logger.info(f"Fetching individual sitemap: {individual_sitemap_url}")
                    individual_urls = fetch_individual_sitemap(individual_sitemap_url)
                    urls.extend(individual_urls)
        else:
            # This is a regular sitemap with URL elements
            url_elements = root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}loc')
            urls = [elem.text for elem in url_elements if elem is not None]

        rag_logger.info(f"Successfully extracted {len(urls)} URLs from sitemap")
        return urls

    except requests.exceptions.RequestException as e:
        error_msg = f"Failed to fetch sitemap from {sitemap_url}: {str(e)}"
        rag_logger.error(error_msg)
        raise
    except ET.ParseError as e:
        error_msg = f"Failed to parse sitemap XML from {sitemap_url}: {str(e)}"
        rag_logger.error(error_msg)
        raise
    except Exception as e:
        error_msg = f"Unexpected error while processing sitemap {sitemap_url}: {str(e)}"
        rag_logger.error(error_msg)
        raise


def fetch_individual_sitemap(sitemap_url: str) -> List[str]:
    """
    Fetch and parse an individual sitemap (not a sitemap index).

    Args:
        sitemap_url: URL to the individual sitemap.xml file

    Returns:
        List of URLs extracted from the sitemap
    """
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()

        root = ET.fromstring(response.content)
        url_elements = root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}loc')
        urls = [elem.text for elem in url_elements if elem is not None]

        return urls
    except Exception as e:
        error_msg = f"Failed to process individual sitemap {sitemap_url}: {str(e)}"
        rag_logger.error(error_msg)
        raise


def filter_urls_by_domain(urls: List[str], base_domain: str) -> List[str]:
    """
    Filter URLs to only include those from a specific domain.

    Args:
        urls: List of URLs to filter
        base_domain: Domain to filter by (e.g., 'example.com')

    Returns:
        List of URLs from the specified domain
    """
    filtered_urls = []
    for url in urls:
        try:
            parsed = urlparse(url)
            if base_domain in parsed.netloc:
                filtered_urls.append(url)
        except Exception:
            # Skip invalid URLs
            continue

    return filtered_urls


def filter_urls_by_pattern(urls: List[str], include_pattern: str = None, exclude_pattern: str = None) -> List[str]:
    """
    Filter URLs based on inclusion/exclusion patterns.

    Args:
        urls: List of URLs to filter
        include_pattern: Only include URLs containing this pattern (optional)
        exclude_pattern: Exclude URLs containing this pattern (optional)

    Returns:
        List of filtered URLs
    """
    filtered_urls = urls

    if include_pattern:
        filtered_urls = [url for url in filtered_urls if include_pattern in url]

    if exclude_pattern:
        filtered_urls = [url for url in filtered_urls if exclude_pattern not in url]

    return filtered_urls


def main():
    """Main function to run the sitemap ingestion process."""
    parser = argparse.ArgumentParser(description='Sitemap Ingestor for RAG Embeddings Pipeline')
    parser.add_argument('--sitemap-url',
                       type=str,
                       default='https://physicalaiandhumanoidrobotics.vercel.app/sitemap.xml',
                       help='URL to the sitemap.xml file (default: https://physicalaiandhumanoidrobotics.vercel.app/sitemap.xml)')
    parser.add_argument('--filter-domain',
                       type=str,
                       help='Only process URLs from this domain (optional)')
    parser.add_argument('--include-pattern',
                       type=str,
                       help='Only process URLs containing this pattern (e.g., "/docs/")')
    parser.add_argument('--exclude-pattern',
                       type=str,
                       help='Exclude URLs containing this pattern (e.g., "/blog/")')
    parser.add_argument('--limit',
                       type=int,
                       help='Limit the number of URLs to process (optional)')
    parser.add_argument('--skip-urls',
                       nargs='*',
                       default=[],
                       help='List of URLs to skip during processing')

    args = parser.parse_args()

    print(f"Starting sitemap ingestion from: {args.sitemap_url}")
    rag_logger.info(f"Starting sitemap ingestion from: {args.sitemap_url}")

    try:
        # Fetch URLs from sitemap
        all_urls = fetch_sitemap_urls(args.sitemap_url)
        print(f"Found {len(all_urls)} URLs in sitemap")

        # Filter by domain if specified
        if args.filter_domain:
            all_urls = filter_urls_by_domain(all_urls, args.filter_domain)
            print(f"After domain filtering: {len(all_urls)} URLs")

        # Filter by include/exclude patterns
        urls_to_process = filter_urls_by_pattern(all_urls, args.include_pattern, args.exclude_pattern)
        if args.include_pattern or args.exclude_pattern:
            print(f"After pattern filtering: {len(urls_to_process)} URLs")

        # Remove URLs to skip
        urls_to_process = [url for url in urls_to_process if url not in args.skip_urls]
        print(f"After excluding skip URLs: {len(urls_to_process)} URLs")

        # Limit if specified
        if args.limit:
            urls_to_process = urls_to_process[:args.limit]
            print(f"After applying limit: {len(urls_to_process)} URLs")

        if not urls_to_process:
            print("No URLs to process after filtering.")
            return

        print(f"Processing {len(urls_to_process)} URLs...")

        # Initialize the RAG pipeline
        pipeline = RAGEmbeddingPipeline()

        # Process all URLs through the pipeline
        success = pipeline.run_full_pipeline(urls_to_process)

        if success:
            print(f"\nSitemap ingestion completed successfully!")
            print(f"Processed {len(urls_to_process)} URLs from {args.sitemap_url}")
        else:
            print(f"\nSitemap ingestion failed.")
            sys.exit(1)

    except Exception as e:
        error_msg = f"Sitemap ingestion failed with error: {str(e)}"
        print(error_msg)
        rag_logger.error(error_msg)
        sys.exit(1)


if __name__ == "__main__":
    main()