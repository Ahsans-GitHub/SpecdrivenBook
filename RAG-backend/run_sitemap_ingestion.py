#!/usr/bin/env python3
"""
Script to run the sitemap ingestion for the Physical AI and Humanoid Robotics textbook.

This script will:
1. Fetch all URLs from the sitemap.xml
2. Process them through the RAG embeddings pipeline
3. Store the embeddings in the vector database
"""

import sys
import os

# Add the project root to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sitemap_ingestor import main as sitemap_main

if __name__ == "__main__":
    print("Starting sitemap ingestion for Physical AI and Humanoid Robotics textbook...")
    print("This will fetch all pages from the sitemap and process them through the RAG pipeline.")

    # Call the sitemap ingestion main function
    sitemap_main()