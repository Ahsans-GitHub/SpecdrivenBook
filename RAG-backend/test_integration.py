#!/usr/bin/env python3
"""
Integration test for the RAG Embeddings Pipeline.

This script tests the complete functionality of the RAG pipeline
to ensure all components work together correctly.
"""

import sys
import os
import time
from unittest.mock import patch, MagicMock

# Add the parent directory to the path to import from utils
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from main import RAGEmbeddingPipeline
from config import Config


def test_pipeline_initialization():
    """Test that the pipeline initializes correctly."""
    print("Testing pipeline initialization...")

    try:
        # Mock the Cohere and Qdrant clients to avoid actual API calls during testing
        with patch('main.cohere.Client') as mock_cohere, \
             patch('main.QdrantClient') as mock_qdrant:

            # Set up mock return values
            mock_cohere_instance = MagicMock()
            mock_cohere.return_value = mock_cohere_instance

            mock_qdrant_instance = MagicMock()
            mock_qdrant.return_value = mock_qdrant_instance

            # Initialize the pipeline
            pipeline = RAGEmbeddingPipeline()

            print("✓ Pipeline initialized successfully")
            return True

    except Exception as e:
        print(f"✗ Pipeline initialization failed: {str(e)}")
        return False


def test_url_validation():
    """Test URL validation functionality."""
    print("\nTesting URL validation...")

    from utils.urls import is_valid_url, validate_and_filter_urls

    # Test valid URLs
    valid_urls = [
        "https://example.com",
        "http://test.org/page",
        "https://subdomain.domain.com:8080/path?query=value"
    ]

    for url in valid_urls:
        if is_valid_url(url):
            print(f"  ✓ Valid URL correctly identified: {url}")
        else:
            print(f"  ✗ Valid URL incorrectly rejected: {url}")
            return False

    # Test invalid URLs
    invalid_urls = [
        "not-a-url",
        "",
        "htp://invalid-protocol.com",
        "https://"
    ]

    for url in invalid_urls:
        if not is_valid_url(url):
            print(f"  ✓ Invalid URL correctly rejected: '{url}'")
        else:
            print(f"  ✗ Invalid URL incorrectly accepted: '{url}'")
            return False

    # Test URL filtering
    mixed_urls = ["https://valid.com", "invalid-url", "https://also-valid.org"]
    filtered = validate_and_filter_urls(mixed_urls)

    if len(filtered) == 2 and "invalid-url" not in filtered:
        print("  ✓ URL filtering works correctly")
        return True
    else:
        print("  ✗ URL filtering failed")
        return False


def test_text_processing():
    """Test text processing functionality."""
    print("\nTesting text processing...")

    from utils.text_processing import clean_html_content, semantic_chunk_text, sanitize_text

    # Test HTML cleaning
    html_content = "<html><head><title>Test</title></head><body><p>This is a <b>test</b> paragraph.</p></body></html>"
    cleaned = clean_html_content(html_content)

    if "This is a test paragraph." in cleaned:
        print("  ✓ HTML cleaning works correctly")
    else:
        print("  ✗ HTML cleaning failed")
        return False

    # Test text sanitization
    dirty_text = "This is a <script>alert('xss')</script> test with \x00\x01 control chars"
    sanitized = sanitize_text(dirty_text)

    if "<script>" not in sanitized and "\x00" not in sanitized:
        print("  ✓ Text sanitization works correctly")
    else:
        print("  ✗ Text sanitization failed")
        return False

    # Test semantic chunking
    long_text = "This is a test sentence. " * 100  # Create a longer text
    chunks = semantic_chunk_text(long_text, chunk_size=100, overlap=20)

    if len(chunks) > 0 and all(len(chunk) <= 100 for chunk in chunks):
        print("  ✓ Text chunking works correctly")
        return True
    else:
        print("  ✗ Text chunking failed")
        return False


def test_configuration_validation():
    """Test configuration validation."""
    print("\nTesting configuration validation...")

    try:
        # This should work if the config is properly set up
        Config.validate_config()
        print("  ✓ Configuration validation passed")
        return True
    except ValueError as e:
        print(f"  ✗ Configuration validation failed: {str(e)}")
        return False


def test_end_to_end_pipeline():
    """Test the end-to-end pipeline with mock data."""
    print("\nTesting end-to-end pipeline (with mocks)...")

    try:
        with patch('main.cohere.Client') as mock_cohere, \
             patch('main.QdrantClient') as mock_qdrant, \
             patch('main.requests.Session') as mock_session:

            # Set up mocks
            mock_cohere_instance = MagicMock()
            mock_cohere_instance.embed.return_value = MagicMock()
            mock_cohere_instance.embed.return_value.embeddings = [[0.1, 0.2, 0.3]] * 5  # Mock embeddings
            mock_cohere.return_value = mock_cohere_instance

            mock_qdrant_instance = MagicMock()
            mock_qdrant_instance.get_collections.return_value = MagicMock()
            mock_qdrant_instance.get_collections.return_value.collections = []
            mock_qdrant_instance.create_collection.return_value = None
            mock_qdrant_instance.upsert.return_value = None
            mock_qdrant.return_value = mock_qdrant_instance

            mock_session_instance = MagicMock()
            mock_session_instance.get.return_value.text = "<html><body><p>Test content for pipeline</p></body></html>"
            mock_session_instance.get.return_value.raise_for_status.return_value = None
            mock_session.return_value = mock_session_instance

            # Initialize pipeline
            pipeline = RAGEmbeddingPipeline()

            # Test with a mock URL
            test_urls = ["https://httpbin.org/html"]  # Using a reliable test endpoint

            # Run the pipeline (this will use mocks instead of real services)
            success = pipeline.run_full_pipeline(test_urls)

            if success:
                print("  ✓ End-to-end pipeline test completed successfully")
                return True
            else:
                print("  ✗ End-to-end pipeline test failed")
                return False

    except Exception as e:
        print(f"  ✗ End-to-end pipeline test failed with error: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all integration tests."""
    print("Starting RAG Embeddings Pipeline Integration Tests\n")

    tests = [
        test_pipeline_initialization,
        test_url_validation,
        test_text_processing,
        test_configuration_validation,
        test_end_to_end_pipeline
    ]

    results = []
    for test in tests:
        results.append(test())

    print(f"\n{'='*50}")
    print("Integration Test Results:")
    print(f"Passed: {sum(results)}/{len(results)}")

    if all(results):
        print("🎉 All integration tests passed!")
        return True
    else:
        print("❌ Some integration tests failed.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)