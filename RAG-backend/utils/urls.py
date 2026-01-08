"""URL validation utilities for the RAG pipeline."""

import re
from urllib.parse import urlparse
from typing import List, Optional

def is_valid_url(url: str) -> bool:
    """
    Validate if a string is a properly formatted URL.

    Args:
        url: The URL string to validate

    Returns:
        bool: True if the URL is valid, False otherwise
    """
    if not url or not isinstance(url, str):
        return False

    # Basic regex pattern for URL validation
    url_pattern = re.compile(
        r'^https?://'  # http:// or https://
        r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
        r'localhost|'  # localhost...
        r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
        r'(?::\d+)?'  # optional port
        r'(?:/?|[/?]\S+)$', re.IGNORECASE)

    return url_pattern.match(url) is not None

def normalize_url(url: str) -> Optional[str]:
    """
    Normalize a URL by ensuring it has the proper scheme.

    Args:
        url: The URL to normalize

    Returns:
        str: The normalized URL, or None if invalid
    """
    if not url:
        return None

    # Remove leading/trailing whitespace
    url = url.strip()

    # Check if URL is valid as-is
    if is_valid_url(url):
        return url

    # Try to add https:// if no scheme is present
    if not url.startswith(('http://', 'https://')):
        url_with_scheme = f'https://{url}'
        if is_valid_url(url_with_scheme):
            return url_with_scheme

    # If still not valid, return None
    return None

def validate_and_filter_urls(urls: List[str]) -> List[str]:
    """
    Validate a list of URLs and return only the valid ones.

    Args:
        urls: List of URL strings to validate

    Returns:
        List[str]: List of valid URLs
    """
    valid_urls = []
    for url in urls:
        normalized = normalize_url(url)
        if normalized:
            valid_urls.append(normalized)
        else:
            print(f"[WARNING] Skipping invalid URL: {url}")

    return valid_urls

def extract_domain(url: str) -> Optional[str]:
    """
    Extract the domain from a URL.

    Args:
        url: The URL to extract domain from

    Returns:
        str: The domain, or None if invalid URL
    """
    try:
        parsed = urlparse(url)
        return parsed.netloc
    except Exception:
        return None