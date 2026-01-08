"""Text processing utilities for the RAG pipeline."""

import re
from typing import List, Tuple
from bs4 import BeautifulSoup

def clean_html_content(html_content: str) -> str:
    """
    Clean HTML content and extract plain text.

    Args:
        html_content: Raw HTML content

    Returns:
        str: Cleaned plain text content
    """
    if not html_content:
        return ""

    try:
        # Parse HTML content
        soup = BeautifulSoup(html_content, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        # Get text content
        text = soup.get_text()

        # Clean up the text
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = ' '.join(chunk for chunk in chunks if chunk)

        return text
    except Exception as e:
        raise ValueError(f"Failed to clean HTML content: {str(e)}")

def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> List[str]:
    """
    Split text into overlapping chunks of specified size.

    Args:
        text: The text to chunk
        chunk_size: Maximum size of each chunk
        overlap: Number of characters to overlap between chunks

    Returns:
        List[str]: List of text chunks
    """
    if not text:
        return []

    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")

    if overlap < 0:
        raise ValueError("overlap must be non-negative")

    if overlap >= chunk_size:
        raise ValueError("overlap must be less than chunk_size")

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]
        chunks.append(chunk)

        # Move start forward by chunk_size - overlap
        start = end - overlap

        # If we're near the end, break to avoid empty chunks
        if start >= len(text):
            break

    return chunks

def semantic_chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> List[str]:
    """
    Split text into semantic chunks based on document structure (headings, paragraphs).

    Args:
        text: The text to chunk
        chunk_size: Maximum size of each chunk
        overlap: Number of characters to overlap between chunks

    Returns:
        List[str]: List of semantically coherent text chunks
    """
    if not text:
        return []

    # Split text into paragraphs
    paragraphs = text.split('\n\n')

    # Process each paragraph to ensure it's not too large
    processed_paragraphs = []
    for paragraph in paragraphs:
        if len(paragraph) <= chunk_size:
            processed_paragraphs.append(paragraph)
        else:
            # If a paragraph is too large, fall back to basic chunking
            sub_chunks = chunk_text(paragraph, chunk_size, overlap)
            processed_paragraphs.extend(sub_chunks)

    # Now combine paragraphs into larger chunks while respecting the size limit
    chunks = []
    current_chunk = ""

    for paragraph in processed_paragraphs:
        # If adding this paragraph would exceed the chunk size
        if len(current_chunk) + len(paragraph) > chunk_size:
            if current_chunk:  # If we have content in the current chunk
                chunks.append(current_chunk.strip())

            # If the paragraph itself is smaller than chunk_size, add it to new chunk
            if len(paragraph) <= chunk_size:
                current_chunk = paragraph
            else:
                # If it's still too big, chunk it further
                sub_chunks = chunk_text(paragraph, chunk_size, overlap)
                if sub_chunks:
                    current_chunk = sub_chunks[0]
                    chunks.extend(sub_chunks[1:])
        else:
            # Add paragraph to current chunk
            if current_chunk:
                current_chunk += "\n\n" + paragraph
            else:
                current_chunk = paragraph

    # Add the last chunk if it exists
    if current_chunk:
        chunks.append(current_chunk.strip())

    # Final validation: ensure no chunk exceeds the size limit
    final_chunks = []
    for chunk in chunks:
        if len(chunk) > chunk_size:
            # If any chunk is still too large, break it down further
            sub_chunks = chunk_text(chunk, chunk_size, overlap)
            final_chunks.extend(sub_chunks)
        else:
            final_chunks.append(chunk)

    return final_chunks

def sanitize_text(text: str) -> str:
    """
    Sanitize text to prevent potential security issues.

    Args:
        text: The text to sanitize

    Returns:
        str: Sanitized text
    """
    if not text:
        return ""

    # Remove potentially dangerous patterns
    # Remove control characters except common whitespace
    sanitized = re.sub(r'[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]', ' ', text)

    # Limit consecutive whitespace to single space
    sanitized = re.sub(r'\s+', ' ', sanitized)

    # Remove potential script tags (extra safety)
    sanitized = re.sub(r'<script[^>]*>.*?</script>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)

    return sanitized.strip()

def extract_title_from_html(html_content: str) -> str:
    """
    Extract the title from HTML content.

    Args:
        html_content: Raw HTML content

    Returns:
        str: The extracted title, or empty string if not found
    """
    try:
        soup = BeautifulSoup(html_content, 'html.parser')
        title_tag = soup.find('title')
        if title_tag:
            return title_tag.get_text().strip()
        return ""
    except Exception:
        return ""

def extract_headers_from_html(html_content: str) -> List[Tuple[str, str]]:
    """
    Extract headers (h1, h2, h3, etc.) from HTML content.

    Args:
        html_content: Raw HTML content

    Returns:
        List[Tuple[str, str]]: List of (header_type, header_text) tuples
    """
    headers = []
    try:
        soup = BeautifulSoup(html_content, 'html.parser')
        for i in range(1, 7):  # h1 through h6
            header_tags = soup.find_all(f'h{i}')
            for tag in header_tags:
                headers.append((f'h{i}', tag.get_text().strip()))
        return headers
    except Exception:
        return []