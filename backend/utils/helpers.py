"""
General helper utilities for the Book RAG Content Ingestion Pipeline.

This module provides utility functions that are used across different components
of the ingestion pipeline.
"""
import time
import asyncio
import hashlib
import uuid
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import urljoin, urlparse


def generate_chunk_id(url: str, chunk_index: int) -> str:
    """
    Generate a unique identifier for a content chunk.

    Args:
        url: The source URL of the content
        chunk_index: The index of this chunk within the content

    Returns:
        A unique identifier for the chunk
    """
    url_hash = hashlib.md5(url.encode()).hexdigest()[:8]
    return f"chunk_{url_hash}_{chunk_index:04d}"


def generate_vector_id(url: str, chunk_index: int) -> str:
    """
    Generate a unique identifier for a vector in Qdrant.

    Args:
        url: The source URL of the content
        chunk_index: The index of this chunk within the content

    Returns:
        A unique identifier for the vector
    """
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f"{url}_{chunk_index}"))


def normalize_url(url: str) -> str:
    """
    Normalize a URL by ensuring it has the proper scheme and removing fragments.

    Args:
        url: The URL to normalize

    Returns:
        The normalized URL
    """
    if not url.startswith(('http://', 'https://')):
        url = 'https://' + url

    parsed = urlparse(url)
    # Reconstruct URL without fragment
    normalized = f"{parsed.scheme}://{parsed.netloc}{parsed.path}"
    if parsed.query:
        normalized += f"?{parsed.query}"

    return normalized


def get_domain_from_url(url: str) -> str:
    """
    Extract the domain from a URL.

    Args:
        url: The URL to extract domain from

    Returns:
        The domain part of the URL
    """
    parsed = urlparse(url)
    return parsed.netloc


def calculate_token_count(text: str, model_name: str = "gpt-3.5-turbo") -> int:
    """
    Calculate an approximate token count for text.
    Note: This is a simplified estimation. For precise counts, use tokenizer libraries.

    Args:
        text: The text to count tokens for
        model_name: The model name to estimate for

    Returns:
        Approximate number of tokens
    """
    # Rough estimation: 1 token ~ 4 characters for English text
    # This is a simplified approach - in production, use proper tokenizers
    if not text:
        return 0

    # More accurate estimation based on common patterns
    # Average is roughly 1 token per 4 characters, but varies by language/model
    return max(1, len(text) // 4)


def create_retry_with_backoff(retry_count: int, base_delay: float = 1.0) -> float:
    """
    Calculate delay with exponential backoff for retries.

    Args:
        retry_count: Current retry attempt number
        base_delay: Base delay in seconds

    Returns:
        Delay in seconds for this retry attempt
    """
    return base_delay * (2 ** retry_count)


async def rate_limit_pause(delay: float):
    """
    Pause execution for a specified delay to implement rate limiting.

    Args:
        delay: Delay in seconds
    """
    if delay > 0:
        await asyncio.sleep(delay)


def merge_metadata(*metadata_dicts: Dict[str, Any]) -> Dict[str, Any]:
    """
    Merge multiple metadata dictionaries into one.

    Args:
        *metadata_dicts: Variable number of metadata dictionaries to merge

    Returns:
        A merged dictionary containing all key-value pairs
    """
    merged = {}
    for md in metadata_dicts:
        if md:
            merged.update(md)
    return merged


def extract_title_from_html(html_content: str) -> Optional[str]:
    """
    Extract title from HTML content if available.

    Args:
        html_content: HTML content to extract title from

    Returns:
        Title if found, otherwise None
    """
    # Simple regex to extract title - in production, use proper HTML parsing
    import re
    title_match = re.search(r'<title[^>]*>(.*?)</title>', html_content, re.IGNORECASE | re.DOTALL)
    if title_match:
        title = title_match.group(1).strip()
        # Remove HTML tags that might be in the title
        import re
        clean_title = re.sub(r'<[^>]+>', '', title)
        return clean_title
    return None


def ensure_absolute_url(base_url: str, relative_url: str) -> str:
    """
    Ensure a relative URL is converted to absolute based on the base URL.

    Args:
        base_url: The base URL
        relative_url: The relative URL to convert

    Returns:
        Absolute URL
    """
    from urllib.parse import urljoin
    return urljoin(base_url, relative_url)


def chunk_list(lst: List[Any], chunk_size: int) -> List[List[Any]]:
    """
    Split a list into chunks of specified size.

    Args:
        lst: List to chunk
        chunk_size: Size of each chunk

    Returns:
        List of lists, each containing up to chunk_size elements
    """
    if chunk_size <= 0:
        raise ValueError("Chunk size must be positive")

    return [lst[i:i + chunk_size] for i in range(0, len(lst), chunk_size)]