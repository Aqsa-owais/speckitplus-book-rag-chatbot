"""
Input validation utilities for the Book RAG Content Ingestion Pipeline.

This module provides validation functions for various types of input data
used throughout the ingestion pipeline.
"""
import re
from typing import Any, List, Dict, Optional
from urllib.parse import urlparse


def is_valid_url(url: str) -> bool:
    """
    Validate if a string is a properly formatted URL.

    Args:
        url: The URL string to validate

    Returns:
        True if the URL is valid, False otherwise
    """
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def validate_chunk_size(chunk_size: int) -> bool:
    """
    Validate chunk size is within reasonable bounds.

    Args:
        chunk_size: The chunk size to validate

    Returns:
        True if valid, False otherwise
    """
    return 0 < chunk_size <= 4096  # Typical max for embedding models


def validate_chunk_overlap(chunk_overlap: int, chunk_size: int) -> bool:
    """
    Validate chunk overlap is reasonable relative to chunk size.

    Args:
        chunk_overlap: The overlap size to validate
        chunk_size: The chunk size to compare against

    Returns:
        True if valid, False otherwise
    """
    return 0 <= chunk_overlap < chunk_size


def validate_content_not_empty(content: str) -> bool:
    """
    Validate that content is not empty or just whitespace.

    Args:
        content: The content string to validate

    Returns:
        True if valid, False otherwise
    """
    return bool(content and content.strip())


def validate_embedding_vector(embedding: List[float], expected_dimension: Optional[int] = None) -> bool:
    """
    Validate that an embedding vector is properly formatted.

    Args:
        embedding: The embedding vector to validate
        expected_dimension: Expected dimension if known

    Returns:
        True if valid, False otherwise
    """
    if not isinstance(embedding, list):
        return False

    if not all(isinstance(val, (int, float)) for val in embedding):
        return False

    if expected_dimension and len(embedding) != expected_dimension:
        return False

    return True


def validate_metadata(metadata: Dict[str, Any]) -> bool:
    """
    Validate that metadata dictionary has required fields and proper types.

    Args:
        metadata: The metadata dictionary to validate

    Returns:
        True if valid, False otherwise
    """
    if not isinstance(metadata, dict):
        return False

    # Check for required fields (if any)
    # Currently just ensuring it's a valid dict structure
    return True


def validate_batch_size(batch_size: int) -> bool:
    """
    Validate batch size is within reasonable bounds.

    Args:
        batch_size: The batch size to validate

    Returns:
        True if valid, False otherwise
    """
    return 0 < batch_size <= 100  # Reasonable upper limit for API calls


def sanitize_content(content: str) -> str:
    """
    Sanitize content by removing problematic characters or sequences.

    Args:
        content: The content to sanitize

    Returns:
        Sanitized content string
    """
    if not content:
        return content

    # Remove null bytes which can cause issues with some systems
    content = content.replace('\x00', '')

    # Normalize whitespace
    content = re.sub(r'\s+', ' ', content)

    return content.strip()