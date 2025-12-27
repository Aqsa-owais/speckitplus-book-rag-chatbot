"""
Configuration management for the Book RAG Content Ingestion Pipeline.

This module handles loading and validating configuration settings from environment variables
and provides a centralized settings object for the application.
"""
import os
from typing import List, Optional
from dataclasses import dataclass
from dotenv import load_dotenv


@dataclass
class Settings:
    """Configuration settings for the ingestion pipeline."""

    # Cohere configuration
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    cohere_model: str = os.getenv("COHERE_MODEL", "embed-english-v3.0")

    # Qdrant configuration
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_collection: str = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")

    # Ingestion configuration
    base_urls: List[str] = None  # Will be set from environment or defaults
    chunk_size: int = int(os.getenv("CHUNK_SIZE", "512"))
    chunk_overlap: int = int(os.getenv("CHUNK_OVERLAP", "50"))
    batch_size: int = int(os.getenv("BATCH_SIZE", "10"))
    max_retries: int = int(os.getenv("MAX_RETRIES", "3"))
    delay_between_requests: float = float(os.getenv("DELAY_BETWEEN_REQUESTS", "1.0"))

    def __post_init__(self):
        """Validate and set default values after initialization."""
        load_dotenv()  # Load environment variables from .env file

        # Reload values after loading .env
        self.cohere_api_key = os.getenv("COHERE_API_KEY", self.cohere_api_key)
        self.cohere_model = os.getenv("COHERE_MODEL", self.cohere_model)
        self.qdrant_url = os.getenv("QDRANT_URL", self.qdrant_url)
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY", self.qdrant_api_key)
        self.qdrant_collection = os.getenv("QDRANT_COLLECTION_NAME", self.qdrant_collection)
        self.chunk_size = int(os.getenv("CHUNK_SIZE", str(self.chunk_size)))
        self.chunk_overlap = int(os.getenv("CHUNK_OVERLAP", str(self.chunk_overlap)))
        self.batch_size = int(os.getenv("BATCH_SIZE", str(self.batch_size)))
        self.max_retries = int(os.getenv("MAX_RETRIES", str(self.max_retries)))
        self.delay_between_requests = float(os.getenv("DELAY_BETWEEN_REQUESTS", str(self.delay_between_requests)))

        # Parse base URLs from environment variable if available
        base_urls_env = os.getenv("BASE_URLS")
        if base_urls_env:
            self.base_urls = [url.strip() for url in base_urls_env.split(",")]
        elif self.base_urls is None:
            self.base_urls = []  # Default to empty list if not provided

        # Validate required settings
        self._validate_settings()

    def _validate_settings(self):
        """Validate that required settings are present."""
        errors = []

        if not self.cohere_api_key:
            errors.append("COHERE_API_KEY is required")

        if not self.qdrant_url:
            errors.append("QDRANT_URL is required")

        if not self.qdrant_api_key:
            errors.append("QDRANT_API_KEY is required")

        if not self.base_urls:
            errors.append("BASE_URLS (at least one URL) is required")

        if self.chunk_size <= 0:
            errors.append("CHUNK_SIZE must be positive")

        if self.chunk_overlap >= self.chunk_size:
            errors.append("CHUNK_OVERLAP must be less than CHUNK_SIZE")

        if self.batch_size <= 0:
            errors.append("BATCH_SIZE must be positive")

        if self.max_retries <= 0:
            errors.append("MAX_RETRIES must be positive")

        if self.delay_between_requests < 0:
            errors.append("DELAY_BETWEEN_REQUESTS must be non-negative")

        if errors:
            raise ValueError(f"Configuration validation failed: {'; '.join(errors)}")