"""
Embedding generation functionality for the Book RAG Content Ingestion Pipeline.

This module handles generating embeddings using Cohere API for text chunks.
"""
import asyncio
import logging
from typing import List, Dict, Any, Optional
from config.settings import Settings
from utils.helpers import chunk_list, rate_limit_pause, create_retry_with_backoff
from utils.validators import validate_embedding_vector


class Embedder:
    """Handles embedding generation using Cohere API."""

    def __init__(self, settings: Settings):
        """
        Initialize the embedder with configuration settings.

        Args:
            settings: Configuration settings for embedding
        """
        self.settings = settings
        self.logger = logging.getLogger(__name__)

        # Import Cohere - will be handled in the method to avoid import errors if not available
        try:
            import cohere
            self.cohere_client = cohere.Client(settings.cohere_api_key)
        except ImportError:
            self.logger.error("Cohere library not found. Please install it with 'pip install cohere'.")
            self.cohere_client = None

    async def generate_embeddings(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Generate embeddings for a list of text chunks.

        Args:
            chunks: List of chunk dictionaries containing text content

        Returns:
            List of chunk dictionaries with embeddings added
        """
        if not chunks:
            return []

        if not self.cohere_client:
            raise RuntimeError("Cohere client not initialized. Check if cohere library is installed and API key is valid.")

        # Process chunks in batches to respect API limits
        chunk_batches = chunk_list(chunks, self.settings.batch_size)
        all_results = []

        for batch_idx, batch in enumerate(chunk_batches):
            self.logger.info(f"Processing batch {batch_idx + 1}/{len(chunk_batches)}")

            try:
                batch_results = await self._process_batch_with_retry(batch)
                all_results.extend(batch_results)

                # Rate limiting between batches
                if batch_idx < len(chunk_batches) - 1:  # Don't sleep after the last batch
                    await rate_limit_pause(self.settings.delay_between_requests)

            except Exception as e:
                self.logger.error(f"Error processing batch {batch_idx + 1}: {e}")
                # Add chunks without embeddings to results with error flag
                for chunk in batch:
                    chunk['embedding'] = None
                    chunk['embedding_error'] = str(e)
                    all_results.append(chunk)

        return all_results

    async def _process_batch_with_retry(self, batch: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Process a batch of chunks with retry logic.

        Args:
            batch: List of chunk dictionaries to process

        Returns:
            List of chunk dictionaries with embeddings
        """
        last_error = None

        for attempt in range(self.settings.max_retries + 1):
            try:
                # Extract text content from chunks
                texts = [chunk['content'] for chunk in batch]

                # Generate embeddings using Cohere
                response = self.cohere_client.embed(
                    texts=texts,
                    model=self.settings.cohere_model,
                    input_type="search_document"  # Appropriate for document search
                )

                # Add embeddings to chunks
                results = []
                for i, chunk in enumerate(batch):
                    embedding = response.embeddings[i] if i < len(response.embeddings) else None

                    if embedding and validate_embedding_vector(embedding):
                        chunk['embedding'] = embedding
                        chunk['embedding_model'] = self.settings.cohere_model
                        results.append(chunk)
                    else:
                        self.logger.warning(f"Invalid embedding for chunk {chunk['id']}")
                        chunk['embedding'] = None
                        chunk['embedding_error'] = "Invalid embedding vector"
                        results.append(chunk)

                self.logger.info(f"Successfully processed batch of {len(batch)} chunks")
                return results

            except Exception as e:
                last_error = e
                self.logger.warning(f"Batch processing failed (attempt {attempt + 1}): {e}")

                if attempt < self.settings.max_retries:  # Don't sleep after the last attempt
                    delay = create_retry_with_backoff(attempt, self.settings.delay_between_requests)
                    self.logger.info(f"Retrying batch after {delay}s delay")
                    await rate_limit_pause(delay)

        # If all retries failed, return chunks with error information
        self.logger.error(f"Failed to process batch after {self.settings.max_retries + 1} attempts. Last error: {last_error}")
        results = []
        for chunk in batch:
            chunk['embedding'] = None
            chunk['embedding_error'] = str(last_error)
            results.append(chunk)

        return results

    async def generate_single_embedding(self, text: str) -> Optional[List[float]]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to generate embedding for

        Returns:
            Embedding vector or None if failed
        """
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model=self.settings.cohere_model,
                input_type="search_document"
            )

            if response.embeddings and len(response.embeddings) > 0:
                embedding = response.embeddings[0]
                if validate_embedding_vector(embedding):
                    return embedding
                else:
                    self.logger.warning("Invalid embedding vector returned from API")
                    return None
            else:
                self.logger.warning("No embeddings returned from API")
                return None

        except Exception as e:
            self.logger.error(f"Error generating single embedding: {e}")
            return None

    def validate_api_key(self) -> bool:
        """
        Validate that the Cohere API key is valid.

        Returns:
            True if API key is valid, False otherwise
        """
        try:
            if not self.cohere_client:
                return False

            # Try a simple API call to validate the key
            response = self.cohere_client.embed(
                texts=["test"],
                model=self.settings.cohere_model,
                input_type="search_document"
            )

            return len(response.embeddings) > 0 if response.embeddings else False

        except Exception as e:
            self.logger.error(f"API key validation failed: {e}")
            return False

    async def get_embedding_dimension(self) -> Optional[int]:
        """
        Get the dimension of embeddings for the configured model.

        Returns:
            Embedding dimension or None if unable to determine
        """
        try:
            # Generate a test embedding to determine dimensions
            test_embedding = await self.generate_single_embedding("test")
            if test_embedding:
                return len(test_embedding)
            return None
        except Exception as e:
            self.logger.error(f"Error determining embedding dimension: {e}")
            return None