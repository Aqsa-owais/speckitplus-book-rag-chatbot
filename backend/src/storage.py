"""
Qdrant storage and retrieval functionality for the Book RAG Content Ingestion Pipeline.

This module handles storing embeddings in Qdrant Cloud with proper metadata.
"""
import asyncio
import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from config.settings import Settings
from utils.validators import validate_embedding_vector
from utils.helpers import generate_vector_id


class Storage:
    """Handles storage and retrieval of embeddings in Qdrant."""

    def __init__(self, settings: Settings):
        """
        Initialize the storage with configuration settings.

        Args:
            settings: Configuration settings for storage
        """
        self.settings = settings
        self.logger = logging.getLogger(__name__)

        # Initialize Qdrant client
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False  # Using HTTP for compatibility
        )

        # Check if collection exists, create if it doesn't
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """Check if the collection exists, create it if it doesn't."""
        try:
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.settings.qdrant_collection not in collection_names:
                # Need to get embedding dimension first
                # For now, we'll use a common dimension for Cohere embeddings
                # In practice, we might want to determine this dynamically
                embedding_size = 1024  # Default size for Cohere embeddings

                self.client.recreate_collection(
                    collection_name=self.settings.qdrant_collection,
                    vectors_config=VectorParams(size=embedding_size, distance=Distance.COSINE),
                )
                self.logger.info(f"Created collection: {self.settings.qdrant_collection}")
            else:
                self.logger.info(f"Collection exists: {self.settings.qdrant_collection}")

        except Exception as e:
            self.logger.error(f"Error checking/creating collection: {e}")
            raise

    async def store_embeddings(self, embedding_results: List[Dict[str, Any]]) -> bool:
        """
        Store embedding results in Qdrant.

        Args:
            embedding_results: List of chunk dictionaries with embeddings

        Returns:
            True if storage was successful, False otherwise
        """
        if not embedding_results:
            self.logger.warning("No embedding results to store")
            return True

        # Filter out chunks that don't have valid embeddings
        valid_chunks = []
        for chunk in embedding_results:
            if chunk.get('embedding') and validate_embedding_vector(chunk['embedding']):
                valid_chunks.append(chunk)
            else:
                self.logger.warning(f"Skipping chunk {chunk.get('id', 'unknown')} due to invalid embedding")

        if not valid_chunks:
            self.logger.warning("No valid embeddings to store")
            return True

        try:
            # Prepare points for insertion
            points = []
            for chunk in valid_chunks:
                vector_id = generate_vector_id(chunk['url'], chunk['chunk_index'])

                # Prepare payload with metadata
                import datetime
                import hashlib

                # Create content hash for change detection
                content_hash = hashlib.md5(chunk['content'].encode()).hexdigest()

                payload = {
                    "url": chunk['url'],
                    "section_title": chunk.get('section_title', ''),
                    "chunk_id": chunk['id'],
                    "content": chunk['content'][:1000],  # Limit content in payload to avoid size issues
                    "chunk_index": chunk['chunk_index'],
                    "token_count": chunk['metadata'].get('token_count', 0),
                    "word_count": chunk['metadata'].get('word_count', 0),
                    "char_count": chunk['metadata'].get('char_count', 0),
                    "source": "book_ingestion_pipeline",
                    "content_hash": content_hash,
                    "created_at": datetime.datetime.utcnow().isoformat(),
                    "updated_at": datetime.datetime.utcnow().isoformat()
                }

                # Add any additional metadata
                for key, value in chunk.get('metadata', {}).items():
                    if key not in payload:
                        payload[key] = value

                point = models.PointStruct(
                    id=vector_id,
                    vector=chunk['embedding'],
                    payload=payload
                )
                points.append(point)

            # Upload points to Qdrant in batches
            batch_size = 100  # Qdrant recommended batch size
            for i in range(0, len(points), batch_size):
                batch = points[i:i + batch_size]
                self.client.upsert(
                    collection_name=self.settings.qdrant_collection,
                    points=batch
                )
                self.logger.info(f"Uploaded batch of {len(batch)} points to Qdrant")

            self.logger.info(f"Successfully stored {len(valid_chunks)} embeddings in Qdrant")
            return True

        except Exception as e:
            self.logger.error(f"Error storing embeddings in Qdrant: {e}")
            return False

    async def retrieve_similar(self, query_embedding: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve similar embeddings from Qdrant.

        Args:
            query_embedding: Embedding vector to search for similar items
            top_k: Number of similar items to return

        Returns:
            List of similar items with metadata
        """
        try:
            if not validate_embedding_vector(query_embedding):
                raise ValueError("Invalid query embedding")

            results = self.client.search(
                collection_name=self.settings.qdrant_collection,
                query_vector=query_embedding,
                limit=top_k
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_result = {
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload,
                    "content": result.payload.get('content', ''),
                    "url": result.payload.get('url', ''),
                    "section_title": result.payload.get('section_title', '')
                }
                formatted_results.append(formatted_result)

            return formatted_results

        except Exception as e:
            self.logger.error(f"Error retrieving similar embeddings: {e}")
            return []

    async def check_content_exists(self, url: str, content_hash: Optional[str] = None) -> bool:
        """
        Check if content from a URL already exists in the collection.

        Args:
            url: URL to check for existing content
            content_hash: Optional hash of content to check for changes

        Returns:
            True if content already exists, False otherwise
        """
        try:
            # Search for points with the same URL
            results = self.client.scroll(
                collection_name=self.settings.qdrant_collection,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="url",
                            match=models.MatchValue(value=url)
                        )
                    ]
                ),
                limit=10000  # Adjust based on expected number of chunks per URL
            )

            return len(results[0]) > 0 if results and results[0] else False

        except Exception as e:
            self.logger.error(f"Error checking if content exists: {e}")
            return False

    async def get_content_hashes_by_url(self, url: str) -> List[str]:
        """
        Get content hashes for all chunks from a specific URL to detect changes.

        Args:
            url: URL to get content hashes for

        Returns:
            List of content hashes for chunks from the URL
        """
        try:
            results = self.client.scroll(
                collection_name=self.settings.qdrant_collection,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="url",
                            match=models.MatchValue(value=url)
                        )
                    ]
                ),
                limit=10000
            )

            if results and results[0]:
                hashes = []
                for point in results[0]:
                    content = point.payload.get('content', '')
                    import hashlib
                    content_hash = hashlib.md5(content.encode()).hexdigest()
                    hashes.append(content_hash)
                return hashes
            return []

        except Exception as e:
            self.logger.error(f"Error getting content hashes for URL {url}: {e}")
            return []

    async def has_content_changed(self, url: str, new_content_chunks: List[Dict[str, Any]]) -> bool:
        """
        Check if content has changed by comparing hashes.

        Args:
            url: URL to check for changes
            new_content_chunks: New content chunks to compare against

        Returns:
            True if content has changed, False otherwise
        """
        try:
            existing_hashes = await self.get_content_hashes_by_url(url)
            if not existing_hashes:
                return True  # New URL, content has "changed"

            # Create hashes for new content
            new_hashes = []
            for chunk in new_content_chunks:
                import hashlib
                content_hash = hashlib.md5(chunk['content'].encode()).hexdigest()
                new_hashes.append(content_hash)

            # Check if any of the new hashes are different from existing ones
            # This is a simplified check - in a real implementation, you might want
            # to compare chunk by chunk based on other identifiers
            return set(existing_hashes) != set(new_hashes)

        except Exception as e:
            self.logger.error(f"Error checking if content changed for URL {url}: {e}")
            return True  # Assume changed if we can't determine

    async def update_content(self, embedding_results: List[Dict[str, Any]]) -> bool:
        """
        Update existing content in Qdrant (upsert operation).

        Args:
            embedding_results: List of chunk dictionaries with embeddings

        Returns:
            True if update was successful, False otherwise
        """
        # For now, this is the same as store_embeddings since upsert is the default behavior
        return await self.store_embeddings(embedding_results)

    async def delete_content_by_url(self, url: str) -> bool:
        """
        Delete all content associated with a specific URL.

        Args:
            url: URL whose content should be deleted

        Returns:
            True if deletion was successful, False otherwise
        """
        try:
            # Find points with the matching URL
            results = self.client.scroll(
                collection_name=self.settings.qdrant_collection,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="url",
                            match=models.MatchValue(value=url)
                        )
                    ]
                ),
                limit=10000  # Adjust based on expected number of chunks per URL
            )

            if results and results[0]:
                # Extract IDs to delete
                ids_to_delete = [point.id for point in results[0]]

                # Delete points
                self.client.delete(
                    collection_name=self.settings.qdrant_collection,
                    points_selector=models.PointIdsList(
                        points=ids_to_delete
                    )
                )

                self.logger.info(f"Deleted {len(ids_to_delete)} points for URL: {url}")
                return True
            else:
                self.logger.info(f"No content found for URL: {url}")
                return True  # Not an error if nothing to delete

        except Exception as e:
            self.logger.error(f"Error deleting content for URL {url}: {e}")
            return False

    async def get_collection_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the collection.

        Returns:
            Dictionary with collection statistics
        """
        try:
            collection_info = self.client.get_collection(self.settings.qdrant_collection)
            return {
                "vectors_count": collection_info.points_count,
                "collection_name": self.settings.qdrant_collection,
                "config": {
                    "vector_size": collection_info.config.params.size,
                    "distance": collection_info.config.params.distance
                }
            }
        except Exception as e:
            self.logger.error(f"Error getting collection stats: {e}")
            return {}

    async def clear_collection(self) -> bool:
        """
        Clear all points from the collection (use with caution!).

        Returns:
            True if successful, False otherwise
        """
        try:
            # Get all point IDs
            results = self.client.scroll(
                collection_name=self.settings.qdrant_collection,
                limit=10000
            )

            if results and results[0]:
                ids_to_delete = [point.id for point in results[0]]

                # Delete all points
                self.client.delete(
                    collection_name=self.settings.qdrant_collection,
                    points_selector=models.PointIdsList(
                        points=ids_to_delete
                    )
                )

                self.logger.info(f"Cleared collection {self.settings.qdrant_collection}")
                return True
            else:
                self.logger.info(f"Collection {self.settings.qdrant_collection} is already empty")
                return True

        except Exception as e:
            self.logger.error(f"Error clearing collection: {e}")
            return False