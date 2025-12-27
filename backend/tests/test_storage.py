"""
Unit tests for the storage module of the Book RAG Content Ingestion Pipeline.
"""
import pytest
from unittest.mock import Mock, patch
from src.storage import Storage
from config.settings import Settings


class TestStorage:
    """Test cases for the Storage class."""

    @pytest.fixture
    def settings(self):
        """Create test settings."""
        # Temporarily modify environment variables for testing
        import os
        original_env = os.environ.copy()

        os.environ['COHERE_API_KEY'] = 'test-key'
        os.environ['QDRANT_URL'] = 'https://test-url'
        os.environ['QDRANT_API_KEY'] = 'test-api-key'
        os.environ['BASE_URLS'] = 'https://example.com'
        os.environ['QDRANT_COLLECTION_NAME'] = 'test_collection'

        settings = Settings()

        # Restore original environment
        os.environ.clear()
        os.environ.update(original_env)

        return settings

    @pytest.fixture
    def storage(self, settings):
        """Create a storage instance."""
        # Mock the Qdrant client to avoid actual API calls
        with patch('qdrant_client.QdrantClient') as mock_client:
            instance = Storage(settings)
            instance.client = Mock()
            return instance

    @pytest.mark.asyncio
    async def test_store_embeddings_empty_list(self, storage):
        """Test storing an empty list of embeddings."""
        result = await storage.store_embeddings([])

        assert result is True

    @pytest.mark.asyncio
    async def test_store_embeddings_with_valid_chunks(self, storage):
        """Test storing embeddings with valid chunks."""
        embedding_results = [
            {
                'id': 'chunk_1',
                'content': 'Test content 1',
                'url': 'https://example.com/1',
                'section_title': 'Section 1',
                'chunk_index': 0,
                'embedding': [0.1, 0.2, 0.3],
                'metadata': {'token_count': 10, 'word_count': 5, 'char_count': 50}
            }
        ]

        result = await storage.store_embeddings(embedding_results)

        assert result is True
        # Verify that upsert was called
        storage.client.upsert.assert_called()

    @pytest.mark.asyncio
    async def test_store_embeddings_with_invalid_chunks(self, storage):
        """Test storing embeddings with invalid chunks (no embedding)."""
        embedding_results = [
            {
                'id': 'chunk_1',
                'content': 'Test content 1',
                'url': 'https://example.com/1',
                'section_title': 'Section 1',
                'chunk_index': 0,
                'metadata': {'token_count': 10, 'word_count': 5, 'char_count': 50}
                # No 'embedding' key
            }
        ]

        result = await storage.store_embeddings(embedding_results)

        assert result is True  # Should still return True, just skip invalid chunks

    @pytest.mark.asyncio
    async def test_retrieve_similar(self, storage):
        """Test retrieving similar embeddings."""
        query_embedding = [0.1, 0.2, 0.3]

        # Mock search result
        mock_result = Mock()
        mock_result.id = 'vector_1'
        mock_result.score = 0.9
        mock_result.payload = {
            'url': 'https://example.com/1',
            'content': 'Test content',
            'section_title': 'Section 1'
        }

        storage.client.search.return_value = [mock_result]

        results = await storage.retrieve_similar(query_embedding, top_k=1)

        assert len(results) == 1
        assert results[0]['id'] == 'vector_1'
        assert results[0]['score'] == 0.9
        assert results[0]['url'] == 'https://example.com/1'

    @pytest.mark.asyncio
    async def test_check_content_exists(self, storage):
        """Test checking if content exists."""
        # Mock scroll result
        mock_point = Mock()
        mock_point.id = 'vector_1'
        storage.client.scroll.return_value = ([mock_point], None)

        result = await storage.check_content_exists('https://example.com/test')

        assert result is True
        storage.client.scroll.assert_called()

    @pytest.mark.asyncio
    async def test_delete_content_by_url(self, storage):
        """Test deleting content by URL."""
        # Mock scroll result to return a point
        mock_point = Mock()
        mock_point.id = 'vector_1'
        storage.client.scroll.return_value = ([mock_point], None)

        result = await storage.delete_content_by_url('https://example.com/test')

        assert result is True
        storage.client.delete.assert_called()

    @pytest.mark.asyncio
    async def test_get_collection_stats(self, storage):
        """Test getting collection stats."""
        # Mock collection info
        mock_collection_info = Mock()
        mock_collection_info.points_count = 100
        mock_collection_info.config.params.size = 1536
        mock_collection_info.config.params.distance = 'cosine'

        storage.client.get_collection.return_value = mock_collection_info

        stats = await storage.get_collection_stats()

        assert stats['vectors_count'] == 100
        assert stats['config']['vector_size'] == 1536

    def test_ensure_collection_exists(self, settings):
        """Test ensuring collection exists (integration test with mocked client)."""
        with patch('qdrant_client.QdrantClient') as mock_client:
            mock_instance = Mock()
            mock_client.return_value = mock_instance

            # Mock get_collections to return an empty list (collection doesn't exist)
            mock_collections = Mock()
            mock_collections.collections = []
            mock_instance.get_collections.return_value = mock_collections

            # This should trigger recreate_collection
            storage = Storage(settings)

            # Verify recreate_collection was called
            mock_instance.recreate_collection.assert_called_once()