"""
Unit tests for the embedder module of the Book RAG Content Ingestion Pipeline.
"""
import pytest
from unittest.mock import Mock, patch
from src.embedder import Embedder
from config.settings import Settings


class TestEmbedder:
    """Test cases for the Embedder class."""

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
        os.environ['BATCH_SIZE'] = '5'

        settings = Settings()

        # Restore original environment
        os.environ.clear()
        os.environ.update(original_env)

        return settings

    @pytest.fixture
    def embedder(self, settings):
        """Create an embedder instance."""
        # Mock the Cohere client to avoid actual API calls
        with patch('cohere.Client') as mock_client:
            instance = Embedder(settings)
            instance.cohere_client = Mock()
            return instance

    def test_generate_single_embedding(self, embedder):
        """Test generating a single embedding."""
        test_text = "Test text for embedding"
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3]]
        embedder.cohere_client.embed.return_value = mock_response

        result = embedder.generate_single_embedding(test_text)

        assert result == [0.1, 0.2, 0.3]
        embedder.cohere_client.embed.assert_called_once()

    @pytest.mark.asyncio
    async def test_generate_embeddings_empty_list(self, embedder):
        """Test generating embeddings for an empty list."""
        result = await embedder.generate_embeddings([])

        assert result == []

    @pytest.mark.asyncio
    async def test_generate_embeddings_with_chunks(self, embedder):
        """Test generating embeddings for a list of chunks."""
        chunks = [
            {
                'id': 'chunk_1',
                'content': 'Test content 1',
                'url': 'https://example.com/1',
                'section_title': 'Section 1',
                'chunk_index': 0,
                'metadata': {}
            },
            {
                'id': 'chunk_2',
                'content': 'Test content 2',
                'url': 'https://example.com/2',
                'section_title': 'Section 2',
                'chunk_index': 1,
                'metadata': {}
            }
        ]

        # Mock the embed API response
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
        embedder.cohere_client.embed.return_value = mock_response

        result = await embedder.generate_embeddings(chunks)

        assert len(result) == 2
        assert result[0]['embedding'] == [0.1, 0.2, 0.3]
        assert result[1]['embedding'] == [0.4, 0.5, 0.6]

    def test_validate_api_key_success(self, embedder):
        """Test API key validation - success case."""
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3]]
        embedder.cohere_client.embed.return_value = mock_response

        result = embedder.validate_api_key()

        assert result is True

    def test_validate_api_key_failure(self, embedder):
        """Test API key validation - failure case."""
        embedder.cohere_client.embed.side_effect = Exception("Invalid API key")

        result = embedder.validate_api_key()

        assert result is False

    @pytest.mark.asyncio
    async def test_get_embedding_dimension(self, embedder):
        """Test getting embedding dimension."""
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3, 0.4, 0.5]]
        embedder.cohere_client.embed.return_value = mock_response

        result = await embedder.get_embedding_dimension()

        assert result == 5