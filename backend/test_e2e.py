"""
End-to-end test for the Book RAG Content Ingestion Pipeline.

This script tests the complete pipeline flow with sample data.
"""
import asyncio
import os
from unittest.mock import patch, Mock
from main import main
from config.settings import Settings


async def test_e2e_pipeline():
    """Test the complete pipeline with mocked external services."""
    # Temporarily set environment variables for testing
    original_env = os.environ.copy()

    try:
        os.environ['COHERE_API_KEY'] = 'test-key'
        os.environ['QDRANT_URL'] = 'https://test-url'
        os.environ['QDRANT_API_KEY'] = 'test-api-key'
        os.environ['BASE_URLS'] = 'https://example.com/test'
        os.environ['CHUNK_SIZE'] = '100'
        os.environ['CHUNK_OVERLAP'] = '20'

        # Mock external dependencies
        with patch('aiohttp.ClientSession.get') as mock_get, \
             patch('cohere.Client') as mock_cohere, \
             patch('qdrant_client.QdrantClient') as mock_qdrant:

            # Mock HTTP response
            mock_response = Mock()
            mock_response.status = 200
            mock_response.text.return_value = '<html><body><p>Test content for the book.</p></body></html>'
            mock_get.return_value.__aenter__.return_value = mock_response

            # Mock Cohere client
            mock_cohere_instance = Mock()
            mock_cohere.return_value = mock_cohere_instance
            mock_embed_response = Mock()
            mock_embed_response.embeddings = [[0.1, 0.2, 0.3]]
            mock_cohere_instance.embed.return_value = mock_embed_response

            # Mock Qdrant client
            mock_qdrant_instance = Mock()
            mock_qdrant.return_value = mock_qdrant_instance
            mock_collections = Mock()
            mock_collections.collections = []
            mock_qdrant_instance.get_collections.return_value = mock_collections

            # Run the pipeline
            await main()

            print("✅ End-to-end test completed successfully!")
            return True

    except Exception as e:
        print(f"❌ End-to-end test failed: {e}")
        return False
    finally:
        # Restore original environment
        os.environ.clear()
        os.environ.update(original_env)


if __name__ == "__main__":
    success = asyncio.run(test_e2e_pipeline())
    if success:
        print("\n🎉 Pipeline test successful!")
    else:
        print("\n💥 Pipeline test failed!")
        exit(1)