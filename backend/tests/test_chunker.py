"""
Unit tests for the chunker module of the Book RAG Content Ingestion Pipeline.
"""
import pytest
from src.chunker import Chunker
from config.settings import Settings


class TestChunker:
    """Test cases for the Chunker class."""

    @pytest.fixture
    def settings(self):
        """Create test settings with small chunk size for testing."""
        # Temporarily modify environment variables for testing
        import os
        original_env = os.environ.copy()

        os.environ['COHERE_API_KEY'] = 'test-key'
        os.environ['QDRANT_URL'] = 'https://test-url'
        os.environ['QDRANT_API_KEY'] = 'test-api-key'
        os.environ['BASE_URLS'] = 'https://example.com'
        os.environ['CHUNK_SIZE'] = '100'
        os.environ['CHUNK_OVERLAP'] = '20'

        settings = Settings()

        # Restore original environment
        os.environ.clear()
        os.environ.update(original_env)

        return settings

    @pytest.fixture
    def chunker(self, settings):
        """Create a chunker instance."""
        return Chunker(settings)

    def test_chunk_text_basic(self, chunker):
        """Test basic text chunking."""
        text = "This is a test sentence. This is another sentence. And a third one."
        url = "https://example.com/test"

        chunks = chunker.chunk_text(text, url)

        assert len(chunks) > 0
        assert all('id' in chunk for chunk in chunks)
        assert all('content' in chunk for chunk in chunks)
        assert all('url' in chunk for chunk in chunks)

    def test_chunk_text_with_section_title(self, chunker):
        """Test text chunking with section title."""
        text = "This is a test sentence. This is another sentence."
        url = "https://example.com/test"
        section_title = "Test Section"

        chunks = chunker.chunk_text(text, url, section_title)

        assert len(chunks) > 0
        assert all(chunk['section_title'] == section_title for chunk in chunks)

    def test_chunk_text_empty(self, chunker):
        """Test text chunking with empty text."""
        chunks = chunker.chunk_text("", "https://example.com/test")

        assert len(chunks) == 0

    def test_split_into_sentences(self, chunker):
        """Test sentence splitting."""
        text = "First sentence. Second sentence! Third sentence? All done."

        sentences = chunker._split_into_sentences(text)

        assert len(sentences) == 4
        assert "First sentence" in sentences[0]
        assert "Second sentence" in sentences[1]
        assert "Third sentence" in sentences[2]
        assert "All done" in sentences[3]

    def test_create_chunk(self, chunker):
        """Test creation of a single chunk."""
        content = "Test content"
        url = "https://example.com/test"
        section_title = "Test Section"
        chunk_index = 0

        chunk = chunker._create_chunk(content, url, section_title, chunk_index)

        assert chunk['id'].startswith('chunk_')
        assert chunk['content'] == content
        assert chunk['url'] == url
        assert chunk['section_title'] == section_title
        assert chunk['chunk_index'] == chunk_index
        assert 'metadata' in chunk
        assert 'token_count' in chunk['metadata']

    def test_chunk_text_by_paragraph(self, chunker):
        """Test paragraph-based chunking."""
        text = "Paragraph 1 with some content.\n\nParagraph 2 with more content.\n\nParagraph 3 ending."
        url = "https://example.com/test"

        chunks = chunker.chunk_text_by_paragraph(text, url)

        assert len(chunks) > 0
        # Should have at least one chunk containing paragraph content
        assert any('Paragraph 1' in chunk['content'] for chunk in chunks)
        assert any('Paragraph 2' in chunk['content'] for chunk in chunks)

    def test_split_large_sentence(self, chunker):
        """Test splitting of a sentence that's too large."""
        # Create a very long sentence that exceeds chunk size
        long_sentence = "word " * 50  # This should exceed the 100-token limit

        parts = chunker._split_large_sentence(long_sentence)

        assert len(parts) > 0
        # Each part should be smaller than the original
        assert all(len(part) < len(long_sentence) for part in parts)