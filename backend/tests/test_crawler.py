"""
Unit tests for the crawler module of the Book RAG Content Ingestion Pipeline.
"""
import pytest
import asyncio
from unittest.mock import AsyncMock, patch
from src.crawler import Crawler
from config.settings import Settings


class TestCrawler:
    """Test cases for the Crawler class."""

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

        settings = Settings()
        settings.max_retries = 1
        settings.delay_between_requests = 0.0  # No delay for tests

        # Restore original environment
        os.environ.clear()
        os.environ.update(original_env)

        return settings

    @pytest.fixture
    def crawler(self, settings):
        """Create a crawler instance."""
        return Crawler(settings)

    @pytest.mark.asyncio
    async def test_crawl_urls_success(self, crawler):
        """Test successful crawling of URLs."""
        urls = ['https://example.com']

        with patch('aiohttp.ClientSession.get') as mock_get:
            mock_response = AsyncMock()
            mock_response.status = 200
            mock_response.text.return_value = '<html><body>Test content</body></html>'
            mock_get.return_value.__aenter__.return_value = mock_response

            results = await crawler.crawl_urls(urls)

            assert len(results) == 1
            assert results[0][0] == 'https://example.com'
            assert 'Test content' in results[0][1]

    @pytest.mark.asyncio
    async def test_crawl_urls_with_invalid_url(self, crawler, caplog):
        """Test crawling with an invalid URL."""
        urls = ['invalid-url']

        results = await crawler.crawl_urls(urls)

        assert len(results) == 0
        assert 'Invalid URL skipped' in caplog.text

    @pytest.mark.asyncio
    async def test_fetch_with_retry_success(self, crawler):
        """Test fetching with retry logic - success case."""
        with patch('aiohttp.ClientSession.get') as mock_get:
            mock_response = AsyncMock()
            mock_response.status = 200
            mock_response.text.return_value = 'Success content'
            mock_get.return_value.__aenter__.return_value = mock_response

            result = await crawler._fetch_with_retry('https://example.com')

            assert result == 'Success content'

    @pytest.mark.asyncio
    async def test_fetch_with_retry_failure(self, crawler):
        """Test fetching with retry logic - failure case."""
        with patch('aiohttp.ClientSession.get') as mock_get:
            mock_response = AsyncMock()
            mock_response.status = 404
            mock_get.return_value.__aenter__.return_value = mock_response

            result = await crawler._fetch_with_retry('https://example.com')

            assert result is None

    def test_get_all_page_urls(self, crawler):
        """Test getting all page URLs from a base URL."""
        urls = asyncio.run(crawler.get_all_page_urls('https://example.com'))

        assert len(urls) == 1
        assert urls[0] == 'https://example.com'