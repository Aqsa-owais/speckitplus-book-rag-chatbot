"""
URL collection and web crawling functionality for the Book RAG Content Ingestion Pipeline.

This module handles fetching content from URLs with proper error handling,
rate limiting, and retry logic.
"""
import asyncio
import logging
from typing import List, Tuple, Optional
from urllib.parse import urljoin, urlparse
import aiohttp
from config.settings import Settings
from utils.validators import is_valid_url
from utils.helpers import rate_limit_pause, create_retry_with_backoff


class Crawler:
    """Handles crawling and fetching content from URLs."""

    def __init__(self, settings: Settings):
        """
        Initialize the crawler with configuration settings.

        Args:
            settings: Configuration settings for the crawler
        """
        self.settings = settings
        self.logger = logging.getLogger(__name__)

    async def crawl_urls(self, urls: List[str]) -> List[Tuple[str, str]]:
        """
        Crawl a list of URLs and return their content.

        Args:
            urls: List of URLs to crawl

        Returns:
            List of tuples containing (URL, raw HTML content)
        """
        results = []
        semaphore = asyncio.Semaphore(5)  # Limit concurrent requests
        processed_urls = set()  # Track processed URLs to avoid duplicates

        async def fetch_single_url(url: str) -> Optional[Tuple[str, str]]:
            """Fetch content from a single URL."""
            if not is_valid_url(url):
                self.logger.warning(f"Invalid URL skipped: {url}")
                return None

            # Check if URL was already processed
            normalized_url = url.lower().strip()
            if normalized_url in processed_urls:
                self.logger.info(f"URL already processed, skipping: {url}")
                return None

            async with semaphore:
                content = await self._fetch_with_retry(url)
                if content:
                    processed_urls.add(normalized_url)
                    return (url, content)
                return None

        # Fetch all URLs concurrently
        tasks = [fetch_single_url(url) for url in urls]
        results = await asyncio.gather(*tasks, return_exceptions=True)

        # Filter out None results and exceptions
        valid_results = []
        for i, result in enumerate(results):
            if isinstance(result, Exception):
                self.logger.error(f"Error fetching {urls[i]}: {result}")
            elif result is not None:
                valid_results.append(result)

        return valid_results

    async def _fetch_with_retry(self, url: str) -> Optional[str]:
        """
        Fetch content from a URL with retry logic.

        Args:
            url: URL to fetch content from

        Returns:
            Raw HTML content if successful, None otherwise
        """
        last_error = None

        for attempt in range(self.settings.max_retries + 1):
            try:
                # Rate limiting pause
                if attempt > 0:  # Only pause after first attempt (for retries)
                    delay = create_retry_with_backoff(attempt - 1, self.settings.delay_between_requests)
                    await rate_limit_pause(delay)
                    self.logger.info(f"Retrying {url} (attempt {attempt + 1}) after {delay}s delay")
                else:
                    # Initial request - apply delay between requests
                    await rate_limit_pause(self.settings.delay_between_requests)

                timeout = aiohttp.ClientTimeout(total=30)  # 30 second timeout
                async with aiohttp.ClientSession(timeout=timeout) as session:
                    async with session.get(url) as response:
                        if response.status == 200:
                            content = await response.text()
                            self.logger.info(f"Successfully fetched {url}")
                            return content
                        else:
                            self.logger.warning(f"HTTP {response.status} for {url}")
                            last_error = f"HTTP {response.status}"

            except asyncio.TimeoutError:
                self.logger.warning(f"Timeout for {url} (attempt {attempt + 1})")
                last_error = "Timeout"
            except aiohttp.ClientError as e:
                self.logger.warning(f"Client error for {url}: {e} (attempt {attempt + 1})")
                last_error = str(e)
            except Exception as e:
                self.logger.warning(f"Unexpected error for {url}: {e} (attempt {attempt + 1})")
                last_error = str(e)

        self.logger.error(f"Failed to fetch {url} after {self.settings.max_retries + 1} attempts. Last error: {last_error}")
        return None

    async def get_all_page_urls(self, base_url: str, max_pages: int = 100) -> List[str]:
        """
        Attempt to discover all page URLs from a base URL (for book sites with navigation).
        This is a basic implementation - more sophisticated discovery would require
        site-specific logic or sitemap parsing.

        Args:
            base_url: Base URL to start discovery from
            max_pages: Maximum number of pages to discover

        Returns:
            List of discovered URLs
        """
        self.logger.info(f"Discovering pages from {base_url}")
        discovered_urls = [base_url]

        # For now, return just the base URL
        # In a more sophisticated implementation, this would parse navigation
        # or sitemap to discover additional pages

        return discovered_urls[:max_pages]