"""
Text extraction and preprocessing functionality for the Book RAG Content Ingestion Pipeline.

This module handles extracting clean text from HTML content, removing formatting,
and preparing content for chunking and embedding.
"""
import logging
import re
from typing import List, Optional
from bs4 import BeautifulSoup
from utils.validators import validate_content_not_empty
from utils.validators import sanitize_content


class Extractor:
    """Handles text extraction and preprocessing from HTML content."""

    def __init__(self):
        """Initialize the extractor."""
        self.logger = logging.getLogger(__name__)

    def extract_text(self, html_content: str) -> str:
        """
        Extract clean text from HTML content.

        Args:
            html_content: Raw HTML content to extract text from

        Returns:
            Clean text with HTML formatting removed
        """
        if not html_content:
            return ""

        try:
            # Parse HTML with BeautifulSoup
            soup = BeautifulSoup(html_content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
                script.decompose()

            # Get text content
            text = soup.get_text()

            # Clean up the text
            clean_text = self._clean_extracted_text(text)

            # Validate and return
            if validate_content_not_empty(clean_text):
                return clean_text
            else:
                self.logger.warning("Extracted content is empty after cleaning")
                return ""

        except Exception as e:
            self.logger.error(f"Error extracting text: {e}")
            return ""

    def extract_text_with_structure(self, html_content: str) -> dict:
        """
        Extract text while preserving some structural information.

        Args:
            html_content: Raw HTML content to extract text from

        Returns:
            Dictionary containing text and structural information
        """
        if not html_content:
            return {"content": "", "title": "", "headings": []}

        try:
            soup = BeautifulSoup(html_content, 'html.parser')

            # Extract title
            title_tag = soup.find('title')
            title = title_tag.get_text().strip() if title_tag else ""

            # Extract headings
            headings = []
            for i in range(1, 7):  # h1 to h6
                for heading in soup.find_all(f'h{i}'):
                    headings.append({
                        'level': i,
                        'text': heading.get_text().strip()
                    })

            # Remove script and style elements
            for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
                script.decompose()

            # Get text content
            text = soup.get_text()

            # Clean up the text
            clean_text = self._clean_extracted_text(text)

            return {
                "content": clean_text,
                "title": title,
                "headings": headings
            }

        except Exception as e:
            self.logger.error(f"Error extracting text with structure: {e}")
            return {"content": "", "title": "", "headings": []}

    def _clean_extracted_text(self, text: str) -> str:
        """
        Clean up extracted text by removing extra whitespace and normalizing.

        Args:
            text: Raw extracted text

        Returns:
            Cleaned text
        """
        if not text:
            return text

        # Replace multiple whitespace with single space
        text = re.sub(r'\s+', ' ', text)

        # Remove leading/trailing whitespace
        text = text.strip()

        # Sanitize content (remove problematic characters)
        text = sanitize_content(text)

        return text

    def extract_content_by_selector(self, html_content: str, selectors: List[str]) -> str:
        """
        Extract content using specific CSS selectors.

        Args:
            html_content: Raw HTML content
            selectors: List of CSS selectors to use for extraction

        Returns:
            Extracted content based on selectors
        """
        if not html_content or not selectors:
            return ""

        try:
            soup = BeautifulSoup(html_content, 'html.parser')
            content_parts = []

            for selector in selectors:
                elements = soup.select(selector)
                for element in elements:
                    content_parts.append(element.get_text())

            combined_content = ' '.join(content_parts)
            clean_content = self._clean_extracted_text(combined_content)

            return clean_content

        except Exception as e:
            self.logger.error(f"Error extracting content by selector: {e}")
            return ""

    def extract_main_content(self, html_content: str) -> str:
        """
        Attempt to extract main content by looking for common content containers.

        Args:
            html_content: Raw HTML content

        Returns:
            Extracted main content
        """
        if not html_content:
            return ""

        # Common selectors for main content areas
        content_selectors = [
            'main',
            '[role="main"]',
            '.main-content',
            '.content',
            '.post-content',
            '.article-content',
            '.markdown-body',  # Common for documentation sites
            '.doc-content',
            '.docs-content',
            '.container',
            '.wrapper',
            'article',
            '.post',
            '.entry-content'
        ]

        # First, try to extract using specific selectors
        for selector in content_selectors:
            content = self.extract_content_by_selector(html_content, [selector])
            if content and len(content) > 100:  # If we get substantial content
                return content

        # Fallback to general extraction
        return self.extract_text(html_content)