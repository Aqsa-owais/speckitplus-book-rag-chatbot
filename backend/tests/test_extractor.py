"""
Unit tests for the extractor module of the Book RAG Content Ingestion Pipeline.
"""
import pytest
from src.extractor import Extractor


class TestExtractor:
    """Test cases for the Extractor class."""

    @pytest.fixture
    def extractor(self):
        """Create an extractor instance."""
        return Extractor()

    def test_extract_text_basic(self, extractor):
        """Test basic text extraction from HTML."""
        html = "<html><body><p>Hello, world!</p></body></html>"
        result = extractor.extract_text(html)

        assert "Hello, world!" in result
        assert "<p>" not in result

    def test_extract_text_with_script_and_style(self, extractor):
        """Test text extraction with script and style tags."""
        html = """
        <html>
            <head>
                <script>console.log('test');</script>
                <style>body { color: red; }</style>
            </head>
            <body>
                <p>Content without scripts or styles</p>
            </body>
        </html>
        """
        result = extractor.extract_text(html)

        assert "Content without scripts or styles" in result
        assert "console.log" not in result
        assert "color: red" not in result

    def test_extract_text_empty(self, extractor):
        """Test text extraction with empty HTML."""
        result = extractor.extract_text("")

        assert result == ""

    def test_extract_text_with_structure(self, extractor):
        """Test text extraction with structural information."""
        html = """
        <html>
            <head><title>Test Title</title></head>
            <body>
                <h1>Main Heading</h1>
                <p>Content here</p>
                <h2>Sub Heading</h2>
                <p>More content</p>
            </body>
        </html>
        """
        result = extractor.extract_text_with_structure(html)

        assert result["title"] == "Test Title"
        assert len(result["headings"]) == 2
        assert result["headings"][0]["text"] == "Main Heading"
        assert result["headings"][0]["level"] == 1
        assert result["headings"][1]["text"] == "Sub Heading"
        assert result["headings"][1]["level"] == 2
        assert "Content here" in result["content"]

    def test_extract_main_content(self, extractor):
        """Test extraction of main content."""
        html = """
        <html>
            <body>
                <div class="sidebar">Sidebar content</div>
                <main class="main-content">
                    <p>Main content that should be extracted</p>
                </main>
                <div class="footer">Footer content</div>
            </body>
        </html>
        """
        result = extractor.extract_main_content(html)

        assert "Main content that should be extracted" in result
        assert "Sidebar content" not in result
        assert "Footer content" not in result

    def test_clean_extracted_text(self, extractor):
        """Test cleaning of extracted text."""
        raw_text = "  Multiple   spaces\tand\n\nnewlines  "
        cleaned = extractor._clean_extracted_text(raw_text)

        assert cleaned == "Multiple spaces and newlines"

    def test_extract_content_by_selector(self, extractor):
        """Test content extraction using CSS selectors."""
        html = """
        <html>
            <body>
                <div class="content">Content 1</div>
                <div class="other">Other content</div>
                <div class="content">Content 2</div>
            </body>
        </html>
        """
        result = extractor.extract_content_by_selector(html, ['.content'])

        assert "Content 1" in result
        assert "Content 2" in result
        assert "Other content" not in result