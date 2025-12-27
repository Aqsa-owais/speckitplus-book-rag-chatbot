"""
Token-aware text chunking functionality for the Book RAG Content Ingestion Pipeline.

This module handles splitting text into chunks with overlap while preserving
semantic boundaries and respecting token limits.
"""
import logging
from typing import List, Dict, Any
from tokenizers import Tokenizer
from tokenizers.models import BPE
from tokenizers.trainers import BpeTrainer
from tokenizers.pre_tokenizers import Whitespace
from utils.validators import validate_chunk_size, validate_chunk_overlap
from utils.helpers import calculate_token_count, chunk_list
from config.settings import Settings


class Chunker:
    """Handles token-aware text chunking with overlap."""

    def __init__(self, settings: Settings):
        """
        Initialize the chunker with configuration settings.

        Args:
            settings: Configuration settings for chunking
        """
        self.settings = settings
        self.logger = logging.getLogger(__name__)

        # Validate settings
        if not validate_chunk_size(self.settings.chunk_size):
            raise ValueError(f"Invalid chunk size: {self.settings.chunk_size}")

        if not validate_chunk_overlap(self.settings.chunk_overlap, self.settings.chunk_size):
            raise ValueError(f"Invalid chunk overlap: {self.settings.chunk_overlap} for chunk size: {self.settings.chunk_size}")

        # Initialize tokenizer (using a simple approach for now)
        # In production, you might want to use a pre-trained tokenizer
        self.tokenizer = self._initialize_tokenizer()

    def _initialize_tokenizer(self):
        """
        Initialize a tokenizer for token counting.
        For now, using a simple approach, but this could be expanded.
        """
        # For now, we'll use a simple approach based on the calculate_token_count helper
        # In a more sophisticated implementation, we would load a proper tokenizer
        return None

    def chunk_text(self, text: str, url: str, section_title: str = "") -> List[Dict[str, Any]]:
        """
        Chunk text into segments with overlap.

        Args:
            text: Text to chunk
            url: Source URL for the text
            section_title: Title of the section (optional)

        Returns:
            List of chunk dictionaries with content, metadata, and IDs
        """
        if not text:
            return []

        # Split text into sentences to maintain semantic boundaries
        sentences = self._split_into_sentences(text)

        # Group sentences into chunks that respect token limits
        chunks = self._create_chunks_from_sentences(sentences, url, section_title)

        return chunks

    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences while preserving sentence boundaries.

        Args:
            text: Text to split into sentences

        Returns:
            List of sentences
        """
        import re

        # Basic sentence splitting using common sentence endings
        # This is a simplified approach - for production, use more sophisticated NLP libraries
        sentence_endings = r'[.!?]+'
        sentences = re.split(sentence_endings, text)

        # Clean up sentences
        cleaned_sentences = []
        for sentence in sentences:
            sentence = sentence.strip()
            if sentence:
                cleaned_sentences.append(sentence)

        return cleaned_sentences

    def _create_chunks_from_sentences(self, sentences: List[str], url: str, section_title: str) -> List[Dict[str, Any]]:
        """
        Create chunks from sentences respecting token limits.

        Args:
            sentences: List of sentences to chunk
            url: Source URL for the text
            section_title: Title of the section

        Returns:
            List of chunk dictionaries
        """
        chunks = []
        current_chunk = []
        current_chunk_token_count = 0
        chunk_index = 0

        i = 0
        while i < len(sentences):
            sentence = sentences[i]
            sentence_token_count = calculate_token_count(sentence)

            # Check if adding this sentence would exceed the chunk size
            if current_chunk_token_count + sentence_token_count <= self.settings.chunk_size:
                # Add sentence to current chunk
                current_chunk.append(sentence)
                current_chunk_token_count += sentence_token_count
                i += 1
            else:
                # If current chunk is empty but sentence is too large, split the sentence
                if len(current_chunk) == 0:
                    # Split the sentence into smaller parts
                    sentence_chunks = self._split_large_sentence(sentence)
                    for part in sentence_chunks:
                        part_token_count = calculate_token_count(part)
                        if part_token_count <= self.settings.chunk_size:
                            chunk_content = " ".join([part])
                            chunk = self._create_chunk(chunk_content, url, section_title, chunk_index)
                            chunks.append(chunk)
                            chunk_index += 1
                    i += 1
                else:
                    # Finalize current chunk
                    chunk_content = " ".join(current_chunk)
                    chunk = self._create_chunk(chunk_content, url, section_title, chunk_index)
                    chunks.append(chunk)
                    chunk_index += 1

                    # Start new chunk with overlap if possible
                    current_chunk, current_chunk_token_count = self._apply_overlap(
                        current_chunk, sentences, i, sentence_token_count
                    )

        # Add the last chunk if it has content
        if current_chunk:
            chunk_content = " ".join(current_chunk)
            chunk = self._create_chunk(chunk_content, url, section_title, chunk_index)
            chunks.append(chunk)

        return chunks

    def _split_large_sentence(self, sentence: str) -> List[str]:
        """
        Split a large sentence that exceeds the chunk size into smaller parts.

        Args:
            sentence: Large sentence to split

        Returns:
            List of smaller sentence parts
        """
        if calculate_token_count(sentence) <= self.settings.chunk_size:
            return [sentence]

        # Split by words to create smaller chunks
        words = sentence.split()
        if len(words) <= 1:
            # If it's a single "word" (like a very long URL), split by characters
            return self._split_by_tokens(sentence)

        parts = []
        current_part = []
        current_part_token_count = 0

        for word in words:
            word_token_count = calculate_token_count(word)

            if current_part_token_count + word_token_count <= self.settings.chunk_size:
                current_part.append(word)
                current_part_token_count += word_token_count
            else:
                if current_part:
                    parts.append(" ".join(current_part))
                    current_part = [word]
                    current_part_token_count = word_token_count
                else:
                    # Word is too large by itself, need to split it
                    sub_parts = self._split_by_tokens(word)
                    parts.extend(sub_parts)
                    current_part = []
                    current_part_token_count = 0

        if current_part:
            parts.append(" ".join(current_part))

        return parts

    def _split_by_tokens(self, text: str) -> List[str]:
        """
        Split text by tokens when other methods fail.

        Args:
            text: Text to split

        Returns:
            List of text parts
        """
        # Calculate approximate characters per token
        chars_per_token = max(1, len(text) // calculate_token_count(text))
        max_chars = self.settings.chunk_size * chars_per_token

        parts = []
        for i in range(0, len(text), max_chars):
            parts.append(text[i:i + max_chars])

        return parts

    def _apply_overlap(self, current_chunk: List[str], sentences: List[str], start_idx: int, next_sentence_token_count) -> tuple:
        """
        Apply overlap to the current chunk by including some sentences from the previous chunk.

        Args:
            current_chunk: Current chunk of sentences
            sentences: Original list of all sentences
            start_idx: Index where we should start the next chunk
            next_sentence_token_count: Token count of the next sentence

        Returns:
            Tuple of (new_chunk_sentences, new_chunk_token_count)
        """
        # Calculate how many sentences from the end of current_chunk to include as overlap
        if self.settings.chunk_overlap == 0:
            return [], 0

        # Create new chunk starting with overlap sentences
        overlap_sentences = []
        overlap_token_count = 0

        # Get sentences from the end of current chunk for overlap
        for i in range(len(current_chunk) - 1, -1, -1):
            sentence_token_count = calculate_token_count(current_chunk[i])
            if overlap_token_count + sentence_token_count <= self.settings.chunk_overlap:
                overlap_sentences.insert(0, current_chunk[i])  # Insert at beginning to maintain order
                overlap_token_count += sentence_token_count
            else:
                break

        return overlap_sentences, overlap_token_count

    def _create_chunk(self, content: str, url: str, section_title: str, chunk_index: int) -> Dict[str, Any]:
        """
        Create a chunk dictionary with metadata.

        Args:
            content: Chunk content
            url: Source URL
            section_title: Section title
            chunk_index: Index of this chunk

        Returns:
            Chunk dictionary with metadata
        """
        from utils.helpers import generate_chunk_id, calculate_token_count

        chunk_id = generate_chunk_id(url, chunk_index)
        token_count = calculate_token_count(content)

        chunk = {
            "id": chunk_id,
            "content": content,
            "url": url,
            "section_title": section_title,
            "chunk_index": chunk_index,
            "metadata": {
                "token_count": token_count,
                "word_count": len(content.split()),
                "char_count": len(content)
            }
        }

        return chunk

    def chunk_text_by_paragraph(self, text: str, url: str, section_title: str = "") -> List[Dict[str, Any]]:
        """
        Alternative chunking method that splits by paragraphs first.

        Args:
            text: Text to chunk
            url: Source URL for the text
            section_title: Title of the section (optional)

        Returns:
            List of chunk dictionaries with content, metadata, and IDs
        """
        import re

        # Split by paragraphs (double newlines or more)
        paragraphs = re.split(r'\n\s*\n', text)
        paragraphs = [p.strip() for p in paragraphs if p.strip()]

        chunks = []
        chunk_index = 0

        current_chunk_content = ""
        current_chunk_token_count = 0

        for paragraph in paragraphs:
            paragraph_token_count = calculate_token_count(paragraph)

            if current_chunk_token_count + paragraph_token_count <= self.settings.chunk_size:
                # Add paragraph to current chunk
                if current_chunk_content:
                    current_chunk_content += "\n\n" + paragraph
                else:
                    current_chunk_content = paragraph
                current_chunk_token_count += paragraph_token_count
            else:
                # Finalize current chunk
                if current_chunk_content:
                    chunk = self._create_chunk(current_chunk_content, url, section_title, chunk_index)
                    chunks.append(chunk)
                    chunk_index += 1

                # Start new chunk
                current_chunk_content = paragraph
                current_chunk_token_count = paragraph_token_count

        # Add the last chunk if it has content
        if current_chunk_content:
            chunk = self._create_chunk(current_chunk_content, url, section_title, chunk_index)
            chunks.append(chunk)

        return chunks