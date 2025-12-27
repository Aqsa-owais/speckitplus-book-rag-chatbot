"""
Main entry point for the Book RAG Content Ingestion Pipeline.

This script orchestrates the entire process:
1. Crawls book URLs
2. Extracts clean text content
3. Chunks the content with overlap
4. Generates embeddings using Cohere
5. Stores embeddings in Qdrant with metadata
"""
import asyncio
import logging
import argparse
from src.crawler import Crawler
from src.extractor import Extractor
from src.chunker import Chunker
from src.embedder import Embedder
from src.storage import Storage
from config.settings import Settings


def setup_logging():
    """Set up basic logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description='Book RAG Content Ingestion Pipeline')

    parser.add_argument(
        '--urls',
        nargs='+',
        help='List of URLs to process (overrides BASE_URLS environment variable)'
    )
    parser.add_argument(
        '--chunk-size',
        type=int,
        help='Maximum number of tokens per chunk (overrides CHUNK_SIZE environment variable)'
    )
    parser.add_argument(
        '--chunk-overlap',
        type=int,
        help='Number of tokens to overlap between chunks (overrides CHUNK_OVERLAP environment variable)'
    )
    parser.add_argument(
        '--batch-size',
        type=int,
        help='Number of chunks to process in each API batch (overrides BATCH_SIZE environment variable)'
    )
    parser.add_argument(
        '--cohere-model',
        type=str,
        help='Name of the Cohere embedding model to use (overrides COHERE_MODEL environment variable)'
    )
    parser.add_argument(
        '--collection-name',
        type=str,
        help='Name of the Qdrant collection to store embeddings (overrides QDRANT_COLLECTION_NAME environment variable)'
    )
    parser.add_argument(
        '--max-retries',
        type=int,
        help='Maximum number of retries for failed operations (overrides MAX_RETRIES environment variable)'
    )
    parser.add_argument(
        '--delay-between-requests',
        type=float,
        help='Delay in seconds between requests (overrides DELAY_BETWEEN_REQUESTS environment variable)'
    )

    return parser.parse_args()


def update_settings_with_args(settings, args):
    """Update settings with command-line argument values."""
    if args.urls:
        settings.base_urls = args.urls
    if args.chunk_size:
        settings.chunk_size = args.chunk_size
    if args.chunk_overlap:
        settings.chunk_overlap = args.chunk_overlap
    if args.batch_size:
        settings.batch_size = args.batch_size
    if args.cohere_model:
        settings.cohere_model = args.cohere_model
    if args.collection_name:
        settings.qdrant_collection = args.collection_name
    if args.max_retries:
        settings.max_retries = args.max_retries
    if args.delay_between_requests:
        settings.delay_between_requests = args.delay_between_requests


async def main():
    """Main function to run the ingestion pipeline."""
    setup_logging()
    logger = logging.getLogger(__name__)

    # Parse command-line arguments
    args = parse_arguments()

    # Load configuration
    settings = Settings()

    # Update settings with command-line arguments
    update_settings_with_args(settings, args)

    logger.info(f"Starting Book RAG Content Ingestion Pipeline")
    logger.info(f"Processing URLs: {settings.base_urls}")
    logger.info(f"Chunk size: {settings.chunk_size}, Chunk overlap: {settings.chunk_overlap}")
    logger.info(f"Batch size: {settings.batch_size}, Cohere model: {settings.cohere_model}")

    # Initialize components
    crawler = Crawler(settings)
    extractor = Extractor()
    chunker = Chunker(settings)
    embedder = Embedder(settings)
    storage = Storage(settings)

    try:
        # Step 1: Crawl URLs to get content
        logger.info("Step 1: Crawling URLs")
        raw_contents = await crawler.crawl_urls(settings.base_urls)

        # Step 2: Extract clean text from raw HTML
        logger.info("Step 2: Extracting clean text")
        extracted_contents = []
        for url, raw_content in raw_contents:
            clean_text = extractor.extract_text(raw_content)
            extracted_contents.append((url, clean_text))

        # Step 3: Chunk the content
        logger.info("Step 3: Chunking content")
        all_chunks = []
        for url, content in extracted_contents:
            chunks = chunker.chunk_text(content, url)
            all_chunks.extend(chunks)

        # Step 4: Generate embeddings
        logger.info("Step 4: Generating embeddings")
        embedding_results = await embedder.generate_embeddings(all_chunks)

        # Step 5: Check for content updates and store in Qdrant
        logger.info("Step 5: Checking for content updates and storing in Qdrant")

        # For each URL, check if content has changed before storing
        processed_urls = set()
        for url, _ in extracted_contents:
            if url not in processed_urls:
                # Check if content has changed
                url_chunks = [chunk for chunk in all_chunks if chunk['url'] == url]
                has_changed = await storage.has_content_changed(url, url_chunks)

                if has_changed:
                    logger.info(f"Content changed for {url}, updating...")
                    # Get embedding results for this URL only
                    url_embedding_results = [result for result in embedding_results if result['url'] == url]
                    await storage.update_content(url_embedding_results)
                else:
                    logger.info(f"Content unchanged for {url}, skipping...")

                processed_urls.add(url)

        logger.info(f"Ingestion completed successfully! Processed {len(all_chunks)} chunks.")

    except Exception as e:
        logger.error(f"Error during ingestion: {str(e)}")
        raise


if __name__ == "__main__":
    asyncio.run(main())