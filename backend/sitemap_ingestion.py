"""
Sitemap-based RAG Content Ingestion Pipeline.

This script parses a sitemap.xml file to extract all URLs and then runs the
RAG content ingestion pipeline on all of them.
"""
import asyncio
import logging
import argparse
import os
from typing import List

from sitemap_parser import parse_sitemap
from main import main as run_ingestion_pipeline
from config.settings import Settings


async def run_ingestion_on_sitemap_urls(sitemap_url: str, max_urls: int = None):
    """
    Parse sitemap and run ingestion pipeline on all extracted URLs.

    Args:
        sitemap_url: URL of the sitemap.xml to parse
        max_urls: Maximum number of URLs to process (useful for testing)
    """
    logger = logging.getLogger(__name__)

    # Parse the sitemap to get all URLs
    logger.info(f"Starting sitemap parsing for: {sitemap_url}")
    urls = await parse_sitemap(sitemap_url)

    if max_urls:
        urls = urls[:max_urls]
        logger.info(f"Limited to first {len(urls)} URLs as per max_urls setting")

    logger.info(f"Found {len(urls)} URLs to process from sitemap")

    if not urls:
        logger.error("No URLs found in sitemap. Exiting.")
        return

    # Print the URLs we'll process
    logger.info("URLs to be processed:")
    for i, url in enumerate(urls, 1):
        logger.info(f"{i:3d}. {url}")

    # Create settings object
    settings = Settings()

    # Update settings with all the URLs
    settings.base_urls = urls

    # Run the ingestion pipeline with all URLs
    logger.info("Starting ingestion pipeline for all URLs...")

    # We'll need to recreate the main pipeline logic here since we can't easily
    # pass the URLs to the existing main function without modifying it
    from src.crawler import Crawler
    from src.extractor import Extractor
    from src.chunker import Chunker
    from src.embedder import Embedder
    from src.storage import Storage

    # Initialize components
    crawler = Crawler(settings)
    extractor = Extractor()
    chunker = Chunker(settings)
    embedder = Embedder(settings)
    storage = Storage(settings)

    try:
        # Step 1: Crawl all URLs to get content
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

        logger.info(f"Ingestion completed successfully! Processed {len(all_chunks)} chunks from {len(settings.base_urls)} URLs.")

    except Exception as e:
        logger.error(f"Error during ingestion: {str(e)}")
        raise


async def main():
    """Main function to run sitemap-based ingestion."""
    parser = argparse.ArgumentParser(description='Run RAG ingestion on all URLs from a sitemap')
    parser.add_argument('--sitemap-url', type=str, required=True,
                        help='URL of the sitemap.xml to parse and ingest')
    parser.add_argument('--max-urls', type=int, default=None,
                        help='Maximum number of URLs to process (useful for testing)')

    args = parser.parse_args()

    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    logger = logging.getLogger(__name__)
    logger.info("Starting sitemap-based RAG content ingestion pipeline")

    await run_ingestion_on_sitemap_urls(args.sitemap_url, args.max_urls)


if __name__ == "__main__":
    asyncio.run(main())