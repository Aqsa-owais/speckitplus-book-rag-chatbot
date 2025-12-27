"""
Sitemap Parser for the Book RAG Content Ingestion Pipeline.

This script parses sitemap.xml files to extract all URLs for content ingestion.
"""
import asyncio
import logging
import xml.etree.ElementTree as ET
from typing import List
import aiohttp
from urllib.parse import urljoin, urlparse


async def fetch_sitemap_content(sitemap_url: str) -> str:
    """
    Fetch the content of a sitemap URL.

    Args:
        sitemap_url: URL of the sitemap to fetch

    Returns:
        Raw XML content of the sitemap
    """
    logger = logging.getLogger(__name__)

    try:
        timeout = aiohttp.ClientTimeout(total=30)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.get(sitemap_url) as response:
                if response.status == 200:
                    content = await response.text()
                    logger.info(f"Successfully fetched sitemap: {sitemap_url}")
                    return content
                else:
                    logger.error(f"Failed to fetch sitemap {sitemap_url}: HTTP {response.status}")
                    return ""
    except Exception as e:
        logger.error(f"Error fetching sitemap {sitemap_url}: {e}")
        return ""


def parse_sitemap_content(sitemap_content: str, base_url: str = None) -> List[str]:
    """
    Parse sitemap XML content and extract all URLs.

    Args:
        sitemap_content: Raw XML content of the sitemap
        base_url: Base URL for resolving relative URLs (optional)

    Returns:
        List of URLs extracted from the sitemap
    """
    if not sitemap_content:
        return []

    try:
        root = ET.fromstring(sitemap_content)

        # Handle both regular sitemaps and sitemap indexes
        urls = []

        # Define namespaces used in sitemaps
        namespaces = {
            'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9',
            'xhtml': 'http://www.w3.org/1999/xhtml'
        }

        # Try to find URLs in <url><loc> elements (regular sitemap)
        for url_elem in root.findall('.//sitemap:url', namespaces):
            loc_elem = url_elem.find('sitemap:loc', namespaces)
            if loc_elem is not None and loc_elem.text:
                urls.append(loc_elem.text.strip())

        # If no URLs found, try without namespace (fallback)
        if not urls:
            for url_elem in root.findall('.//url'):
                loc_elem = url_elem.find('loc')
                if loc_elem is not None and loc_elem.text:
                    urls.append(loc_elem.text.strip())

        # Remove duplicates while preserving order
        unique_urls = []
        seen = set()
        for url in urls:
            if url not in seen:
                unique_urls.append(url)
                seen.add(url)

        return unique_urls

    except ET.ParseError as e:
        logging.error(f"Error parsing sitemap XML: {e}")
        return []
    except Exception as e:
        logging.error(f"Unexpected error parsing sitemap: {e}")
        return []


async def parse_sitemap(sitemap_url: str) -> List[str]:
    """
    Fetch and parse a sitemap to extract all URLs.

    Args:
        sitemap_url: URL of the sitemap to parse

    Returns:
        List of URLs extracted from the sitemap
    """
    sitemap_content = await fetch_sitemap_content(sitemap_url)
    urls = parse_sitemap_content(sitemap_content, sitemap_url)
    return urls


async def main():
    """Main function to demonstrate sitemap parsing."""
    import argparse

    parser = argparse.ArgumentParser(description='Parse sitemap.xml and extract URLs for RAG ingestion')
    parser.add_argument('--sitemap-url', type=str, required=True,
                        help='URL of the sitemap.xml to parse')
    parser.add_argument('--output-file', type=str,
                        help='Optional file to save extracted URLs')

    args = parser.parse_args()

    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    logger = logging.getLogger(__name__)
    logger.info(f"Starting sitemap parsing for: {args.sitemap_url}")

    # Parse the sitemap
    urls = await parse_sitemap(args.sitemap_url)

    logger.info(f"Found {len(urls)} URLs in sitemap:")
    for i, url in enumerate(urls, 1):
        print(f"{i:3d}. {url}")

    # Optionally save to file
    if args.output_file:
        with open(args.output_file, 'w', encoding='utf-8') as f:
            for url in urls:
                f.write(url + '\n')
        logger.info(f"URLs saved to {args.output_file}")

    return urls


if __name__ == "__main__":
    # If run directly with command line arguments, use the main function
    import sys
    if len(sys.argv) > 1:
        asyncio.run(main())
    else:
        # Example usage
        async def example():
            sitemap_url = "https://speckitplus-book-rag-chatbot.vercel.app/sitemap.xml"
            urls = await parse_sitemap(sitemap_url)
            print(f"Found {len(urls)} URLs:")
            for url in urls[:10]:  # Print first 10 URLs as example
                print(f"  - {url}")
            if len(urls) > 10:
                print(f"  ... and {len(urls) - 10} more URLs")
            return urls

        # Run the example
        extracted_urls = asyncio.run(example())