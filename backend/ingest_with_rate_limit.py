import asyncio
import time
from src.crawler import crawl_url
from src.extractor import extract_clean_text
from src.chunker import chunk_content
from src.embedder import generate_embeddings
from src.storage import StorageManager
from config.settings import Settings
from dotenv import load_dotenv
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

load_dotenv()
settings = Settings()

async def main():
    # URLs to process
    urls = ["https://speckitplus-book-rag-chatbot.vercel.app"]

    # Initialize storage manager
    storage_manager = StorageManager(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        collection_name=settings.qdrant_collection_name
    )

    for url in urls:
        print(f"Processing {url}")

        # Step 1: Crawl URL
        print("Step 1: Crawling URL")
        raw_content = crawl_url(url)
        if not raw_content:
            print(f"Failed to crawl {url}")
            continue

        # Step 2: Extract clean text
        print("Step 2: Extracting clean text")
        clean_text = extract_clean_text(raw_content)
        if not clean_text:
            print(f"No clean text extracted from {url}")
            continue

        # Step 3: Chunk content
        print("Step 3: Chunking content")
        chunks = chunk_content(
            clean_text,
            chunk_size=settings.chunk_size,
            chunk_overlap=settings.chunk_overlap
        )
        print(f"Created {len(chunks)} chunks")

        # Step 4: Generate embeddings with rate limiting
        print("Step 4: Generating embeddings (with rate limiting)")
        embeddings = []
        for i, chunk in enumerate(chunks):
            print(f"Processing chunk {i+1}/{len(chunks)}")
            try:
                # Generate embedding for this chunk
                embedding = await generate_embeddings([chunk], settings.cohere_model)
                embeddings.extend(embedding)

                # Add delay to respect rate limits
                time.sleep(1)  # Sleep for 1 second between requests

            except Exception as e:
                print(f"Error processing chunk {i+1}: {e}")
                # Add the chunk with a None embedding to maintain alignment
                embeddings.append(None)

        # Step 5: Store in Qdrant
        print("Step 5: Storing in Qdrant")
        valid_data = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            if embedding is not None:
                valid_data.append({
                    'chunk_id': f"{url}_chunk_{i:04d}",
                    'content': chunk,
                    'url': url,
                    'chunk_index': i
                })

        if valid_data:
            print(f"Storing {len(valid_data)} valid chunks in Qdrant")
            storage_manager.store_embeddings_with_metadata(
                embeddings=[embeddings[i] for i in range(len(embeddings)) if embeddings[i] is not None],
                contents=[chunk['content'] for chunk in valid_data],
                metadata=[{
                    'chunk_id': chunk['chunk_id'],
                    'url': chunk['url'],
                    'chunk_index': chunk['chunk_index']
                } for chunk in valid_data]
            )
        else:
            print("No valid data to store")

    print("Ingestion completed!")

if __name__ == "__main__":
    asyncio.run(main())