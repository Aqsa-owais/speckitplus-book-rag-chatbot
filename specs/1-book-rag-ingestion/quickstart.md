# Quickstart: Book RAG Content Ingestion Pipeline

## Prerequisites

- Python 3.11 or higher
- `uv` package manager installed (`pip install uv` or follow installation guide)
- Cohere API key
- Qdrant Cloud account and API key
- Access to the book website URLs you want to ingest

## Setup

1. **Clone or create the project structure**:
   ```bash
   mkdir rag-chatbot
   cd rag-chatbot
   ```

2. **Create the backend directory**:
   ```bash
   mkdir backend
   cd backend
   ```

3. **Initialize the project with uv**:
   ```bash
   uv init
   # Or if starting fresh:
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

4. **Create environment file**:
   ```bash
   cp .env.example .env
   ```

5. **Install dependencies**:
   ```bash
   # Using uv (recommended)
   uv pip install -r requirements.txt

   # Or using pip
   pip install -r requirements.txt
   ```

## Configuration

1. **Set up environment variables in `.env`**:
   ```env
   # Cohere API Configuration
   COHERE_API_KEY=your_cohere_api_key_here

   # Qdrant Configuration
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=book_embeddings

   # Ingestion Configuration
   BASE_URLS=https://your-book.com/chapter1,https://your-book.com/chapter2
   CHUNK_SIZE=512
   CHUNK_OVERLAP=50
   BATCH_SIZE=10
   COHERE_MODEL=embed-english-v3.0
   MAX_RETRIES=3
   DELAY_BETWEEN_REQUESTS=1.0
   ```

2. **Configure ingestion parameters** in your code or as command-line arguments:
   - `base_urls`: List of URLs to crawl
   - `chunk_size`: Max tokens per chunk (default: 512)
   - `chunk_overlap`: Token overlap between chunks (default: 50)

## Running the Ingestion Pipeline

1. **Prepare your book URLs** in the main configuration

2. **Execute the main script**:
   ```bash
   python main.py
   ```

3. **Or run with command-line arguments to override environment variables**:
   ```bash
   python main.py --urls https://your-book.com/chapter1 https://your-book.com/chapter2 \
                  --chunk-size 768 \
                  --collection-name my_book_embeddings
   ```

4. **Monitor the output** for progress and any errors

## Expected Output

- Content extracted from all provided URLs
- Text chunks created with proper metadata
- Embeddings generated and stored in Qdrant
- Success/failure logs for each step
- Progress tracking showing ingestion status

## Validation

1. **Check Qdrant collection**: Verify embeddings are stored with correct metadata
2. **Test search**: Perform a sample semantic search to validate the ingestion worked
3. **Review logs**: Ensure all URLs were processed successfully

## Repeatable Ingestion

The pipeline supports repeatable processing:
- Content changes are detected automatically
- Only changed content is updated
- Unchanged content is preserved

## Troubleshooting

- **API rate limits**: Adjust `delay_between_requests` in configuration
- **Memory issues**: Reduce batch size or process smaller chunks
- **Network errors**: Verify URL accessibility and API keys
- **Qdrant connection**: Check URL and API key in environment variables
- **Large books**: Process in batches to manage memory usage