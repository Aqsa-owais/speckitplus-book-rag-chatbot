# Quickstart: RAG Retrieval Pipeline Validation

## Prerequisites

- Python 3.11 or higher
- Access to Qdrant Cloud with stored embeddings from the ingestion pipeline
- Cohere API key
- Existing embeddings stored in Qdrant from the ingestion pipeline

## Setup

1. **Ensure backend directory exists**:
   ```bash
   cd rag-chatbot/backend
   ```

2. **Verify environment variables**:
   ```bash
   # Check that your .env file has the required variables
   cat .env
   ```

   Your `.env` file should contain:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=book_embeddings  # or whatever collection name was used for ingestion
   ```

3. **Install dependencies** (if not already installed from the ingestion pipeline):
   ```bash
   pip install -r requirements.txt
   ```

## Running the Validation

1. **Run with default test queries**:
   ```bash
   python retrieve.py
   ```

2. **Run with a custom query**:
   ```bash
   python retrieve.py --query "What does the book say about vector databases?"
   ```

3. **Run with custom parameters**:
   ```bash
   python retrieve.py --query "Explain semantic search" --top-k 10 --collection-name my_embeddings
   ```

4. **Run with multiple test queries from a file**:
   ```bash
   python retrieve.py --query-file test_queries.txt
   ```

## Expected Output

The validation tool will output:
- Retrieved text chunks ranked by relevance
- Associated metadata (URL, section, chunk ID)
- Similarity scores for each result
- Validation metrics (metadata completeness, execution time, etc.)

## Validation Report

After running the validation, a report will be generated showing:
- Query execution success/failure
- Percentage of results with complete metadata
- Average relevance of results
- Execution time statistics
- Embedding dimension verification results
- Consistency of results across multiple runs

## Troubleshooting

- **Connection errors**: Verify QDRANT_URL and QDRANT_API_KEY are correct
- **No results returned**: Check that embeddings exist in the specified collection
- **Wrong embedding model**: Ensure the Cohere model for queries matches the one used for ingestion
- **Missing metadata**: Verify that the ingestion pipeline stored metadata correctly
- **Embedding dimension mismatch**: Check that the embedding model used for queries matches the one used during ingestion