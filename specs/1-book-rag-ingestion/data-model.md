# Data Model: Book RAG Content Ingestion Pipeline

## Core Entities

### BookContentChunk
**Description**: Represents a segment of book content that has been extracted, cleaned, and prepared for embedding
**Fields**:
- `id` (str): Unique identifier for the chunk (UUID or URL-based)
- `content` (str): The clean text content of the chunk
- `url` (str): Source URL where the content was extracted from
- `section_title` (str): Title of the section/chapter this chunk belongs to
- `chunk_index` (int): Sequential index of this chunk within the section
- `metadata` (dict): Additional metadata (word count, token count, etc.)
- `embedding` (list[float]): Vector representation of the content (optional, for in-memory operations)

### EmbeddingVector
**Description**: Represents the numerical representation of text content in high-dimensional space, stored with associated metadata for retrieval
**Fields**:
- `vector_id` (str): Unique identifier for this vector in Qdrant
- `payload` (dict): Metadata associated with the vector containing:
  - `url` (str): Source URL
  - `section_title` (str): Section title
  - `chunk_id` (str): Original chunk identifier
  - `content` (str): Original content snippet
  - `timestamp` (datetime): When this embedding was created
- `vector` (list[float]): The actual embedding vector

### IngestionConfig
**Description**: Represents the parameters and settings that control the ingestion process
**Fields**:
- `base_urls` (list[str]): List of base URLs to crawl
- `chunk_size` (int): Maximum number of tokens per chunk (default: 512)
- `chunk_overlap` (int): Number of tokens to overlap between chunks (default: 50)
- `batch_size` (int): Number of chunks to process in each API batch (default: 10)
- `cohere_model` (str): Name of the Cohere embedding model to use (default: "embed-english-v3.0")
- `qdrant_collection` (str): Name of the Qdrant collection to store embeddings
- `max_retries` (int): Maximum number of retries for failed operations (default: 3)
- `delay_between_requests` (float): Delay in seconds between requests (default: 1.0)

## Relationships

- One `IngestionConfig` manages the ingestion of multiple `BookContentChunk` objects
- Each `BookContentChunk` generates one `EmbeddingVector` when processed
- Multiple `EmbeddingVector` objects are stored in a single Qdrant collection

## Validation Rules

### BookContentChunk
- `content` must not be empty
- `url` must be a valid URL format
- `chunk_index` must be non-negative
- `content` length should be within token limits for embedding models

### EmbeddingVector
- `vector_id` must be unique within the collection
- `vector` must have the correct dimensionality for the chosen model
- `payload` must contain required metadata fields

### IngestionConfig
- `chunk_size` must be positive and within model limits (typically < 4096 tokens)
- `chunk_overlap` must be less than `chunk_size`
- `batch_size` must be within API limits
- `base_urls` must contain at least one valid URL

## State Transitions

### BookContentChunk Lifecycle
1. **Extracted**: Content is extracted from URL, basic validation passed
2. **Cleaned**: HTML removed, content formatted, ready for chunking
3. **Prepared**: Tokenized and validated for embedding
4. **Embedded**: Embedding generated and stored in vector database
5. **Indexed**: Available for semantic search in Qdrant

## Data Flow

1. URLs are crawled and content is extracted into `BookContentChunk` objects
2. Chunks are validated and prepared for embedding generation
3. Embeddings are generated and stored as `EmbeddingVector` objects
4. Vectors are uploaded to Qdrant with associated metadata
5. Collection is indexed and ready for semantic search operations