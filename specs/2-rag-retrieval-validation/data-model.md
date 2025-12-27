# Data Model: RAG Retrieval Pipeline Validation

## Core Entities

### Query
**Description**: Represents a natural language search query input by the user for validation
**Fields**:
- `text` (str): The natural language query text
- `top_k` (int): Number of top results to retrieve (default: 5)
- `filters` (dict): Optional filters to apply to the search
- `query_embedding` (list[float]): The embedding vector generated from the query text

### RetrievalResult
**Description**: Represents a text chunk retrieved from the vector database during validation
**Fields**:
- `content` (str): The text content of the retrieved chunk
- `similarity_score` (float): The similarity score between query and this result
- `metadata` (dict): Metadata associated with the chunk containing:
  - `url` (str): Source URL where the content was extracted from
  - `section_title` (str): Title of the section/chapter this chunk belongs to
  - `chunk_id` (str): Original chunk identifier
- `rank` (int): The rank of this result in the returned list (0-indexed)

### ValidationReport
**Description**: Represents the output of the validation process with metrics and assessments
**Fields**:
- `query_text` (str): The original query that was executed
- `total_results` (int): Total number of results returned
- `metadata_completeness` (float): Percentage of results with complete metadata (0.0 to 1.0)
- `relevance_score` (float): Assessment of result relevance (0.0 to 1.0)
- `execution_time` (float): Time taken to execute the query in seconds
- `success` (bool): Whether the query execution was successful
- `error_message` (str): Error message if the query failed, otherwise None
- `results` (list[RetrievalResult]): List of retrieved results
- `validation_timestamp` (datetime): When the validation was performed

## Relationships

- One `Query` generates one `ValidationReport`
- One `ValidationReport` contains multiple `RetrievalResult` objects
- Multiple `ValidationReport` objects can be aggregated for comprehensive validation

## Validation Rules

### Query
- `text` must not be empty
- `top_k` must be a positive integer (typically between 1 and 20)
- `query_embedding` must have the correct dimensionality for the chosen model

### RetrievalResult
- `content` must not be empty
- `similarity_score` must be between 0 and 1 (for cosine similarity)
- `metadata` must contain required fields (url, section_title, chunk_id)
- `rank` must be non-negative

### ValidationReport
- `total_results` must be non-negative
- `metadata_completeness` must be between 0.0 and 1.0
- `relevance_score` must be between 0.0 and 1.0
- `results` list length must match `total_results`

## State Transitions

### ValidationReport Lifecycle
1. **Initialized**: Query is prepared, report structure is created
2. **Executing**: Query is sent to Qdrant, waiting for results
3. **Retrieved**: Results are received from Qdrant
4. **Validated**: Results are analyzed and metrics are calculated
5. **Completed**: Report is finalized with all validation metrics

## Data Flow

1. Natural language query is input and converted to embedding
2. Embedding is used to search Qdrant for similar vectors
3. Top-k matching vectors are retrieved with metadata
4. Results are validated for completeness and relevance
5. Validation report is generated with metrics and assessments