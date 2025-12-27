# Data Model: FastAPI RAG Integration

## Request Models

### ChatRequest
- **query**: `str` (required) - The user's question or query text
- **top_k**: `int` (optional, default=5) - Number of top results to retrieve from Qdrant

### Example JSON:
```json
{
  "query": "What are the key principles of good software design?",
  "top_k": 5
}
```

## Response Models

### ChatResponse
- **response**: `str` (required) - The agent's response to the query
- **formatted_response**: `str` (required) - Formatted response with sources and citations
- **validation**: `dict` (required) - Validation information about the response
  - **grounded**: `bool` - Whether the response is grounded in retrieved content
  - **supporting_evidence**: `list` - List of supporting evidence from retrieved content
  - **confidence_score**: `float` - Confidence score of the response (0.0-1.0)
- **hallucination_detected**: `bool` (required) - Whether hallucination was detected in the response
- **retrieved_content_used**: `list` (required) - List of retrieved content items used in the response

### Example JSON:
```json
{
  "response": "The agent's response text here...",
  "formatted_response": "The formatted response with citations...",
  "validation": {
    "grounded": true,
    "supporting_evidence": [
      {
        "content": "Relevant content from retrieved document...",
        "source_url": "https://example.com/source",
        "score": 0.85
      }
    ],
    "confidence_score": 0.85
  },
  "hallucination_detected": false,
  "retrieved_content_used": [
    {
      "content": "Relevant content from retrieved document...",
      "source_url": "https://example.com/source",
      "score": 0.85
    }
  ]
}
```

## Error Response Model

### ErrorResponse
- **error**: `str` (required) - Error message
- **details**: `str` (optional) - Additional error details

### Example JSON:
```json
{
  "error": "Invalid request format",
  "details": "Query field is required"
}
```

## Entity Relationships

### Chat Query → Agent Response
- One chat query maps to one agent response
- The query is processed through the RAG system to generate the response
- The response includes validation information based on the retrieved content

### Agent Response → Retrieved Content
- One agent response can reference multiple retrieved content items
- The retrieved content provides the grounding for the response
- Each retrieved item has source information and relevance score