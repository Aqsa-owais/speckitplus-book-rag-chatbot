# FastAPI RAG Integration

This project implements a FastAPI server that integrates with the existing retrieval-enabled agent to provide a backend API for RAG (Retrieval-Augmented Generation) chatbot functionality. The server exposes a chat endpoint that accepts user queries, processes them through the RAG system, and returns JSON responses.

## Features

- FastAPI-based REST API for RAG chatbot functionality
- Integration with existing retrieval-enabled agent
- Chat endpoint that processes queries using RAG system
- Health check endpoint for monitoring
- Proper request/response validation
- Error handling and logging
- Simple frontend UI for local testing

## API Endpoints

- `GET /health` - Health check endpoint
- `POST /chat` - Process user queries through the RAG system

## Request/Response Format

### Chat Request
```json
{
  "query": "Your question here",
  "top_k": 5
}
```

### Chat Response
```json
{
  "response": "Agent's response to the query",
  "formatted_response": "Formatted response with citations",
  "validation": {
    "grounded": true,
    "supporting_evidence": [],
    "confidence_score": 0.8
  },
  "hallucination_detected": false,
  "retrieved_content_used": []
}
```

## Setup and Running

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables in `.env` file

3. Run the server:
   ```bash
   uvicorn api:app --reload --host 0.0.0.0 --port 8000
   ```

4. Access the API at `http://localhost:8000`
   - API documentation at `http://localhost:8000/docs`
   - Health check at `http://localhost:8000/health`

## Frontend

The project includes a simple HTML/CSS/JS frontend in the `frontend/` directory that can connect to the backend API for local testing.

## Testing

Run the tests using pytest:
```bash
pytest tests/
```