# Quickstart: FastAPI RAG Integration

## Prerequisites

- Python 3.11+
- pip package manager
- Access to OpenRouter API key (or Gemini API key)
- Access to Cohere API key
- Access to Qdrant Cloud instance with book_embeddings collection

## Setup

1. **Clone the repository** (if needed):
   ```bash
   git clone <repository-url>
   cd rag-chatbot
   ```

2. **Install dependencies**:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

3. **Set up environment variables**:
   - Copy `.env.example` to `.env`
   - Add your API keys to the `.env` file:
     - `OPENROUTER_API_KEY` or `GEMINI_API_KEY`
     - `COHERE_API_KEY`
     - `QDRANT_URL`, `QDRANT_API_KEY`, `QDRANT_COLLECTION_NAME`

4. **Verify the agent works** (optional but recommended):
   ```bash
   python agent.py --query "Test query"
   ```

## Running the Application

1. **Start the FastAPI server**:
   ```bash
   cd backend
   uvicorn api:app --reload --host 0.0.0.0 --port 8000
   ```

2. **Access the API**:
   - API documentation: `http://localhost:8000/docs`
   - Health check: `http://localhost:8000/health`
   - Chat endpoint: `http://localhost:8000/chat` (POST request)

3. **Start the frontend** (optional):
   - Open `frontend/index.html` in a web browser
   - The frontend will connect to the backend API at `http://localhost:8000`

## Testing the API

**Example API call**:
```bash
curl -X POST "http://localhost:8000/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of good software design?",
    "top_k": 5
  }'
```

**Expected response**:
```json
{
  "response": "The agent's response text here...",
  "formatted_response": "Formatted response with citations...",
  "validation": {
    "grounded": true,
    "supporting_evidence": [],
    "confidence_score": 0.85
  },
  "hallucination_detected": false,
  "retrieved_content_used": []
}
```

## Local Development

- The server automatically reloads when you modify the `api.py` file
- API documentation is available at `http://localhost:8000/docs` with interactive testing
- Frontend connects to the API and displays the conversation in real-time
- Error responses follow standard HTTP status codes for easy debugging

## Troubleshooting

- **500 errors**: Check that environment variables are properly set and API keys are valid
- **Connection errors**: Verify Qdrant and API endpoints are accessible
- **Slow responses**: First query may be slower due to model loading; subsequent queries should be faster
- **Invalid request errors**: Ensure JSON format matches the expected schema