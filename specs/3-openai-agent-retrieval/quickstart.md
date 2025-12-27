# Quickstart: OpenAI Agent with Qdrant Retrieval

## Prerequisites

- Python 3.11 or higher
- Access to OpenAI API with assistant creation permissions
- Access to Qdrant Cloud with stored book embeddings
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
   OPENAI_API_KEY=your_openai_api_key_here
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=book_embeddings  # or whatever collection name was used for ingestion
   ```

3. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

   Make sure you have the OpenAI package installed:
   ```bash
   pip install openai
   ```

## Running the Agent

1. **Start the agent with default configuration**:
   ```bash
   python agent.py
   ```

2. **Run with a specific query**:
   ```bash
   python agent.py --query "What does the book say about ROS 2?"
   ```

3. **Run with custom parameters**:
   ```bash
   python agent.py --query "Explain physical AI concepts" --top-k 10
   ```

## Expected Behavior

The retrieval-enabled agent will:

1. Accept your query and initialize an OpenAI assistant with a retrieval tool
2. The assistant will decide if it needs to retrieve information from the book
3. If retrieval is needed, the assistant will call the retrieval tool
4. The retrieval tool will:
   - Generate an embedding for your query using Cohere
   - Search Qdrant for relevant book chunks
   - Return the most relevant chunks to the assistant
5. The assistant will synthesize a response based on the retrieved context
6. The response will be grounded in the actual book content

## Sample Interaction

```
User: What is physical AI?
Agent: Physical AI refers to the integration of artificial intelligence with physical systems, such as robots or automated machinery. According to the book content, physical AI combines perception, reasoning, and action to create systems that can interact with the real world intelligently. The key components include sensor systems for perception, reasoning engines for decision-making, and actuator systems for physical action.

[Retrieved from: https://speckitplus-book-rag-chatbot.vercel.app/docs/module-1-physical-ai/introduction]
```

## Validation

The agent includes validation to ensure:

- Responses are grounded in retrieved book content
- Retrieval tool is called appropriately
- Context relevance is maintained
- Factual accuracy is preserved

## Troubleshooting

- **OpenAI API errors**: Verify OPENAI_API_KEY is correct and you have assistant creation permissions
- **Qdrant connection errors**: Verify QDRANT_URL and QDRANT_API_KEY are correct
- **No retrieval results**: Check that embeddings exist in the specified collection
- **Agent not calling retrieval tool**: Verify the tool is properly registered with the assistant
- **Relevance issues**: Check that the Cohere model for queries matches the one used during ingestion