# Data Model: OpenAI Agent with Qdrant Retrieval

## Core Entities

### Agent
**Description**: Represents the OpenAI agent instance that processes user queries and generates responses using retrieved context
**Fields**:
- `assistant_id` (str): The unique identifier for the OpenAI assistant
- `retrieval_tool_id` (str): The identifier for the registered retrieval tool
- `model` (str): The OpenAI model used by the agent (e.g., gpt-4-turbo)
- `instructions` (str): System instructions for the agent's behavior
- `metadata` (dict): Additional configuration parameters for the agent

### RetrievalTool
**Description**: Represents the function or callable that connects to Qdrant and retrieves relevant book chunks
**Fields**:
- `function_name` (str): Name of the function that can be called by the agent
- `description` (str): Description of what the tool does for the agent
- `parameters` (dict): JSON schema defining the input parameters for the tool
- `top_k` (int): Number of top results to retrieve (default: 5)
- `collection_name` (str): Name of the Qdrant collection to search

### UserQuery
**Description**: Represents the natural language input from users that the agent processes
**Fields**:
- `query_text` (str): The actual text of the user's question or request
- `session_id` (str): Identifier for the conversation session
- `timestamp` (datetime): When the query was received
- `metadata` (dict): Additional context about the query (user preferences, etc.)

### RetrievedContext
**Description**: Represents the book content retrieved from Qdrant based on the user query
**Fields**:
- `query_embedding` (list[float]): The embedding vector generated from the user query
- `retrieved_chunks` (list[dict]): List of book chunks retrieved from Qdrant
- `relevance_scores` (list[float]): Similarity scores for each retrieved chunk
- `metadata` (dict): Additional information about the retrieval process
- `total_chunks_found` (int): Total number of relevant chunks found

### AgentResponse
**Description**: Represents the output from the agent that is grounded in retrieved book content
**Fields**:
- `response_text` (str): The agent's response to the user
- `source_chunks_used` (list[str]): IDs of the chunks that informed the response
- `confidence_level` (float): Agent's confidence in the response accuracy
- `grounding_verification` (bool): Whether the response was grounded in retrieved content
- `tool_calls` (list[dict]): List of tools called during response generation
- `timestamp` (datetime): When the response was generated

## Relationships

- One `Agent` can process multiple `UserQuery` objects
- One `UserQuery` triggers one `RetrievedContext`
- One `RetrievedContext` informs one `AgentResponse`
- One `Agent` can generate multiple `AgentResponse` objects over time

## Validation Rules

### Agent
- `assistant_id` must be a valid OpenAI assistant identifier
- `model` must be a supported OpenAI model
- `instructions` must not exceed token limits

### RetrievalTool
- `function_name` must be a valid Python function name
- `top_k` must be a positive integer (typically between 1 and 20)
- `collection_name` must exist in Qdrant

### UserQuery
- `query_text` must not be empty
- `session_id` must be unique for concurrent sessions

### RetrievedContext
- `retrieved_chunks` must contain actual content
- `relevance_scores` must be between 0 and 1
- `total_chunks_found` must be non-negative

### AgentResponse
- `response_text` must not be empty
- `source_chunks_used` must reference actual retrieved chunks
- `confidence_level` must be between 0.0 and 1.0

## State Transitions

### AgentResponse Lifecycle
1. **Initialized**: User query is received, agent begins processing
2. **Tool Calling**: Agent decides to call retrieval tool
3. **Retrieved**: Context is retrieved from Qdrant
4. **Reasoning**: Agent processes retrieved context
5. **Generated**: Agent response is created
6. **Validated**: Response is verified as grounded in retrieved content
7. **Completed**: Response is returned to user

## Data Flow

1. User query is submitted to the OpenAI agent
2. Agent evaluates if retrieval is needed
3. If retrieval is needed, agent calls the retrieval tool with the query
4. Retrieval tool generates embedding and searches Qdrant for relevant chunks
5. Retrieved chunks are returned to the agent
6. Agent synthesizes response using the retrieved context
7. Agent response is validated for grounding in book content
8. Final response is returned to the user