# Research: OpenAI Agent with Qdrant Retrieval

## Decision: OpenAI Agents SDK Integration
**Rationale**: Using the OpenAI Agents SDK (Assistants API) to create a retrieval-enabled agent that can call tools during conversation. This provides a managed way to handle agent state, conversation history, and tool integration without implementing complex orchestration from scratch.

**Alternatives considered**:
- LangChain agents (would require additional orchestration for tool calling)
- Custom agent implementation (would require significant development effort)
- OpenAI Chat Completions API without Assistants (would lose managed conversation state)

## Decision: Tool-Based Retrieval Architecture
**Rationale**: Implementing retrieval as a registered tool that the agent can call when needed. This allows the agent to decide when to retrieve information from Qdrant based on the user query, providing a natural conversational flow.

**Alternatives considered**:
- Pre-retrieval approach (retrieve context before agent interaction) - would limit agent's ability to make retrieval decisions
- Manual context injection (would require custom orchestration)
- Function calling approach (would be less integrated than proper tool registration)

## Decision: Single File Implementation
**Rationale**: The user specifically requested a single file implementation (`backend/agent.py`). This keeps the agent lightweight, focused, and easy to execute without complex setup. It also allows for easy distribution and execution in various environments.

**Alternatives considered**:
- Multi-module approach (separate files for each functionality) - would add complexity without significant benefit for an agent
- Integration into existing agent systems - would mix different agent approaches

## Decision: Environment Variable Configuration
**Rationale**: Using python-dotenv for configuration management to securely handle API keys and connection parameters, consistent with the existing retrieval pipeline.

**Alternatives considered**:
- Direct configuration files (potential security risks)
- Command-line arguments for sensitive data (not recommended for API keys)

## Decision: Cohere for Query Embeddings
**Rationale**: Using Cohere API to generate embeddings for query text, matching the same model used during ingestion. This ensures consistency in the embedding space for accurate similarity matching.

**Alternatives considered**:
- OpenAI embeddings (would require different API key and pricing model)
- Hugging Face transformers (self-hosted, more complex setup)
- Sentence Transformers (local models, potentially different from ingestion model)

## Decision: Validation and Logging
**Rationale**: Implementing comprehensive logging and validation to ensure the agent is using retrieved context appropriately and producing grounded responses. This includes tracking when retrieval is called and verifying response accuracy.

**Alternatives considered**:
- Basic logging only (insufficient for validation of grounded responses)
- External validation tools (would add complexity)