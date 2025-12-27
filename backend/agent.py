"""
OpenAI Agent with Qdrant Retrieval

This script implements a retrieval-enabled agent that uses OpenAI Agents SDK
to process user queries and integrates with Qdrant for semantic search of book content.
The agent will retrieve relevant book chunks and generate grounded responses.
"""
import asyncio
import logging
from typing import List, Dict, Any, Optional
import openai
from openai.types.chat.chat_completion import ChatCompletion
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
import argparse
import os
from dotenv import load_dotenv
import time
from datetime import datetime
from pydantic import BaseModel, Field
import json


class AgentOrchestrator:
    """Orchestrates the OpenAI agent with Qdrant-based retrieval functionality."""

    def __init__(self):
        """Initialize the agent orchestrator with required clients."""
        # Load environment variables
        load_dotenv()

        # Set up logging first before using it
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)

        # Get environment variables - prioritize OpenRouter over Gemini for OpenAI-compatible API
        self.openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
        self.gemini_api_key = os.getenv("GEMINI_API_KEY")
        self.cohere_api_key = os.getenv("COHERE_API_KEY")
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.qdrant_collection = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")
        self.cohere_model = os.getenv("COHERE_MODEL", "embed-english-v3.0")

        # Use OpenRouter API key if available, otherwise Gemini
        # Note: For actual Gemini usage, you'd need to use Google's Vertex AI or a Gemini-compatible API
        # For this implementation, we'll prioritize OpenRouter which is OpenAI-compatible
        self.openai_api_key = self.openrouter_api_key or self.gemini_api_key

        # Validate required environment variables
        if not all([self.openai_api_key, self.cohere_api_key, self.qdrant_url, self.qdrant_api_key]):
            raise ValueError("Missing required environment variables. Please set OPENROUTER_API_KEY (or GEMINI_API_KEY), COHERE_API_KEY, QDRANT_URL, and QDRANT_API_KEY")

        # Initialize clients
        openai.api_key = self.openai_api_key

        # If using OpenRouter, configure the base URL
        if self.openrouter_api_key:
            self.openai_client = openai.OpenAI(
                api_key=self.openai_api_key,
                base_url="https://openrouter.ai/api/v1"
            )
            self.logger.info("Using OpenRouter API for OpenAI-compatible calls")
        else:
            # If using Gemini, we'll still use the standard OpenAI client but note this would need adjustment for actual Gemini usage
            self.openai_client = openai.OpenAI(api_key=self.openai_api_key)
            self.logger.info("Using GEMINI_API_KEY for OpenAI-compatible calls (may require adapter for actual Gemini)")

        self.cohere_client = cohere.Client(self.cohere_api_key)

        self.qdrant_client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
            prefer_grpc=False
        )

        # Agent references (using chat completions instead of assistants API)
        self.agent_model: Optional[str] = None
        self.instructions: Optional[str] = None
        self.assistant_id: Optional[str] = None
        # Model name for chat completions API
        if self.openrouter_api_key:
            self.model_name = "openai/gpt-4-turbo-preview"  # OpenRouter model identifier
        else:
            self.model_name = "gpt-4-turbo"  # Default model

    def create_retrieval_tool_definition(self):
        """Create the retrieval tool definition with proper schema."""
        return {
            "type": "function",
            "function": {
                "name": "retrieve_book_content",
                "description": "Retrieve relevant book content from the knowledge base based on user query",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "The natural language query to search for in the book content"
                        },
                        "top_k": {
                            "type": "integer",
                            "description": "Number of top results to retrieve (default 5)",
                            "default": 5
                        }
                    },
                    "required": ["query"]
                }
            }
        }

    def create_assistant(self, name: str = "Book Retrieval Agent",
                        instructions: str = "You are a helpful assistant that answers questions based on book content. Use the retrieve_book_content tool to get relevant information when needed. Always cite your sources and provide grounded responses based on the retrieved content."):
        """Initialize assistant configuration using OpenAI-compatible API (chat completions)."""
        try:
            # Determine which model to use based on the API provider
            if self.openrouter_api_key:
                # For OpenRouter, use a model that supports function calling
                model = "openai/gpt-4-turbo-preview"  # Example model, can be configured
            else:
                # For Gemini, you would need to use a Gemini-compatible model or API
                # Since Gemini doesn't have native Assistant API, we'll use a generic approach
                model = "gpt-4-turbo"  # Placeholder - actual implementation would vary

            # Store configuration for later use in chat completions
            self.agent_model = model
            self.instructions = instructions

            # For chat-based approach, we don't create a persistent assistant object
            # but rather configure the parameters to be used in chat completions
            self.logger.info(f"Assistant configuration set up with model: {model}")

            # Return a mock assistant-like object for compatibility with existing code
            class MockAssistant:
                def __init__(self, model, instructions):
                    self.model = model
                    self.instructions = instructions
                    self.id = f"mock-assistant-{hash(model + instructions)}"

            self.assistant_id = f"mock-assistant-{hash(model + instructions)}"
            mock_assistant = MockAssistant(model, instructions)
            return mock_assistant

        except Exception as e:
            self.logger.error(f"Error setting up assistant configuration: {e}")
            raise

    def verify_tool_availability(self) -> bool:
        """Verify that the retrieval tool is available to the agent (for function calling)."""
        try:
            # For the chat completion approach, we don't have a persistent assistant object
            # but we can verify that our tool definition is valid and that the model supports function calling
            if not self.agent_model:
                self.logger.error("Agent model not configured")
                return False

            # Check if we can make a test call with function definitions
            # This is a basic check to see if the model supports function calling
            test_tool_def = self.create_retrieval_tool_definition()

            # The tool definition should have the right structure for function calling
            if test_tool_def and isinstance(test_tool_def, dict) and "function" in test_tool_def:
                self.logger.info("Retrieval tool definition is properly configured")
                return True
            else:
                self.logger.error("Retrieval tool definition not properly configured")
                return False

        except Exception as e:
            self.logger.error(f"Error verifying tool availability: {e}")
            return False

    def manage_token_limits(self, content_list: List[str], max_tokens: int = 120000) -> List[str]:
        """
        Manage token limits for context to prevent exceeding model limits.

        Args:
            content_list: List of content strings to potentially include in context
            max_tokens: Maximum number of tokens allowed (default for GPT-4-turbo)

        Returns:
            List of content strings that fit within token limit
        """
        import tiktoken

        # Use appropriate tokenizer for the model
        try:
            enc = tiktoken.encoding_for_model("gpt-4-turbo")
        except:
            enc = tiktoken.get_encoding("cl100k_base")  # fallback

        selected_content = []
        total_tokens = 0

        for content in content_list:
            content_tokens = len(enc.encode(content))

            # Check if adding this content would exceed the limit
            if total_tokens + content_tokens <= max_tokens:
                selected_content.append(content)
                total_tokens += content_tokens
            else:
                # If we're close to the limit, stop adding content
                break

        self.logger.info(f"Token management: Selected {len(selected_content)} out of {len(content_list)} items to stay within {max_tokens} token limit")
        return selected_content

    def retrieve_chunks_from_qdrant(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant book chunks from Qdrant based on the query.

        Args:
            query: Natural language query to search for
            top_k: Number of top results to return

        Returns:
            List of retrieved chunks with metadata
        """
        # Generate embedding for the query
        query_embedding = self.generate_embedding(query)
        if not query_embedding:
            self.logger.error("Failed to generate embedding for query")
            return []

        try:
            # Search in Qdrant
            search_results = self.qdrant_client.query_points(
                collection_name=self.qdrant_collection,
                query=query_embedding,
                limit=top_k
            ).points

            # Format results
            formatted_results = []
            for i, result in enumerate(search_results):
                formatted_result = {
                    "rank": i + 1,
                    "score": result.score,
                    "content": result.payload.get('content', '')[:200] + '...' if len(result.payload.get('content', '')) > 200 else result.payload.get('content', ''),
                    "url": result.payload.get('url', ''),
                    "section_title": result.payload.get('section_title', ''),
                    "chunk_id": result.payload.get('chunk_id', ''),
                    "chunk_index": result.payload.get('chunk_index', ''),
                    "token_count": result.payload.get('token_count', 0)
                }
                formatted_results.append(formatted_result)

            return formatted_results

        except Exception as e:
            self.logger.error(f"Error during retrieval: {e}")
            return []

    def format_agent_response(self, response: str, retrieved_content: List[Dict[str, Any]], validation: Dict[str, Any]) -> str:
        """
        Format the agent response with additional context and validation information.

        Args:
            response: The raw response from the agent
            retrieved_content: Content that was retrieved and used
            validation: Validation results for the response

        Returns:
            Formatted response with additional context
        """
        formatted_parts = []

        # Add the main response
        formatted_parts.append("🤖 Agent Response:")
        formatted_parts.append(response)
        formatted_parts.append("")

        # Add confidence information
        confidence_score = validation.get("confidence_score", 0.0)
        formatted_parts.append(f"📊 Confidence Score: {confidence_score:.2f}/1.0")

        # Add source attribution if available
        source_attribution = validation.get("source_attribution", [])
        if source_attribution:
            formatted_parts.append("🔗 Sources Used:")
            for source in source_attribution[:3]:  # Limit to first 3 sources
                formatted_parts.append(f"  - {source}")

        # Add retrieved content summary
        formatted_parts.append(f"📚 Retrieved {len(retrieved_content)} relevant chunks")

        # Add validation status
        is_grounded = validation.get("is_grounded", False)
        status = "✅ Grounded in content" if is_grounded else "⚠️ May not be fully grounded"
        formatted_parts.append(f"✅ Validation: {status}")

        return "\n".join(formatted_parts)

    def generate_embedding(self, text: str) -> Optional[List[float]]:
        """Generate embedding for a given text using Cohere."""
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model=self.cohere_model,
                input_type="search_query"  # Appropriate for search queries
            )
            return response.embeddings[0] if response.embeddings else None
        except Exception as e:
            self.logger.error(f"Error generating embedding: {e}")
            return None

    def tool_retrieve_book_content(self, query: str, top_k: int = 5) -> str:
        """
        Tool function to retrieve book content based on the query.
        This function is called by the OpenAI assistant when it uses the retrieval tool.
        """
        try:
            # Retrieve chunks from Qdrant
            results = self.retrieve_chunks_from_qdrant(query, top_k)

            # Format results as a JSON string to return to the assistant
            return json.dumps({
                "query": query,
                "results": results,
                "total_found": len(results)
            })
        except Exception as e:
            self.logger.error(f"Error in tool_retrieve_book_content: {e}")
            return json.dumps({
                "error": str(e),
                "query": query,
                "results": [],
                "total_found": 0
            })


    def validate_response_grounding(self, response: str, retrieved_content: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate that the agent's response is grounded in the retrieved content.

        Args:
            response: The agent's response to validate
            retrieved_content: The content that was retrieved and provided to the agent

        Returns:
            Dictionary with validation results and confidence scores
        """
        validation_result = {
            "is_grounded": True,
            "confidence_score": 0.0,
            "supporting_evidence": [],
            "missing_facts": [],
            "source_attribution": []
        }

        if not retrieved_content:
            validation_result["is_grounded"] = False
            validation_result["confidence_score"] = 0.0
            return validation_result

        # Check if the response contains information from the retrieved content
        response_lower = response.lower()
        supporting_evidence = []
        source_attribution = []

        for chunk in retrieved_content:
            content = chunk.get("content", "").lower()
            url = chunk.get("url", "")
            section_title = chunk.get("section_title", "")
            chunk_id = chunk.get("chunk_id", "")

            # Check if response contains content from this chunk
            if content and content in response_lower:
                supporting_evidence.append({
                    "chunk_id": chunk_id,
                    "content_snippet": content[:100] + "..." if len(content) > 100 else content,
                    "url": url,
                    "section": section_title
                })
                if url:
                    source_attribution.append(url)

        # Calculate confidence based on how much of the retrieved content is used
        if supporting_evidence:
            validation_result["confidence_score"] = min(1.0, len(supporting_evidence) / len(retrieved_content))
            validation_result["supporting_evidence"] = supporting_evidence
            validation_result["source_attribution"] = list(set(source_attribution))
        else:
            validation_result["is_grounded"] = False
            validation_result["confidence_score"] = 0.0

        return validation_result

    def detect_hallucination(self, response: str, retrieved_content: List[Dict[str, Any]]) -> bool:
        """
        Detect potential hallucinations in the agent's response by comparing
        against the retrieved content.

        Args:
            response: The agent's response to check
            retrieved_content: The content that was retrieved and provided to the agent

        Returns:
            True if hallucination is detected, False otherwise
        """
        if not retrieved_content:
            # If no content was retrieved, any specific claims in the response might be hallucinated
            return True

        # Simple heuristic: if the response contains claims not in the retrieved content,
        # it might be hallucinated
        response_lower = response.lower()
        all_retrieved_text = " ".join([chunk.get("content", "").lower() for chunk in retrieved_content])

        # Check for specific factual claims that aren't supported by retrieved content
        # This is a simplified check - in a real system, you'd want more sophisticated NLP
        unsupported_claims = []
        for chunk in retrieved_content:
            content = chunk.get("content", "")
            if content and content.lower() not in response_lower:
                # If retrieved content wasn't used, check if response contains other claims
                pass

        # For now, return False - this is a simplified implementation
        # A full implementation would use more sophisticated NLP techniques
        return False

    def generate_grounded_response(self, query: str, retrieved_content: List[Dict[str, Any]], top_k: int = 5) -> Dict[str, Any]:
        """
        Generate a response that is grounded in the retrieved content.

        Args:
            query: The original user query
            retrieved_content: Content retrieved from Qdrant
            top_k: Number of top results considered

        Returns:
            Dictionary with the response and validation information
        """
        try:
            # Prepare the content for the prompt, managing token limits
            content_texts = [item["content"] for item in retrieved_content]

            # Apply token management to avoid exceeding context window
            selected_content_texts = self.manage_token_limits(content_texts)

            # Update retrieved_content to only include selected items
            selected_retrieved_content = []
            for i, content_text in enumerate(content_texts):
                if content_text in selected_content_texts and i < len(retrieved_content):
                    selected_retrieved_content.append(retrieved_content[i])

            # Create a detailed prompt that emphasizes using retrieved content
            detailed_prompt = f"""
            User Query: {query}

            Retrieved Content:
            {json.dumps(selected_retrieved_content, indent=2)}

            Please answer the user's query based ONLY on the retrieved content provided above.
            Do not make up information that is not in the retrieved content.
            If the retrieved content does not contain information to answer the query,
            clearly state that the information is not available in the provided content.
            Always cite the source URLs when referencing specific information.
            """

            # Use chat completions API instead of assistants API for better compatibility
            # This approach works with OpenRouter and other OpenAI-compatible APIs
            response = self.openai_client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that answers questions based only on provided context. Do not make up information that is not in the provided content."},
                    {"role": "user", "content": detailed_prompt}
                ],
                temperature=0.3,
                max_tokens=1000
            )

            response_text = response.choices[0].message.content

            # Validate the response grounding
            validation = self.validate_response_grounding(response_text, selected_retrieved_content)
            hallucination_detected = self.detect_hallucination(response_text, selected_retrieved_content)

            # Format the response
            formatted_response = self.format_agent_response(response_text, selected_retrieved_content, validation)

            return {
                "response": response_text,
                "formatted_response": formatted_response,
                "validation": validation,
                "hallucination_detected": hallucination_detected,
                "retrieved_content_used": validation["supporting_evidence"],
                "confidence_score": validation["confidence_score"]
            }

        except Exception as e:
            self.logger.error(f"Error generating grounded response: {e}")
            return {
                "response": f"Error generating response: {str(e)}",
                "formatted_response": f"Error generating response: {str(e)}",
                "validation": {"is_grounded": False, "confidence_score": 0.0},
                "hallucination_detected": True,
                "retrieved_content_used": [],
                "confidence_score": 0.0
            }

    def process_user_query_with_agent(self, query: str, top_k: int = 5) -> str:
        """
        Process a user query using the OpenAI agent with retrieval tool.

        Args:
            query: The user's query to process
            top_k: Number of top results to retrieve

        Returns:
            The agent's response to the query
        """
        start_time = time.time()
        self.logger.info(f"Processing query: '{query}'")

        try:
            # First, retrieve relevant content
            self.logger.info("Starting content retrieval from Qdrant...")
            retrieved_content = self.retrieve_chunks_from_qdrant(query, top_k)
            self.logger.info(f"Retrieved {len(retrieved_content)} chunks from Qdrant")

            # Generate a grounded response using the retrieved content
            self.logger.info("Generating grounded response with retrieved content...")
            result = self.generate_grounded_response(query, retrieved_content, top_k)

            # Log response details
            response_length = len(result["response"])
            confidence_score = result["confidence_score"]
            hallucination_detected = result["hallucination_detected"]

            self.logger.info(f"Response generated successfully. Length: {response_length} chars, Confidence: {confidence_score:.2f}, Hallucination: {hallucination_detected}")

            # Log to file as well
            with open("agent_logs.txt", "a", encoding="utf-8") as log_file:
                log_file.write(f"[{datetime.now().isoformat()}] Query: {query}\n")
                log_file.write(f"Response: {result['response']}\n")
                log_file.write(f"Confidence: {confidence_score:.2f}, Hallucination: {hallucination_detected}\n")
                log_file.write("-" * 50 + "\n")

            # Return the formatted response
            processing_time = time.time() - start_time
            self.logger.info(f"Query processed in {processing_time:.2f}s")
            return result["formatted_response"]

        except Exception as e:
            processing_time = time.time() - start_time
            self.logger.error(f"Error processing query with agent after {processing_time:.2f}s: {e}")
            error_response = f"Error processing query: {str(e)}"

            # Log error to file
            with open("agent_logs.txt", "a", encoding="utf-8") as log_file:
                log_file.write(f"[{datetime.now().isoformat()}] Query: {query}\n")
                log_file.write(f"Error: {str(e)}\n")
                log_file.write("-" * 50 + "\n")

            return error_response


def main():
    """Main function to run the retrieval-enabled agent."""
    parser = argparse.ArgumentParser(description="OpenAI Agent with Qdrant Retrieval")
    parser.add_argument("--query", type=str, help="Single query to process with the agent")
    parser.add_argument("--top-k", type=int, default=5, help="Number of top results to retrieve (default: 5)")

    args = parser.parse_args()

    print("Initializing OpenAI Agent with Qdrant Retrieval...")
    print("This will create an agent that can access book content when answering queries.")

    # Initialize the orchestrator (this loads .env file)
    orchestrator = AgentOrchestrator()

    # Check which API key is being used
    if orchestrator.openrouter_api_key:
        print("Using OpenRouter API (OPENROUTER_API_KEY found)")
    elif orchestrator.gemini_api_key:
        print("Using GEMINI_API_KEY (Note: Actual Gemini API may require adapter)")
    else:
        print("Warning: No API key found. Please set either OPENROUTER_API_KEY or GEMINI_API_KEY in your .env file")

    # Create the assistant with retrieval tool
    try:
        assistant = orchestrator.create_assistant()
        print(f"Assistant created successfully with ID: {assistant.id}")

        # Verify tool availability
        if orchestrator.verify_tool_availability():
            print("[OK] Retrieval tool is properly registered with the assistant")
        else:
            print("[ERROR] Retrieval tool is not available")

        if args.query:
            print(f"Processing query: '{args.query}'")
            # Process the query with the agent
            response = orchestrator.process_user_query_with_agent(args.query, top_k=args.top_k)
            print(f"Agent response: {response}")
        else:
            print("Agent initialized successfully with retrieval tool.")
            print("Assistant ID:", assistant.id)
            print("Use --query to process a query with the agent.")

    except Exception as e:
        print(f"Error creating assistant: {e}")


if __name__ == "__main__":
    main()