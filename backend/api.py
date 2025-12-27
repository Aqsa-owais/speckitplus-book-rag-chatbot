"""
FastAPI RAG Integration API

This module implements a FastAPI server that integrates with the existing
retrieval-enabled agent to provide a backend API for RAG chatbot functionality.
The server exposes a chat endpoint that accepts user queries, processes them
through the RAG system, and returns JSON responses.
"""
import asyncio
import logging
from datetime import datetime, timezone
from typing import Optional, Dict, Any, List
from pydantic import BaseModel, Field
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from contextlib import asynccontextmanager
import os
import sys
import json

# Add the backend directory to the path so we can import agent.py
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import the existing agent functionality
from agent import AgentOrchestrator

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Pydantic models for request/response schemas
class ChatRequest(BaseModel):
    """Request model for chat endpoint"""
    query: str = Field(..., description="The user's question or query text")
    top_k: int = Field(5, ge=1, le=20, description="Number of top results to retrieve from Qdrant")


class SupportingEvidence(BaseModel):
    """Model for supporting evidence from retrieved content"""
    content: str
    source_url: str
    score: float = Field(ge=0.0, le=1.0)


class Validation(BaseModel):
    """Model for response validation information"""
    grounded: bool
    supporting_evidence: List[SupportingEvidence]
    confidence_score: float = Field(ge=0.0, le=1.0)


class RetrievedContent(BaseModel):
    """Model for retrieved content items"""
    content: str
    source_url: str
    score: float = Field(ge=0.0, le=1.0)


class ChatResponse(BaseModel):
    """Response model for chat endpoint"""
    response: str
    formatted_response: str
    validation: Validation
    hallucination_detected: bool
    retrieved_content_used: List[RetrievedContent]


class ErrorResponse(BaseModel):
    """Error response model"""
    error: str
    details: Optional[str] = None


# Application lifespan management
@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan context manager for startup/shutdown events
    """
    logger.info("Starting up FastAPI RAG Integration server...")

    # Initialize the agent orchestrator
    global agent_orchestrator
    try:
        agent_orchestrator = AgentOrchestrator()
        logger.info("Agent orchestrator initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize agent orchestrator: {e}")
        raise

    yield

    # Cleanup on shutdown
    logger.info("Shutting down FastAPI RAG Integration server...")


# Create FastAPI app with lifespan
app = FastAPI(
    title="RAG Chatbot API",
    description="FastAPI integration for RAG chatbot functionality",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware for frontend connectivity
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global exception handler
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """
    Global exception handler for unhandled exceptions
    """
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content=ErrorResponse(
            error="Internal server error",
            details=str(exc) if isinstance(exc, (ValueError, TypeError)) else "An unexpected error occurred"
        ).model_dump()
    )

# Validation exception handler
@app.exception_handler(ValueError)
async def validation_exception_handler(request: Request, exc: ValueError):
    """
    Exception handler for validation errors
    """
    logger.warning(f"Validation error: {exc}")
    return JSONResponse(
        status_code=422,
        content=ErrorResponse(
            error="Validation error",
            details=str(exc)
        ).model_dump()
    )

# Health check endpoint
@app.get("/health", response_model=Dict[str, Any])
async def health_check():
    """
    Health check endpoint that returns the status of the API server
    """
    return {
        "status": "healthy",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "message": "RAG Chatbot API is operational"
    }

# Chat endpoint
@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Process a user query through the RAG system
    """
    try:
        logger.info(f"Processing chat request: {request.query[:50]}...")

        # Validate the request
        if not request.query or not request.query.strip():
            raise HTTPException(status_code=422, detail="Query field is required and cannot be empty")

        # Check for very long user queries that might exceed token limits
        if len(request.query) > 10000:  # Adjust this limit as needed
            raise HTTPException(status_code=422, detail="Query is too long. Please limit your query to fewer characters.")

        # Process the query using the agent orchestrator
        if 'agent_orchestrator' not in globals():
            logger.error("Agent orchestrator not initialized")
            raise HTTPException(status_code=500, detail="Agent orchestrator not available")

        # We need to call the agent's internal method to get full response data
        # First, retrieve the content
        try:
            retrieved_content = agent_orchestrator.retrieve_chunks_from_qdrant(request.query, request.top_k)
        except Exception as e:
            logger.error(f"Error retrieving content from Qdrant: {e}", exc_info=True)
            raise HTTPException(status_code=500, detail=f"Qdrant database unavailable: {str(e)}")

        # Generate grounded response with full details
        try:
            result = agent_orchestrator.generate_grounded_response(request.query, retrieved_content, request.top_k)
        except Exception as e:
            logger.error(f"Error generating response with agent: {e}", exc_info=True)
            raise HTTPException(status_code=500, detail=f"Agent processing failed: {str(e)}")

        # Extract the information needed for our API response
        response_text = result.get("response", "")
        formatted_response = result.get("formatted_response", response_text)
        validation_data = result.get("validation", {})
        hallucination_detected = result.get("hallucination_detected", False)
        retrieved_content_used = result.get("retrieved_content_used", [])

        # Convert retrieved content to our API format
        retrieved_content_formatted = []
        for item in retrieved_content_used:
            if isinstance(item, dict) and "content" in item and "source_url" in item:
                retrieved_content_formatted.append(
                    RetrievedContent(
                        content=item["content"],
                        source_url=item["source_url"],
                        score=item.get("score", 0.5)  # default score if not available
                    )
                )

        # Convert validation supporting evidence if available
        supporting_evidence_formatted = []
        if validation_data and "supporting_evidence" in validation_data:
            for evidence in validation_data["supporting_evidence"]:
                if isinstance(evidence, dict) and "content" in evidence and "source_url" in evidence:
                    supporting_evidence_formatted.append(
                        SupportingEvidence(
                            content=evidence["content"],
                            source_url=evidence["source_url"],
                            score=evidence.get("score", 0.5)
                        )
                    )

        # Create the response
        chat_response = ChatResponse(
            response=response_text,
            formatted_response=formatted_response,
            validation=Validation(
                grounded=validation_data.get("grounded", True),
                supporting_evidence=supporting_evidence_formatted,
                confidence_score=validation_data.get("confidence_score", 0.5)
            ),
            hallucination_detected=hallucination_detected,
            retrieved_content_used=retrieved_content_formatted
        )

        logger.info("Chat request processed successfully")
        return chat_response

    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        logger.error(f"Error processing chat request: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")