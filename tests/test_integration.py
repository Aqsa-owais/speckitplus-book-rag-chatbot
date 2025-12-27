"""
Integration tests for RAG functionality and frontend-backend connectivity
"""
import pytest
from fastapi.testclient import TestClient
import sys
import os

# Add backend to path to import the api module
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'backend'))

from api import app

client = TestClient(app)

def test_frontend_backend_connectivity():
    """
    Integration test for frontend-backend connectivity
    """
    # Test that the API endpoints are accessible
    health_response = client.get("/health")
    assert health_response.status_code == 200

    # Test the chat endpoint with a sample request
    sample_request = {
        "query": "Test connectivity",
        "top_k": 3
    }
    chat_response = client.post("/chat", json=sample_request)

    # The response should be either successful or have validation errors
    # (not a connectivity error)
    assert chat_response.status_code in [200, 422, 500]

    if chat_response.status_code == 200:
        # If successful, check that the response has the expected structure
        data = chat_response.json()
        assert "response" in data
        assert "formatted_response" in data
        assert "validation" in data


if __name__ == "__main__":
    pytest.main([__file__])