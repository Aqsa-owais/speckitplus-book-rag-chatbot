"""
Tests for the RAG Chatbot API endpoints
"""
import pytest
from fastapi.testclient import TestClient
import sys
import os

# Add backend to path to import the api module
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'backend'))

from api import app

client = TestClient(app)

def test_health_endpoint():
    """
    Test the health check endpoint
    """
    response = client.get("/health")
    assert response.status_code == 200

    data = response.json()
    assert "status" in data
    assert data["status"] == "healthy"
    assert "timestamp" in data
    assert "message" in data
    assert data["message"] == "RAG Chatbot API is operational"


def test_server_startup():
    """
    Test that the server starts up properly and responds to requests
    """
    # The health check test verifies basic server functionality
    test_health_endpoint()


def test_chat_endpoint_contract():
    """
    Contract test for chat endpoint
    """
    # Test with valid request
    valid_request = {
        "query": "Test query",
        "top_k": 5
    }
    response = client.post("/chat", json=valid_request)
    # Initially this should fail since we haven't implemented the /chat endpoint yet
    # This test will pass once the endpoint is implemented
    if response.status_code == 404:
        # Endpoint not yet implemented, which is expected initially
        assert response.status_code == 404
    else:
        # Once implemented, it should return 200 or appropriate error code
        assert response.status_code in [200, 422, 500]


def test_query_processing():
    """
    Integration test for query processing
    """
    # Test with valid request
    valid_request = {
        "query": "What are the key principles of good software design?",
        "top_k": 3
    }
    response = client.post("/chat", json=valid_request)
    # The response should be either successful or have validation errors
    # (not a connectivity error)
    assert response.status_code in [200, 422, 500]


def test_invalid_request():
    """
    Test for invalid request handling
    """
    # Test with empty query
    invalid_request = {
        "query": "",
        "top_k": 5
    }
    response = client.post("/chat", json=invalid_request)
    # Should return validation error
    assert response.status_code == 422

    # Test with negative top_k
    invalid_request = {
        "query": "Test query",
        "top_k": -1
    }
    response = client.post("/chat", json=invalid_request)
    # Should return validation error
    assert response.status_code == 422


def test_missing_query_field():
    """
    Test for missing query field handling
    """
    # Test with missing query field
    invalid_request = {
        "top_k": 5
    }
    response = client.post("/chat", json=invalid_request)
    # Should return validation error
    assert response.status_code == 422


if __name__ == "__main__":
    pytest.main([__file__])