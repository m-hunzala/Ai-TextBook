"""
Unit tests for query_selected endpoint
Tests that the endpoint properly uses the strict system instruction and low temperature
to prevent hallucination beyond the provided selected text.
"""
import pytest
from fastapi.testclient import TestClient
from app.main import app
import json

client = TestClient(app)

def test_query_selected_with_irrelevant_question():
    """Test that the model responds with 'I don't know based on the provided text' when the question cannot be answered from the selected text."""
    # Provide a short paragraph about dogs
    selected_text = """
    Dogs are domesticated mammals that are commonly kept as pets. They are known for their loyalty and companionship with humans. 
    There are many breeds of dogs, each with different sizes, colors, and temperaments.
    """
    
    # Ask a question that cannot be answered from the provided text
    response = client.post("/query_selected", json={
        "selected_text": selected_text,
        "question": "What is the capital of France?"
    })
    
    assert response.status_code == 200
    
    data = response.json()
    answer = data["answer"].strip().lower()
    
    # Check that the response contains the expected "I don't know" message
    assert "i don't know based on the provided text" in answer


def test_query_selected_with_relevant_question():
    """Test that the model can answer questions that are present in the selected text."""
    selected_text = """
    The theory of relativity was developed by Albert Einstein. It revolutionized physics by redefining concepts of space and time.
    Special relativity was published in 1905 and deals with objects moving at constant speeds.
    """
    
    response = client.post("/query_selected", json={
        "selected_text": selected_text,
        "question": "Who developed the theory of relativity?"
    })
    
    assert response.status_code == 200
    
    data = response.json()
    answer = data["answer"].strip().lower()
    
    # Check that the model provides a relevant answer
    assert "einstein" in answer or "albert einstein" in answer


def test_query_selected_empty_text():
    """Test that the endpoint properly handles empty selected text."""
    response = client.post("/query_selected", json={
        "selected_text": "",
        "question": "What is the meaning of life?"
    })
    
    # Should return 400 for empty text
    assert response.status_code == 400


def test_query_selected_temperature_setting():
    """Test that the model does not hallucinate by asking about unrelated topics."""
    selected_text = """
    Coffee is made from coffee beans. Coffee beans are the seeds of the coffee plant.
    Coffee is consumed as a beverage in many cultures around the world.
    """
    
    # Ask about something completely unrelated to the selected text
    response = client.post("/query_selected", json={
        "selected_text": selected_text,
        "question": "What are the main components of a car engine?"
    })
    
    assert response.status_code == 200
    
    data = response.json()
    answer = data["answer"].strip().lower()
    
    # Check that the response contains the expected "I don't know" message
    assert "i don't know based on the provided text" in answer


if __name__ == "__main__":
    # Run the tests
    test_query_selected_with_irrelevant_question()
    test_query_selected_with_relevant_question()
    test_query_selected_empty_text()
    test_query_selected_temperature_setting()
    print("All tests passed!")