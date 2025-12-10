import hashlib
from typing import Any

def get_content_hash(content: str) -> str:
    """
    Generate a hash for the content to check for duplicates
    """
    return hashlib.sha256(content.encode()).hexdigest()

def clean_text(text: str) -> str:
    """
    Clean text by removing extra whitespace and normalizing
    """
    # Remove extra whitespaces
    text = ' '.join(text.split())
    return text.strip()

def truncate_text(text: str, max_length: int = 1000) -> str:
    """
    Truncate text to a maximum length
    """
    if len(text) <= max_length:
        return text
    return text[:max_length] + "..."

def format_response(response: str) -> str:
    """
    Format the response for better display
    """
    # Add any formatting needed for the response
    return response.strip()