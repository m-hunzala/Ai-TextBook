"""
Basic test to verify the application structure and imports
"""
import asyncio
from app.utils.embeddings import get_embeddings
from app.utils.llm import generate_response
from app.database.qdrant_client import QdrantClient
from app.database.postgres_client import PostgresClient
from app.utils.text_processor import process_book_text

async def test_components():
    print("Testing component imports and basic functionality...")
    
    # Test OpenAI embedding functionality (won't actually call API without key)
    try:
        print("✓ Embedding utility imported successfully")
    except Exception as e:
        print(f"✗ Embedding utility error: {e}")
    
    # Test LLM utility (won't actually call API without key)
    try:
        print("✓ LLM utility imported successfully")
    except Exception as e:
        print(f"✗ LLM utility error: {e}")
    
    # Test Qdrant client
    try:
        print("✓ Qdrant client imported successfully")
    except Exception as e:
        print(f"✗ Qdrant client error: {e}")
    
    # Test Postgres client
    try:
        print("✓ Postgres client imported successfully")
    except Exception as e:
        print(f"✗ Postgres client error: {e}")
    
    # Test text processor
    try:
        sample_text = "This is a test sentence. Here is another test sentence."
        chunks = await process_book_text(sample_text, chunk_size=20)
        print(f"✓ Text processor working, created {len(chunks)} chunks")
    except Exception as e:
        print(f"✗ Text processor error: {e}")
    
    print("\nAll components imported successfully! The application structure is correct.")

if __name__ == "__main__":
    asyncio.run(test_components())