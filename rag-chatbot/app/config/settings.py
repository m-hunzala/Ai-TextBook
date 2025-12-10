from pydantic_settings import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    # OpenAI settings
    OPENAI_API_KEY: str
    OPENAI_EMBEDDING_MODEL: str = "text-embedding-ada-002"
    OPENAI_CHAT_MODEL: str = "gpt-3.5-turbo"

    # Qdrant settings
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_COLLECTION: str = "book_chunks"

    # PostgreSQL settings
    NEON_DATABASE_URL: str
    DATABASE_URL: str  # For backward compatibility with existing code

    # Application settings
    APP_NAME: str = "Book RAG Chatbot"
    DEBUG: bool = False

    # Embedding Settings
    EMBED_MODEL: str = "text-embedding-ada-002"

    # Chunking Settings
    CHUNK_SIZE: int = 800
    CHUNK_OVERLAP: int = 100

    class Config:
        env_file = ".env"

    @property
    def effective_database_url(self) -> str:
        """Returns the appropriate database URL, prioritizing NEON_DATABASE_URL"""
        return self.NEON_DATABASE_URL or self.DATABASE_URL

settings = Settings()