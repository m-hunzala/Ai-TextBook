import asyncpg
import logging
from typing import Optional
from datetime import datetime

logger = logging.getLogger(__name__)

class PostgresClient:
    def __init__(self):
        from app.config import settings
        self.connection_string = settings.effective_database_url

    async def __aenter__(self):
        try:
            self.pool = await asyncpg.create_pool(
                dsn=self.connection_string,
                min_size=1,
                max_size=10,
                command_timeout=60
            )
            # Initialize tables if they don't exist
            await self._initialize_tables()
        except Exception as e:
            logger.error(f"Error connecting to PostgreSQL: {e}")
            raise
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if hasattr(self, 'pool'):
            await self.pool.close()

    async def _initialize_tables(self):
        """Create necessary tables if they don't exist"""
        async with self.pool.acquire() as conn:
            # Create books table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS books (
                    id SERIAL PRIMARY KEY,
                    title VARCHAR(255) NOT NULL,
                    author VARCHAR(255),
                    content_hash VARCHAR(255),
                    chunk_count INTEGER,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)

            # Create book_chunks table for ingestion script
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS book_chunks (
                    id SERIAL PRIMARY KEY,
                    book_id TEXT,
                    chunk_id TEXT,
                    chapter TEXT,
                    page INTEGER,
                    start_pos INTEGER,
                    end_pos INTEGER,
                    qdrant_id TEXT,
                    created_at TIMESTAMPTZ DEFAULT NOW()
                )
            """)

            # Create queries table for logging
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS queries (
                    id SERIAL PRIMARY KEY,
                    query_text TEXT NOT NULL,
                    response_text TEXT NOT NULL,
                    context_text TEXT,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)

            logger.info("PostgreSQL tables initialized")

    async def store_book_metadata(self, title: str, author: str, content_hash: Optional[str], chunk_count: int):
        """Store book metadata in PostgreSQL"""
        async with self.pool.acquire() as conn:
            query = """
                INSERT INTO books (title, author, content_hash, chunk_count)
                VALUES ($1, $2, $3, $4)
            """
            await conn.execute(query, title, author, content_hash, chunk_count)
            logger.info(f"Stored book metadata: {title} by {author}")

    async def store_query_log(self, query: str, response: str, context: str):
        """Store query and response in PostgreSQL for analytics"""
        async with self.pool.acquire() as conn:
            query_sql = """
                INSERT INTO queries (query_text, response_text, context_text)
                VALUES ($1, $2, $3)
            """
            await conn.execute(query_sql, query, response, context)
            logger.info(f"Stored query log for: {query[:50]}...")