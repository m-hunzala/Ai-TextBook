"""
Async Neon DB connector with book_chunks table creation
"""
import asyncpg
import logging
from typing import List, Dict, Any, Optional
from app.config import settings

logger = logging.getLogger(__name__)

class NeonDB:
    def __init__(self):
        self.connection_string = settings.effective_database_url
        self.pool = None

    async def connect(self):
        """Establish connection pool to Neon DB"""
        try:
            self.pool = await asyncpg.create_pool(
                dsn=self.connection_string,
                min_size=1,
                max_size=10,
                command_timeout=60
            )
            await self._create_tables()
            logger.info("Connected to Neon DB and initialized tables")
        except Exception as e:
            logger.error(f"Error connecting to Neon DB: {e}")
            raise

    async def close(self):
        """Close connection pool"""
        if self.pool:
            await self.pool.close()

    async def _create_tables(self):
        """Create book_chunks table if it doesn't exist"""
        async with self.pool.acquire() as conn:
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
            logger.info("Ensured book_chunks table exists")

    async def insert_chunk_metadata(self, book_id: str, chunk_id: str, chapter: str, 
                                   page: int, start_pos: int, end_pos: int, qdrant_id: str):
        """Insert a single chunk's metadata into Postgres"""
        async with self.pool.acquire() as conn:
            await conn.execute("""
                INSERT INTO book_chunks (book_id, chunk_id, chapter, page, start_pos, end_pos, qdrant_id)
                VALUES ($1, $2, $3, $4, $5, $6, $7)
            """, book_id, chunk_id, chapter, page, start_pos, end_pos, qdrant_id)

    async def bulk_insert_chunk_metadata(self, chunks: List[Dict[str, Any]]):
        """Bulk insert chunk metadata into Postgres"""
        values = [
            (chunk['book_id'], chunk['chunk_id'], chunk['chapter'], 
             chunk['page'], chunk['start_pos'], chunk['end_pos'], chunk['qdrant_id'])
            for chunk in chunks
        ]
        
        async with self.pool.acquire() as conn:
            await conn.executemany("""
                INSERT INTO book_chunks (book_id, chunk_id, chapter, page, start_pos, end_pos, qdrant_id)
                VALUES ($1, $2, $3, $4, $5, $6, $7)
            """, values)

    async def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """Retrieve a chunk by its ID"""
        async with self.pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT * FROM book_chunks WHERE chunk_id = $1", chunk_id
            )
            return dict(row) if row else None