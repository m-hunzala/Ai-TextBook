import os
import asyncpg
from typing import Optional
from contextlib import asynccontextmanager
from dotenv import load_dotenv

load_dotenv()

class Database:
    def __init__(self):
        self.pool = None
        self.db_url = os.getenv("NEON_DATABASE_URL")
        if not self.db_url:
            raise ValueError("NEON_DATABASE_URL environment variable is required")

    async def connect(self):
        """Create connection pool"""
        try:
            self.pool = await asyncpg.create_pool(
                dsn=self.db_url,
                min_size=1,
                max_size=10,
                command_timeout=60,
            )
            print("Successfully connected to Neon database")
        except Exception as e:
            print(f"Error connecting to Neon database: {e}")
            raise

    async def disconnect(self):
        """Close connection pool"""
        if self.pool:
            await self.pool.close()

    @asynccontextmanager
    async def get_connection(self):
        """Get a connection from the pool"""
        if not self.pool:
            await self.connect()

        conn = await self.pool.acquire()
        try:
            yield conn
        finally:
            await self.pool.release(conn)

# Global database instance
db = Database()