import asyncpg
from typing import Optional, List, Dict, Any
from .database import db
import uuid

class DataStore:
    def __init__(self):
        pass

    async def log_query(self, query: str, user_id: Optional[str] = None, session_id: Optional[str] = None, response_time_ms: Optional[int] = None) -> int:
        """
        Log a user query to the database and return the query ID
        """
        async with db.get_connection() as conn:
            query_id = await conn.fetchval(
                """
                INSERT INTO user_queries (query, user_id, session_id, response_time_ms)
                VALUES ($1, $2, $3, $4)
                RETURNING id
                """,
                query, user_id, session_id, response_time_ms
            )
            return query_id

    async def log_source_click(self, query_id: int, source_url: str, source_title: Optional[str] = None, 
                              user_id: Optional[str] = None, session_id: Optional[str] = None) -> None:
        """
        Log when a user clicks on a source link
        """
        async with db.get_connection() as conn:
            await conn.execute(
                """
                INSERT INTO clicked_sources (query_id, source_url, source_title, user_id, session_id)
                VALUES ($1, $2, $3, $4, $5)
                """,
                query_id, source_url, source_title, user_id, session_id
            )

    async def get_top_queries(self, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Get top queries by frequency
        """
        async with db.get_connection() as conn:
            rows = await conn.fetch(
                """
                SELECT query, COUNT(*) as count, AVG(response_time_ms) as avg_response_time
                FROM user_queries
                WHERE timestamp >= NOW() - INTERVAL '30 days'
                GROUP BY query
                ORDER BY count DESC
                LIMIT $1
                """,
                limit
            )
            return [dict(row) for row in rows]

    async def get_top_sources(self, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Get top clicked sources
        """
        async with db.get_connection() as conn:
            rows = await conn.fetch(
                """
                SELECT source_url, source_title, COUNT(*) as click_count
                FROM clicked_sources
                WHERE timestamp >= NOW() - INTERVAL '30 days'
                GROUP BY source_url, source_title
                ORDER BY click_count DESC
                LIMIT $1
                """,
                limit
            )
            return [dict(row) for row in rows]

    async def get_query_count(self) -> int:
        """
        Get total number of queries
        """
        async with db.get_connection() as conn:
            row = await conn.fetchrow("SELECT COUNT(*) as count FROM user_queries")
            return row['count']

    async def get_click_count(self) -> int:
        """
        Get total number of source clicks
        """
        async with db.get_connection() as conn:
            row = await conn.fetchrow("SELECT COUNT(*) as count FROM clicked_sources")
            return row['count']

# Global instance
datastore = DataStore()