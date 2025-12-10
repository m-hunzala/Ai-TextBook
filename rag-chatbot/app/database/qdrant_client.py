from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
from typing import List, Optional
import logging

logger = logging.getLogger(__name__)

class QdrantClient:
    def __init__(self):
        from app.config import settings
        self.client = AsyncQdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=10
        )
        self.collection_name = settings.QDRANT_COLLECTION

    async def __aenter__(self):
        # Check if collection exists, if not, create it
        try:
            collections = await self.client.get_collections()
            collection_names = [coll.name for coll in collections.collections]
            
            if self.collection_name not in collection_names:
                await self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1536,  # OpenAI embedding dimension
                        distance=models.Distance.COSINE
                    )
                )
        except Exception as e:
            logger.error(f"Error checking/creating collection: {e}")
            raise
        
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.client.aclose()

    async def store_chunks(self, chunks: List[dict]):
        """
        Store chunks in the vector database
        Each chunk should have: id, text, metadata
        """
        points = []
        for chunk in chunks:
            points.append(models.PointStruct(
                id=chunk.get('id'),
                vector=chunk.get('embedding'),
                payload={
                    "text": chunk.get('text'),
                    "source": chunk.get('source', ''),
                    "page_number": chunk.get('page_number', 0),
                    "chunk_index": chunk.get('chunk_index', 0)
                }
            ))
        
        await self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )
        logger.info(f"Stored {len(points)} chunks in Qdrant")

    async def search(self, query_vector: List[float], limit: int = 5):
        """
        Search for similar chunks based on the query vector
        """
        results = await self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit
        )
        return results

    async def delete_by_source(self, source: str):
        """
        Delete all points with a specific source
        """
        await self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.FilterSelector(
                filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="source",
                            match=models.MatchValue(value=source)
                        )
                    ]
                )
            )
        )
        logger.info(f"Deleted chunks with source: {source}")

    async def get_collection_info(self):
        """
        Get information about the collection
        """
        try:
            collection_info = await self.client.get_collection(self.collection_name)
            return collection_info
        except Exception as e:
            logger.error(f"Error getting collection info: {e}")
            raise