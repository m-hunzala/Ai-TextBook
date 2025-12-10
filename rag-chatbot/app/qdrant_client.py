"""
Qdrant client for vector operations
"""
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
import logging
from app.config import settings

logger = logging.getLogger(__name__)

class QdrantClient:
    def __init__(self):
        self.client = AsyncQdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=10
        )
        self.collection_name = settings.QDRANT_COLLECTION

    async def __aenter__(self):
        # Check if collection exists, create if not
        try:
            collections = await self.client.get_collections()
            collection_names = [coll.name for coll in collections.collections]
            
            if self.collection_name not in collection_names:
                await self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1536,  # Default OpenAI embedding size
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Created collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Error checking/creating collection: {e}")
            raise
        
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.client.aclose()

    async def upsert_vectors(self, vectors: List[List[float]], metadata: List[Dict[str, Any]]):
        """
        Upsert vectors with metadata to Qdrant
        
        Args:
            vectors: List of embedding vectors
            metadata: List of metadata dictionaries corresponding to each vector
        """
        # Create PointStructs for upserting
        points = []
        for i, (vector, meta) in enumerate(zip(vectors, metadata)):
            points.append(models.PointStruct(
                id=meta.get('chunk_id', f'chunk_{i}'),
                vector=vector,
                payload=meta
            ))
        
        await self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )
        logger.info(f"Upserted {len(points)} vectors to collection {self.collection_name}")

    async def search_vectors(self, query_vector: List[float], top_k: int = 5):
        """
        Search for similar vectors in Qdrant
        
        Args:
            query_vector: The query embedding vector
            top_k: Number of results to return
            
        Returns:
            List of search results
        """
        results = await self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=top_k
        )
        return results

    async def delete_collection(self):
        """Delete the entire collection (use with caution!)"""
        await self.client.delete_collection(self.collection_name)
        logger.info(f"Deleted collection: {self.collection_name}")