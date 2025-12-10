import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional
from dotenv import load_dotenv
from embeddings import embedding_provider

load_dotenv()

class QdrantVectorStore:
    def __init__(self):
        self.url = os.getenv("QDRANT_URL")
        self.api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "documents")
        
        if not self.url:
            raise ValueError("QDRANT_URL environment variable is required")
        
        # Initialize Qdrant client
        if self.api_key:
            self.client = QdrantClient(url=self.url, api_key=self.api_key)
        else:
            self.client = QdrantClient(url=self.url)
        
        # Create collection if it doesn't exist
        self._create_collection()

    def _get_vector_size(self) -> int:
        """
        Determine vector size based on the embedding provider
        This method is synchronous and determines vector size based on environment variables
        """
        # Check which provider is configured and return appropriate size
        if os.getenv("GEMINI_API_KEY"):
            return 768  # Google's embedding-001 returns 768-dimensional vectors
        elif os.getenv("OPENAI_API_KEY"):
            return 1536  # OpenAI's text-embedding-ada-002 returns 1536-dimensional vectors
        else:
            return 768  # Default fallback

    def _create_collection(self):
        """
        Create the collection in Qdrant if it doesn't exist
        """
        try:
            # Try to get collection info to check if it exists
            collection_info = self.client.get_collection(self.collection_name)
            print(f"Collection {self.collection_name} already exists")
        except:
            # Collection doesn't exist, create it
            vector_size = self._get_vector_size()
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )
            print(f"Created collection {self.collection_name} with vector size {vector_size}")
    
    async def upsert_vectors(self, chunks: List[Dict]) -> int:
        """
        Upsert document chunks to Qdrant
        """
        if not chunks:
            return 0
        
        # Extract texts to embed
        texts = [chunk['text'] for chunk in chunks]
        
        # Generate embeddings
        embeddings = await embedding_provider.embed_text(texts)
        
        # Prepare points for upsert
        points = []
        for i, chunk in enumerate(chunks):
            points.append(
                models.PointStruct(
                    id=i,  # Using index as ID - in production, use a proper UUID
                    vector=embeddings[i],
                    payload={
                        'text': chunk['text'],
                        'metadata': chunk['metadata']
                    }
                )
            )
        
        # Upsert to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )
        
        return len(chunks)
    
    async def search(self, query: str, top_k: int = 5, context_ids: Optional[List[str]] = None) -> List[Dict]:
        """
        Search for similar documents in Qdrant
        """
        # Generate embedding for the query
        query_embedding = await embedding_provider.embed_text([query])
        query_vector = query_embedding[0]
        
        # Prepare filters if context_ids are provided
        if context_ids:
            # Filter to only include specific document IDs
            filter_condition = models.Filter(
                must=[
                    models.HasIdCondition(
                        has_id=[int(id_str) for id_str in context_ids if id_str.isdigit()]
                    )
                ]
            )
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                query_filter=filter_condition,
                limit=top_k
            )
        else:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k
            )
        
        # Format results
        formatted_results = []
        for result in results:
            formatted_results.append({
                'id': str(result.id),
                'text': result.payload.get('text', ''),
                'metadata': result.payload.get('metadata', {}),
                'score': result.score
            })
        
        return formatted_results
    
    def delete_collection(self):
        """
        Delete the entire collection (use with caution)
        """
        self.client.delete_collection(self.collection_name)