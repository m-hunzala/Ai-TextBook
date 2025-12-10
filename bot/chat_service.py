import asyncio
import logging
import os
import uuid
from typing import List, Dict, Any, Optional, Tuple
from datetime import datetime

from qdrant_client import QdrantClient
from qdrant_client.http import models
import openai
from openai import AsyncOpenAI
import tiktoken
from sentence_transformers import SentenceTransformer
import asyncpg

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Environment variables
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
OPENAI_MODEL = os.getenv("OPENAI_MODEL", "gpt-3.5-turbo")


class ChatService:
    def __init__(self):
        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            prefer_grpc=True
        )

        # Initialize OpenAI client
        self.openai_client = AsyncOpenAI(api_key=OPENAI_API_KEY)

        # Initialize embedding model (either OpenAI or local)
        self.embedding_model = os.getenv("EMBEDDING_MODEL", "text-embedding-3-small")
        self.use_local_embeddings = os.getenv("USE_LOCAL_EMBEDDINGS", "false").lower() == "true"

        if self.use_local_embeddings:
            self.local_embedding_model = SentenceTransformer('all-MiniLM-L6-v2')

        # Initialize database connection
        self.db_pool = None
        self._db_initialized = False

        # Initialize session storage (in a real app, this would be a database)
        self.sessions = {}

        self.collection_name = "documents"

    async def init_db(self):
        """Initialize the database connection pool."""
        if not self._db_initialized:
            self.db_pool = await asyncpg.create_pool(NEON_DB_URL)
            self._db_initialized = True

    async def store_highlight(self, document_url: str, highlight_text: str, user_id: Optional[str] = None, session_id: Optional[str] = None):
        """Store user highlight in the database."""
        if not self._db_initialized:
            await self.init_db()

        async with self.db_pool.acquire() as conn:
            await conn.execute('''
                INSERT INTO user_highlights (document_url, highlight_text, user_id, session_id)
                VALUES ($1, $2, $3, $4)
            ''', document_url, highlight_text, user_id, session_id)
    
    async def _get_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Get embeddings for a list of texts."""
        if self.use_local_embeddings:
            # Use local embedding model
            embeddings = self.local_embedding_model.encode(texts)
            return [embedding.tolist() for embedding in embeddings]
        else:
            # Use OpenAI embedding API
            response = await self.openai_client.embeddings.create(
                input=texts,
                model=self.embedding_model
            )
            return [data.embedding for data in response.data]
    
    async def _search_in_qdrant(self, query_embedding: List[float], limit: int = 5, highlight_text: Optional[str] = None) -> List[Dict[str, Any]]:
        """Search in Qdrant for relevant documents."""
        search_filter = None
        if highlight_text:
            # If we're searching within a highlighted text, filter by text content
            search_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="text",
                        match=models.MatchText(text=highlight_text)
                    )
                ]
            )
        
        results = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            query_filter=search_filter,
            limit=limit
        )
        
        # Extract relevant information from search results
        documents = []
        for hit in results:
            documents.append({
                "text": hit.payload.get("text", ""),
                "source_url": hit.payload.get("source_url", ""),
                "source_type": hit.payload.get("source_type", ""),
                "chunk_id": hit.payload.get("chunk_id", ""),
                "score": hit.score
            })
        
        return documents
    
    async def query(self, query: str, highlight_text: Optional[str] = None, session_id: Optional[str] = None, document_url: Optional[str] = None) -> Dict[str, Any]:
        """Query the RAG system."""
        # Create embedding for the query
        query_embedding = (await self._get_embeddings([query]))[0]

        # If highlight_text is provided, store it in the database
        if highlight_text:
            # Use the provided document_url if available, otherwise use a placeholder
            doc_url = document_url or "unknown"
            await self.store_highlight(doc_url, highlight_text, session_id=session_id)

        # Search for relevant documents
        documents = await self._search_in_qdrant(query_embedding, limit=5, highlight_text=highlight_text)

        if not documents:
            return {
                "response": "I couldn't find any relevant information to answer your query.",
                "sources": [],
                "session_id": session_id
            }

        # Generate response using OpenAI
        context = "\n".join([doc["text"] for doc in documents])
        prompt = f"""
        Based on the following context, please answer the question. If the context doesn't contain enough information, say so.

        Context: {context}

        Question: {query}

        Answer:
        """

        response = await self.openai_client.chat.completions.create(
            model=OPENAI_MODEL,
            messages=[
                {"role": "system", "content": "You are a helpful assistant that provides accurate answers based on provided context. Always cite your sources when possible."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=500,
            temperature=0.3
        )

        answer = response.choices[0].message.content

        # Prepare sources for response
        sources = []
        for doc in documents:
            sources.append({
                "text": doc["text"][:200] + "..." if len(doc["text"]) > 200 else doc["text"],  # Truncate for response
                "source_url": doc["source_url"],
                "score": doc["score"]
            })

        return {
            "response": answer,
            "sources": sources,
            "session_id": session_id or str(uuid.uuid4())
        }
    
    async def chat(self, message: str, session_id: Optional[str] = None, highlight_text: Optional[str] = None, document_url: Optional[str] = None) -> Dict[str, Any]:
        """Chat with memory of previous conversations."""
        # Get or create session
        if not session_id:
            session_id = str(uuid.uuid4())

        if session_id not in self.sessions:
            self.sessions[session_id] = {
                "history": [],
                "created_at": datetime.now()
            }

        # If highlight_text is provided, store it in the database
        if highlight_text:
            # Use the provided document_url if available, otherwise use a placeholder
            doc_url = document_url or "unknown"
            await self.store_highlight(doc_url, highlight_text, session_id=session_id)

        # Add user message to history
        self.sessions[session_id]["history"].append({
            "role": "user",
            "content": message,
            "timestamp": datetime.now()
        })

        # Create embedding for the query
        query_embedding = (await self._get_embeddings([message]))[0]

        # Search for relevant documents
        documents = await self._search_in_qdrant(query_embedding, limit=5, highlight_text=highlight_text)

        # Build context from documents and conversation history
        context = ""
        if documents:
            context = "Relevant context:\n" + "\n".join([doc["text"] for doc in documents])

        # Build conversation history prompt
        history_text = "\n".join([
            f"{msg['role'].title()}: {msg['content']}"
            for msg in self.sessions[session_id]["history"][-5:]  # Use last 5 messages
        ])

        full_prompt = f"""
        {context}

        Conversation history:
        {history_text}

        User: {message}

        Assistant:
        """

        try:
            response = await self.openai_client.chat.completions.create(
                model=OPENAI_MODEL,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that provides accurate answers based on provided context. Always cite your sources when possible."},
                    {"role": "user", "content": full_prompt}
                ],
                max_tokens=500,
                temperature=0.3
            )

            answer = response.choices[0].message.content

            # Add assistant response to history
            self.sessions[session_id]["history"].append({
                "role": "assistant",
                "content": answer,
                "timestamp": datetime.now()
            })

            # Prepare sources for response
            sources = []
            for doc in documents:
                sources.append({
                    "text": doc["text"][:200] + "..." if len(doc["text"]) > 200 else doc["text"],  # Truncate for response
                    "source_url": doc["source_url"],
                    "score": doc["score"]
                })

            return {
                "response": answer,
                "sources": sources,
                "session_id": session_id
            }
        except Exception as e:
            logger.error(f"Error during chat completion: {str(e)}")
            return {
                "response": f"Sorry, I encountered an error processing your request: {str(e)}",
                "sources": [],
                "session_id": session_id
            }
    
    def get_session_history(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get the chat history for a session."""
        return self.sessions.get(session_id)
    
    def clear_session(self, session_id: str):
        """Clear the chat history for a session."""
        if session_id in self.sessions:
            del self.sessions[session_id]