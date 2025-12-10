import asyncio
import logging
import os
from typing import List, Dict, Any, Optional
from pathlib import Path
from datetime import datetime
import aiohttp
import PyPDF2
import requests
from bs4 import BeautifulSoup
import markdown
from qdrant_client import QdrantClient
from qdrant_client.http import models
from sentence_transformers import SentenceTransformer
import openai
from openai import AsyncOpenAI
import tiktoken
import asyncpg

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Environment variables
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")


class IngestionService:
    def __init__(self):
        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            prefer_grpc=True
        )

        # Initialize embedding model (either OpenAI or local)
        self.embedding_model = os.getenv("EMBEDDING_MODEL", "text-embedding-3-small")
        self.use_local_embeddings = os.getenv("USE_LOCAL_EMBEDDINGS", "false").lower() == "true"

        if self.use_local_embeddings:
            self.local_embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
        else:
            self.openai_client = AsyncOpenAI(api_key=OPENAI_API_KEY)

        # Initialize database connection
        self.db_pool = None
        self._db_initialized = False

        # Initialize collection
        self.collection_name = "documents"
        self._ensure_collection_exists()

    async def init_db(self):
        """Initialize the database connection pool."""
        if not self._db_initialized:
            self.db_pool = await asyncpg.create_pool(NEON_DB_URL)
            self._db_initialized = True

            # Create tables if they don't exist
            async with self.db_pool.acquire() as conn:
                # Documents metadata table
                await conn.execute('''
                    CREATE TABLE IF NOT EXISTS documents (
                        id SERIAL PRIMARY KEY,
                        url TEXT UNIQUE,
                        title TEXT,
                        content_type TEXT,
                        source_type TEXT,
                        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                    )
                ''')

                # User highlights table
                await conn.execute('''
                    CREATE TABLE IF NOT EXISTS user_highlights (
                        id SERIAL PRIMARY KEY,
                        document_url TEXT REFERENCES documents(url),
                        highlight_text TEXT,
                        user_id TEXT,
                        session_id TEXT,
                        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                    )
                ''')

                # Create indexes for better performance
                await conn.execute('CREATE INDEX IF NOT EXISTS idx_documents_url ON documents(url)')
                await conn.execute('CREATE INDEX IF NOT EXISTS idx_highlights_doc ON user_highlights(document_url)')
                await conn.execute('CREATE INDEX IF NOT EXISTS idx_highlights_session ON user_highlights(session_id)')
    
    def _ensure_collection_exists(self):
        """Ensure the Qdrant collection exists with the proper configuration."""
        try:
            # Check if collection exists
            self.qdrant_client.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' already exists")
        except:
            # Create collection if it doesn't exist
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
            )
            logger.info(f"Created collection '{self.collection_name}'")
    
    async def process_document(self, document_url: str, source_type: str, job_id: str):
        """Process a document based on its source type."""
        try:
            # Initialize database if not already done
            await self.init_db()

            # Update job status to processing
            await self.update_job_status(job_id, "processing")

            if source_type == "url":
                text_content = await self._extract_text_from_url(document_url)
            elif source_type in ["pdf", "markdown", "md"]:
                text_content = await self._extract_text_from_file(document_url, source_type)
            else:
                raise ValueError(f"Unsupported source type: {source_type}")

            # Store document metadata in database
            async with self.db_pool.acquire() as conn:
                await conn.execute('''
                    INSERT INTO documents (url, content_type, source_type)
                    VALUES ($1, $2, $3)
                    ON CONFLICT (url) DO UPDATE SET
                        content_type = EXCLUDED.content_type,
                        source_type = EXCLUDED.source_type
                ''', document_url, 'text/plain', source_type)

            # Split text into chunks
            chunks = self._split_text_into_chunks(text_content)

            # Create embeddings and store in Qdrant
            await self._store_chunks_in_qdrant(chunks, document_url, source_type)

            # Update job status to completed
            await self.update_job_status(job_id, "completed")
            logger.info(f"Successfully processed document for job_id: {job_id}")

        except Exception as e:
            logger.error(f"Error processing document for job_id {job_id}: {str(e)}")
            await self.update_job_status(job_id, f"failed: {str(e)}")
    
    async def _extract_text_from_url(self, url: str) -> str:
        """Extract text content from a URL."""
        async with aiohttp.ClientSession() as session:
            async with session.get(url) as response:
                content_type = response.headers.get('content-type', '').lower()
                
                if 'application/pdf' in content_type:
                    # Handle PDF content from URL
                    pdf_content = await response.read()
                    return self._extract_text_from_pdf_content(pdf_content)
                elif 'text/html' in content_type:
                    # Handle HTML content
                    html_content = await response.text()
                    soup = BeautifulSoup(html_content, 'html.parser')
                    return soup.get_text(separator=' ', strip=True)
                else:
                    # Default to text extraction
                    text_content = await response.text()
                    return text_content
    
    async def _extract_text_from_file(self, file_url: str, file_type: str) -> str:
        """Extract text from a file (PDF, Markdown)."""
        # For now, we'll assume file_url is accessible and we can download it
        async with aiohttp.ClientSession() as session:
            async with session.get(file_url) as response:
                if file_type.lower() in ['pdf']:
                    content = await response.read()
                    return self._extract_text_from_pdf_content(content)
                elif file_type.lower() in ['md', 'markdown']:
                    content = await response.text()
                    # Convert markdown to text
                    html = markdown.markdown(content)
                    soup = BeautifulSoup(html, 'html.parser')
                    return soup.get_text(separator=' ', strip=True)
        
        return ""
    
    def _extract_text_from_pdf_content(self, pdf_content: bytes) -> str:
        """Extract text from PDF content."""
        from io import BytesIO
        pdf_file = BytesIO(pdf_content)
        pdf_reader = PyPDF2.PdfReader(pdf_file)
        text = ""
        for page in pdf_reader.pages:
            text += page.extract_text() + "\n"
        return text
    
    def _split_text_into_chunks(self, text: str, chunk_size: int = 1000, overlap: int = 200) -> List[Dict[str, Any]]:
        """Split text into chunks of specified size with overlap."""
        if not text:
            return []
        
        # Use tiktoken to count tokens more accurately
        encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
        tokens = encoding.encode(text)
        
        chunks = []
        start_idx = 0
        step_size = chunk_size - overlap
        
        while start_idx < len(tokens):
            end_idx = min(start_idx + chunk_size, len(tokens))
            chunk_tokens = tokens[start_idx:end_idx]
            chunk_text = encoding.decode(chunk_tokens)
            
            chunks.append({
                "text": chunk_text,
                "start_idx": start_idx,
                "end_idx": end_idx
            })
            
            start_idx += step_size
        
        return chunks
    
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
    
    async def _store_chunks_in_qdrant(self, chunks: List[Dict[str, Any]], source_url: str, source_type: str):
        """Store text chunks in Qdrant vector database."""
        points = []
        
        for i, chunk in enumerate(chunks):
            # Get embedding for the chunk
            embedding = (await self._get_embeddings([chunk["text"]]))[0]
            
            point = models.PointStruct(
                id=i,
                vector=embedding,
                payload={
                    "text": chunk["text"],
                    "source_url": source_url,
                    "source_type": source_type,
                    "chunk_id": i,
                    "start_idx": chunk["start_idx"],
                    "end_idx": chunk["end_idx"],
                    "created_at": str(datetime.now())
                }
            )
            points.append(point)
        
        # Upload points to Qdrant
        self.qdrant_client.upsert(
            collection_name=self.collection_name,
            points=points
        )
    
    async def update_job_status(self, job_id: str, status: str):
        """Update the status of an ingestion job."""
        # In a real implementation, this would store the status in a database
        # For now, we'll use a simple in-memory solution for demo purposes
        if not hasattr(self, '_job_status'):
            self._job_status = {}
        self._job_status[job_id] = status
    
    async def get_job_status(self, job_id: str) -> Optional[str]:
        """Get the status of an ingestion job."""
        if hasattr(self, '_job_status'):
            return self._job_status.get(job_id)
        return None