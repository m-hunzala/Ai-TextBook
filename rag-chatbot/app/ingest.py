#!/usr/bin/env python3
"""
Book ingestion script for RAG chatbot
Supports PDF and plain text files with configurable chunking
"""
import argparse
import asyncio
import hashlib
import logging
import os
import re
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple, Dict, Any
from urllib.parse import urlparse
from urllib.request import urlopen

import asyncpg
import openai
import tiktoken
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Set OpenAI API key
openai.api_key = os.getenv('OPENAI_API_KEY')

# Async rate limiting
class RateLimiter:
    def __init__(self, rate: float = 3.0):  # 3 requests per second
        self.rate = rate
        self.last_call = 0.0
        self.lock = asyncio.Lock()
    
    async def wait(self):
        async with self.lock:
            now = time.time()
            time_since_last = now - self.last_call
            if time_since_last < 1.0 / self.rate:
                sleep_time = (1.0 / self.rate) - time_since_last
                await asyncio.sleep(sleep_time)
            self.last_call = time.time()

async def read_pdf(file_path: str) -> str:
    """
    Read text content from a PDF file using pypdf
    """
    try:
        import pypdf
        with open(file_path, 'rb') as file:
            reader = pypdf.PdfReader(file)
            text = ""
            for page_num, page in enumerate(reader.pages):
                text += f"\n--- PAGE {page_num + 1} ---\n"
                text += page.extract_text()
        return text
    except ImportError:
        logger.error("pypdf not found. Install with: pip install pypdf")
        raise
    except Exception as e:
        logger.error(f"Error reading PDF: {e}")
        raise

async def read_text_file(file_path: str) -> str:
    """
    Read text content from a plain text file
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            return file.read()
    except Exception as e:
        logger.error(f"Error reading text file: {e}")
        raise

async def read_from_url(url: str) -> str:
    """
    Read content from a URL (either PDF or text)
    """
    try:
        with urlopen(url) as response:
            content = response.read()
            
        # Check if it's a PDF by looking at the magic bytes
        if content.startswith(b'%PDF'):
            logger.info("Detected PDF content from URL")
            # Write to temporary file and read with PDF reader
            import tempfile
            with tempfile.NamedTemporaryFile(suffix='.pdf', delete=False) as temp_file:
                temp_file.write(content)
                temp_path = temp_file.name
            
            try:
                text = await read_pdf(temp_path)
            finally:
                os.unlink(temp_path)
            return text
        else:
            # Assume it's plain text
            logger.info("Assuming plain text content from URL")
            return content.decode('utf-8')
    except Exception as e:
        logger.error(f"Error reading from URL: {e}")
        raise

async def chunk_text(text: str, chunk_size: int, overlap: int, book_id: str) -> List[Dict[str, Any]]:
    """
    Split text into chunks with overlap
    Returns a list of dictionaries with chunk data
    """
    if chunk_size <= overlap:
        raise ValueError("Chunk size must be greater than overlap")
    
    # Use TikToken encoder to split more intelligently
    encoder = tiktoken.encoding_for_model("gpt-3.5-turbo")
    tokens = encoder.encode(text)
    
    chunks = []
    start_idx = 0
    chunk_id = 0
    
    while start_idx < len(tokens):
        # Calculate end index based on chunk size
        end_idx = start_idx + chunk_size
        
        # Extract tokens and convert back to text
        chunk_tokens = tokens[start_idx:end_idx]
        chunk_text = encoder.decode(chunk_tokens)
        
        # Create chunk metadata
        chunk_data = {
            'book_id': book_id,
            'chunk_id': f"{book_id}_{chunk_id}",
            'text': chunk_text,
            'start_pos': start_idx,
            'end_pos': end_idx
        }
        
        # Determine page number and chapter if available in the text
        page_match = re.search(r'--- PAGE (\d+) ---', chunk_text)
        if page_match:
            chunk_data['page'] = int(page_match.group(1))
        else:
            chunk_data['page'] = -1  # Unknown page
            
        # Try to identify chapters
        chapter_match = re.search(r'(Chapter\s+\d+|CHAPTER\s+\d+|Chapitre\s+\d+)', chunk_text, re.IGNORECASE)
        if chapter_match:
            chunk_data['chapter'] = chapter_match.group(0)
        else:
            chunk_data['chapter'] = 'Unknown'
        
        chunks.append(chunk_data)
        
        # Move start index based on overlap
        start_idx = end_idx - overlap
        chunk_id += 1
        
        # If remaining tokens are less than or equal to overlap, we're done
        if len(tokens) - start_idx <= overlap:
            break
    
    logger.info(f"Created {len(chunks)} chunks from text")
    return chunks

async def get_embedding(text: str, rate_limiter: RateLimiter) -> List[float]:
    """
    Get embedding for text using OpenAI
    """
    await rate_limiter.wait()
    
    try:
        model = os.getenv('EMBED_MODEL', 'text-embedding-ada-002')
        response = await openai.Embedding.acreate(
            input=text,
            model=model
        )
        return response.data[0].embedding
    except Exception as e:
        logger.error(f"Error getting embedding: {e}")
        raise

async def upsert_to_qdrant(qdrant_client: AsyncQdrantClient, chunks: List[Dict], rate_limiter: RateLimiter):
    """
    Upsert chunks to Qdrant with embeddings
    """
    points = []
    
    for i, chunk in enumerate(chunks):
        # Get embedding for the chunk text
        embedding = await get_embedding(chunk['text'], rate_limiter)
        
        # Create a PointStruct for Qdrant
        point = PointStruct(
            id=chunk['chunk_id'],
            vector=embedding,
            payload={
                'book_id': chunk['book_id'],
                'chunk_id': chunk['chunk_id'],
                'chapter': chunk['chapter'],
                'page': chunk['page'],
                'text': chunk['text'],
                'start_pos': chunk['start_pos'],
                'end_pos': chunk['end_pos']
            }
        )
        points.append(point)
        
        if (i + 1) % 10 == 0:  # Progress update every 10 chunks
            logger.info(f"Processed {i + 1}/{len(chunks)} chunks for Qdrant")
    
    # Upsert all points to Qdrant
    collection_name = os.getenv('QDRANT_COLLECTION', 'book_chunks')
    
    try:
        await qdrant_client.upsert(
            collection_name=collection_name,
            points=points,
            wait=True
        )
        logger.info(f"Successfully upserted {len(points)} chunks to Qdrant")
    except Exception as e:
        logger.error(f"Error upserting to Qdrant: {e}")
        raise

async def insert_to_postgres(postgres_pool: asyncpg.Pool, chunks: List[Dict]):
    """
    Insert chunk metadata to Neon Postgres
    """
    query = """
        INSERT INTO book_chunks 
        (book_id, chunk_id, chapter, page, start_pos, end_pos, qdrant_id) 
        VALUES ($1, $2, $3, $4, $5, $6, $7)
        ON CONFLICT (chunk_id) DO UPDATE SET
            chapter = EXCLUDED.chapter,
            page = EXCLUDED.page,
            start_pos = EXCLUDED.start_pos,
            end_pos = EXCLUDED.end_pos
    """
    
    # Prepare data for bulk insert
    values = [
        (
            chunk['book_id'],
            chunk['chunk_id'],
            chunk['chapter'],
            chunk['page'],
            chunk['start_pos'],
            chunk['end_pos'],
            chunk['chunk_id']  # qdrant_id is the same as chunk_id
        )
        for chunk in chunks
    ]
    
    try:
        async with postgres_pool.acquire() as conn:
            await conn.executemany(query, values)
        logger.info(f"Successfully inserted {len(values)} records to Postgres")
    except Exception as e:
        logger.error(f"Error inserting to Postgres: {e}")
        raise

async def ensure_postgres_tables(postgres_pool: asyncpg.Pool):
    """
    Ensure the required tables exist in Postgres
    """
    create_table_query = """
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
    );
    """
    
    try:
        async with postgres_pool.acquire() as conn:
            await conn.execute(create_table_query)
        logger.info("Ensured book_chunks table exists")
    except Exception as e:
        logger.error(f"Error creating Postgres table: {e}")
        raise

async def main():
    parser = argparse.ArgumentParser(description='Ingest a book into the RAG system')
    parser.add_argument('--file', required=True, help='Path to PDF or text file, or URL')
    parser.add_argument('--book-id', required=True, help='Unique identifier for the book')
    parser.add_argument('--chunk-size', type=int, default=int(os.getenv('CHUNK_SIZE', 800)), 
                        help=f'Size of text chunks (default from env: {os.getenv("CHUNK_SIZE", 800)})')
    parser.add_argument('--overlap', type=int, default=int(os.getenv('CHUNK_OVERLAP', 100)), 
                        help=f'Overlap between chunks (default from env: {os.getenv("CHUNK_OVERLAP", 100)})')
    
    args = parser.parse_args()
    
    # Validate required environment variables
    required_env_vars = ['OPENAI_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY', 'NEON_DATABASE_URL']
    missing_env_vars = [var for var in required_env_vars if not os.getenv(var)]
    
    if missing_env_vars:
        logger.error(f"Missing required environment variables: {missing_env_vars}")
        sys.exit(1)
    
    # Determine if the input is a URL or file path
    is_url = bool(urlparse(args.file).scheme)
    
    try:
        # Read the book content
        logger.info(f"Reading book content from: {args.file}")
        if is_url:
            text = await read_from_url(args.file)
        else:
            file_path = Path(args.file)
            if not file_path.exists():
                logger.error(f"File does not exist: {args.file}")
                sys.exit(1)
                
            if file_path.suffix.lower() == '.pdf':
                text = await read_pdf(str(file_path))
            else:
                text = await read_text_file(str(file_path))
        
        logger.info(f"Successfully read book content, length: {len(text)} characters")
        
        # Chunk the text
        logger.info(f"Chunking text with size {args.chunk_size} and overlap {args.overlap}")
        chunks = await chunk_text(text, args.chunk_size, args.overlap, args.book_id)
        
        # Initialize async rate limiter to prevent exceeding OpenAI rate limits
        rate_limiter = RateLimiter(rate=3.0)  # 3 requests per second
        
        # Initialize Qdrant client
        logger.info("Initializing Qdrant client")
        qdrant_client = AsyncQdrantClient(
            url=os.getenv('QDRANT_URL'),
            api_key=os.getenv('QDRANT_API_KEY'),
            timeout=10
        )
        
        # Initialize Postgres pool
        logger.info("Initializing Postgres connection pool")
        postgres_pool = await asyncpg.create_pool(
            dsn=os.getenv('NEON_DATABASE_URL'),
            min_size=1,
            max_size=5
        )
        
        # Ensure tables exist
        await ensure_postgres_tables(postgres_pool)
        
        # Process chunks
        logger.info("Processing chunks...")
        
        # Upsert to Qdrant
        await upsert_to_qdrant(qdrant_client, chunks, rate_limiter)
        
        # Insert metadata to Postgres
        await insert_to_postgres(postgres_pool, chunks)
        
        # Close connections
        await qdrant_client.aclose()
        await postgres_pool.close()
        
        logger.info("Ingestion completed successfully!")
        
    except Exception as e:
        logger.error(f"Error during ingestion: {e}")
        sys.exit(1)

if __name__ == "__main__":
    asyncio.run(main())