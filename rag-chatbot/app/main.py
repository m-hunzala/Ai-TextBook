from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import asyncio
import uuid
import time
from datetime import datetime
import sys
import os

# Add the current directory to the path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from app.config import settings
from app.db import NeonDB
from app.qdrant_client import QdrantClient as QdrantClientWrapper
from app.agents_new import generate_answer_with_evidence_fallback
from app.ingest import (
    read_pdf, read_text_file, read_from_url, chunk_text,
    get_embedding as ingest_get_embedding, RateLimiter
)

import asyncpg
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models

app = FastAPI(
    title="Book RAG Chatbot API",
    description="API for ingesting and querying book content with RAG",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# In-memory job tracking (in production, use a database)
ingestion_jobs: Dict[str, Dict[str, Any]] = {}

async def run_ingestion_job(job_id: str, file_path: str, book_id: str, chunk_size: int, overlap: int):
    """Background task to run the ingestion pipeline"""
    try:
        ingestion_jobs[job_id]["status"] = "processing"
        ingestion_jobs[job_id]["started_at"] = time.time()
        
        logger = __import__('logging').getLogger(__name__)
        logger.info(f"Starting ingestion job {job_id} for book {book_id}")
        
        # Determine if the input is a URL or file path
        from urllib.parse import urlparse
        is_url = bool(urlparse(file_path).scheme)
        
        # Read the book content
        if is_url:
            text = await read_from_url(file_path)
        else:
            file_path_obj = __import__('pathlib').Path(file_path)
            if not file_path_obj.exists():
                raise FileNotFoundError(f"File does not exist: {file_path}")
                
            if file_path_obj.suffix.lower() == '.pdf':
                text = await read_pdf(file_path)
            else:
                text = await read_text_file(file_path)
        
        # Chunk the text
        chunks = await chunk_text(text, chunk_size, overlap, book_id)
        
        # Initialize rate limiter
        rate_limiter = RateLimiter(rate=3.0)  # 3 requests per second
        logger.info(f"Processing {len(chunks)} chunks for book {book_id}")
        
        # Initialize Qdrant client
        async with QdrantClientWrapper() as qdrant_client:
            # Prepare vectors and metadata for upsert
            vectors = []
            metadata = []
            
            for chunk in chunks:
                embedding = await ingest_get_embedding(chunk['text'], rate_limiter)
                vectors.append(embedding)
                metadata.append({
                    'book_id': chunk['book_id'],
                    'chunk_id': chunk['chunk_id'],
                    'chapter': chunk['chapter'],
                    'page': chunk['page'],
                    'text': chunk['text'],
                    'start_pos': chunk['start_pos'],
                    'end_pos': chunk['end_pos']
                })
            
            # Upsert vectors to Qdrant
            await qdrant_client.upsert_vectors(vectors, metadata)
        
        # Initialize Neon DB and insert metadata
        db = NeonDB()
        await db.connect()
        try:
            await db.bulk_insert_chunk_metadata(chunks)
        finally:
            await db.close()
        
        ingestion_jobs[job_id]["status"] = "completed"
        ingestion_jobs[job_id]["completed_at"] = time.time()
        ingestion_jobs[job_id]["chunks_processed"] = len(chunks)
        
        logger.info(f"Ingestion job {job_id} completed successfully")
        
    except Exception as e:
        logger = __import__('logging').getLogger(__name__)
        logger.error(f"Error in ingestion job {job_id}: {str(e)}")
        ingestion_jobs[job_id]["status"] = "failed"
        ingestion_jobs[job_id]["error"] = str(e)
        ingestion_jobs[job_id]["completed_at"] = time.time()

# Pydantic models
class IngestRequest(BaseModel):
    file_path: str  # Can be file path or URL
    book_id: str
    chunk_size: Optional[int] = settings.CHUNK_SIZE
    overlap: Optional[int] = settings.CHUNK_OVERLAP

class IngestResponse(BaseModel):
    job_id: str
    message: str
    status: str = "processing"

class QueryRequest(BaseModel):
    book_id: str
    question: str
    top_k: Optional[int] = 5

class Source(BaseModel):
    chunk_id: str
    score: float
    snippet: str

class EvidenceItem(BaseModel):
    chunk_id: str
    text_snippet: str

class QueryResponse(BaseModel):
    answer: str
    summary: str
    sources: List[Source]
    evidence: List[EvidenceItem]

class QuerySelectedRequest(BaseModel):
    selected_text: str
    question: str

class QuerySelectedResponse(BaseModel):
    answer: str
    mode: str = "selected_text_only"

class HealthResponse(BaseModel):
    status: str
    timestamp: datetime

@app.post("/ingest", response_model=IngestResponse)
async def ingest(request: IngestRequest, background_tasks: BackgroundTasks):
    """
    Start the ingestion process for a book file/URL
    Returns a job ID to track progress
    """
    try:
        # Generate a unique job ID
        job_id = str(uuid.uuid4())
        
        # Record the job
        ingestion_jobs[job_id] = {
            "status": "pending",
            "file_path": request.file_path,
            "book_id": request.book_id,
            "chunk_size": request.chunk_size,
            "overlap": request.overlap,
            "created_at": time.time()
        }
        
        # Run the ingestion in the background
        background_tasks.add_task(
            run_ingestion_job,
            job_id,
            request.file_path,
            request.book_id,
            request.chunk_size,
            request.overlap
        )
        
        return IngestResponse(
            job_id=job_id,
            message=f"Ingestion started for book {request.book_id}",
            status="processing"
        )
    except Exception as e:
        logger = __import__('logging').getLogger(__name__)
        logger.error(f"Error starting ingestion: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error starting ingestion: {str(e)}")

@app.post("/query", response_model=QueryResponse)
async def query(request: QueryRequest):
    """
    Query the book content using vector search and generate an answer
    Implements RAG flow: embed question -> search Qdrant -> build context -> call LLM
    """
    try:
        # 1. Embed question
        rate_limiter = RateLimiter(rate=3.0)
        q_emb = await ingest_get_embedding(request.question, rate_limiter)
        
        # Initialize Qdrant client
        async with QdrantClientWrapper() as qdrant_client:
            # 2. Qdrant search
            results = await qdrant_client.search_vectors(q_emb, top_k=request.top_k)
            
            # Filter results by book_id
            filtered_results = []
            for r in results:
                if r.payload.get('book_id') == request.book_id:
                    filtered_results.append(r)
            
            # Limit to top_k after filtering
            filtered_results = filtered_results[:request.top_k]
        
        # 3. Build context = join top snippets
        context_chunks = []
        sources = []
        
        for r in filtered_results:
            text = r.payload.get('text', '')
            chunk_id = r.payload.get('chunk_id', '')
            score = r.score if hasattr(r, 'score') else 0.0
            
            chunk_data = {
                'chunk_id': chunk_id,
                'text': text,
                'score': score
            }
            context_chunks.append(chunk_data)
            
            sources.append(Source(
                chunk_id=chunk_id,
                score=score,
                snippet=text[:200] + "..." if len(text) > 200 else text  # Truncate snippet
            ))
        
        # Build context text with chunk IDs (for reference)
        context_text = "\n\n".join([f"[{r.payload.get('chunk_id', '')}] {r.payload.get('text', '')}" for r in filtered_results])
        
        # Use the agents module to generate answer with evidence
        result = generate_answer_with_evidence_fallback(context_chunks, request.question)
        
        # Map evidence from agent result to our expected format
        evidence = [EvidenceItem(chunk_id=item['chunk_id'], text_snippet=item['text_snippet']) 
                   for item in result.get('evidence', [])]
        
        return QueryResponse(
            answer=result.get('answer', ''),
            summary=result.get('summary', ''),
            sources=sources,
            evidence=evidence
        )
    except Exception as e:
        logger = __import__('logging').getLogger(__name__)
        logger.error(f"Error in query: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.post("/query_selected", response_model=QuerySelectedResponse)
async def query_selected(payload: QuerySelectedRequest):
    """
    Query using only the provided selected text as context
    Do NOT call vector DB. Use only the provided selected_text as the context.
    """
    try:
        # Validate input
        sel = payload.selected_text.strip()
        if not sel:
            raise HTTPException(status_code=400, detail="selected_text empty")
        
        # Prepare the system prompt and user message
        SYSTEM_PROMPT = """You are a helpful assistant. ONLY use the text inside the CONTEXT markers to answer. If the answer is not present, reply exactly: "I don't know based on the provided text.""""
        
        import openai
        openai.api_key = settings.OPENAI_API_KEY
        
        resp = await openai.ChatCompletion.acreate(
            model=getattr(settings, 'EMBED_MODEL', 'gpt-4o-mini'),
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": f"CONTEXT:\n{sel}\nENDCONTEXT\n\nQuestion: {payload.question}"}
            ],
            max_tokens=512,
            temperature=0.0
        )
        
        answer = resp["choices"][0]["message"]["content"].strip()
        
        return QuerySelectedResponse(
            answer=answer,
            mode="selected_text_only"
        )
    except Exception as e:
        logger = __import__('logging').getLogger(__name__)
        logger.error(f"Error in query_selected: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.get("/health")
async def health():
    """
    Health check endpoint
    """
    return HealthResponse(
        status="healthy",
        timestamp=datetime.utcnow()
    )

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)