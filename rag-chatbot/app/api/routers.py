from fastapi import APIRouter, HTTPException, BackgroundTasks
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import asyncio
import uuid
import time
import sys
import os

# Add the current directory to the path so we can import from ingest.py
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import the ingestion script components
try:
    from ingest import (
        read_pdf, read_text_file, read_from_url, chunk_text,
        get_embedding as ingest_get_embedding, upsert_to_qdrant, insert_to_postgres, RateLimiter
    )
except ImportError:
    # If not available as module, import directly (when running as script)
    import importlib.util
    spec = importlib.util.spec_from_file_location("ingest", "../ingest.py")
    ingest_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ingest_module)

    read_pdf = ingest_module.read_pdf
    read_text_file = ingest_module.read_text_file
    read_from_url = ingest_module.read_from_url
    chunk_text = ingest_module.chunk_text
    ingest_get_embedding = ingest_module.get_embedding
    upsert_to_qdrant = ingest_module.upsert_to_qdrant
    insert_to_postgres = ingest_module.insert_to_postgres
    RateLimiter = ingest_module.RateLimiter

import asyncpg
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
import openai
from app.config import settings
from app.utils.embeddings import get_embeddings
from app.utils.llm import generate_response
from app.database.qdrant_client import QdrantClient
from app.database.postgres_client import PostgresClient

router = APIRouter()

# In-memory job tracking (in production, use a database)
ingestion_jobs: Dict[str, Dict[str, Any]] = {}

async def run_ingestion_job(job_id: str, file_path: str, book_id: str, chunk_size: int, overlap: int):
    """Background task to run the ingestion pipeline"""
    try:
        ingestion_jobs[job_id]["status"] = "processing"
        ingestion_jobs[job_id]["started_at"] = time.time()

        import logging
        logger = logging.getLogger(__name__)
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
        qdrant_client = AsyncQdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=10
        )

        # Initialize Postgres pool
        postgres_pool = await asyncpg.create_pool(
            dsn=settings.effective_database_url,
            min_size=1,
            max_size=5
        )

        # Upsert to Qdrant
        await upsert_to_qdrant(qdrant_client, chunks, rate_limiter)

        # Insert metadata to Postgres
        await insert_to_postgres(postgres_pool, chunks)

        # Close connections
        await qdrant_client.aclose()
        await postgres_pool.close()

        ingestion_jobs[job_id]["status"] = "completed"
        ingestion_jobs[job_id]["completed_at"] = time.time()
        ingestion_jobs[job_id]["chunks_processed"] = len(chunks)

        logger.info(f"Ingestion job {job_id} completed successfully")

    except Exception as e:
        logger = logging.getLogger(__name__)
        logger.error(f"Error in ingestion job {job_id}: {str(e)}")
        ingestion_jobs[job_id]["status"] = "failed"
        ingestion_jobs[job_id]["error"] = str(e)
        ingestion_jobs[job_id]["completed_at"] = time.time()

# Pydantic models for new endpoints
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
    context_source: str = "selected_text_only"

class HealthResponse(BaseModel):
    status: str
    message: str

@router.post("/ingest", response_model=IngestResponse, summary="Start book ingestion process")
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
        import logging
        logger = logging.getLogger(__name__)
        logger.error(f"Error starting ingestion: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error starting ingestion: {str(e)}")

@router.post("/query", response_model=QueryResponse, summary="Query book content with vector search")
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
        qdrant_client = AsyncQdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=10
        )

        # 2. Qdrant search
        collection_name = settings.QDRANT_COLLECTION
        results = await qdrant_client.search(
            collection_name=collection_name,
            query_vector=q_emb,
            query_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="book_id",
                        match=models.MatchValue(value=request.book_id)
                    )
                ]
            ),
            limit=request.top_k
        )

        # 3. Build context = join top snippets
        context_chunks = []
        sources = []

        for r in results:
            text = r.payload.get('text', '')
            chunk_id = r.id if hasattr(r, 'id') else r.payload.get('chunk_id', '')
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

        # Build context text with chunk IDs
        context_text = "\n\n".join([f"[{r.payload.get('chunk_id', '')}] {r.payload.get('text', '')}" for r in results])

        # Use the agents module to generate answer with evidence
        from app.agents import generate_answer_with_evidence
        result = generate_answer_with_evidence(context_chunks, request.question)

        # Close Qdrant client
        await qdrant_client.aclose()

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
        import logging
        logger = logging.getLogger(__name__)
        logger.error(f"Error in query: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@router.post("/query_selected", response_model=QuerySelectedResponse, summary="Query using only selected text")
async def query_selected(request: QuerySelectedRequest):
    """
    Query using only the provided selected text as context
    Do NOT call vector DB. Use only the provided selected_text as the context.
    """
    try:
        # Validate input
        selected_text = request.selected_text.strip()
        if not selected_text:
            raise HTTPException(status_code=400, detail="selected_text empty")

        # Prepare the system prompt and user message
        SYSTEM_PROMPT = """You are a helpful assistant. ONLY use the text inside the CONTEXT markers to answer. If the answer is not present, reply exactly: "I don't know based on the provided text.""""

        # Call OpenAI for completion
        openai.api_key = settings.OPENAI_API_KEY
        response = await openai.ChatCompletion.acreate(
            model=getattr(settings, 'EMBED_MODEL', 'gpt-4o-mini'),
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": f"CONTEXT:\n{selected_text}\nENDCONTEXT\n\nQuestion: {request.question}"}
            ],
            max_tokens=512,
            temperature=0.0
        )

        answer = response.choices[0].message.content.strip()

        return QuerySelectedResponse(
            answer=answer,
            context_source="selected_text_only"
        )
    except Exception as e:
        import logging
        logger = logging.getLogger(__name__)
        logger.error(f"Error in query_selected: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@router.get("/health", response_model=HealthResponse, summary="Health check")
async def health_check():
    """
    Health check endpoint to verify API is running
    """
    return HealthResponse(
        status="healthy",
        message="Book RAG Chatbot API is running"
    )