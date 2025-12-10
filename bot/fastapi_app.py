import os
import uuid
import logging
from typing import Optional, List, Dict, Any
from datetime import datetime
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, Depends, BackgroundTasks, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field
from fastapi_limiter import FastAPILimiter
from fastapi_limiter.depends import RateLimiter
import redis.asyncio as redis
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
import tiktoken

from ingest import IngestionService
from chat_service import ChatService

# Initialize the limiter
limiter = Limiter(key_func=get_remote_address)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Environment variables
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
NEON_DB_URL = os.getenv("NEON_DB_URL")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
REDIS_URL = os.getenv("REDIS_URL", "redis://localhost:6379")

if not all([NEON_DB_URL, QDRANT_URL, QDRANT_API_KEY]):
    logger.warning("Required environment variables are missing for production deployment.")

# Models
class DocumentIngest(BaseModel):
    url: Optional[str] = None
    file_url: Optional[str] = None  # For PDF/Markdown file URLs
    source_type: str = Field(..., description="Type of source: 'url', 'pdf', 'markdown'")


class QueryRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=1000)
    highlight_text: Optional[str] = None
    session_id: Optional[str] = None
    document_url: Optional[str] = None


class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=1000)
    session_id: Optional[str] = None
    highlight_text: Optional[str] = None
    document_url: Optional[str] = None


class ChatResponse(BaseModel):
    response: str
    sources: List[Dict[str, Any]]
    session_id: str
    query: str


class IngestionResponse(BaseModel):
    job_id: str
    status: str
    message: str


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Initialize Redis for rate limiting
    redis_connection = redis.from_url(REDIS_URL)
    await FastAPILimiter.init(redis_connection)
    
    # Initialize services
    app.ingestion_service = IngestionService()
    app.chat_service = ChatService()
    
    yield
    
    # Cleanup
    await redis_connection.close()


app = FastAPI(
    title="RAG Chatbot API",
    description="A Retrieval-Augmented Generation chatbot microservice with document ingestion capabilities",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add rate limiting
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)


# Dependency for API key validation
def validate_api_key(request: Request):
    api_key = request.headers.get("X-API-Key")
    if not api_key:
        raise HTTPException(status_code=401, detail="API Key is missing")
    
    # In production, validate against a database of valid API keys
    expected_api_key = os.getenv("EXPECTED_API_KEY")
    if api_key != expected_api_key:
        raise HTTPException(status_code=401, detail="Invalid API Key")


# Routes
@app.get("/health")
async def health_check():
    return {"status": "healthy", "timestamp": datetime.now()}


@app.post("/ingest", response_model=IngestionResponse)
@limiter.limit("10/minute")
async def ingest_documents(
    request: Request,
    document: DocumentIngest,
    background_tasks: BackgroundTasks,
    api_key: str = Depends(validate_api_key)
):
    # Generate job ID for tracking
    job_id = str(uuid.uuid4())
    
    # Validate input
    if not document.url and not document.file_url:
        raise HTTPException(status_code=400, detail="Either URL or File URL must be provided")
    
    if document.url:
        source_type = "url"
    elif document.file_url:
        source_type = document.source_type
    else:
        raise HTTPException(status_code=400, detail="Invalid source provided")
    
    # Start background task for document processing
    background_tasks.add_task(
        app.ingestion_service.process_document,
        document_url=document.url or document.file_url,
        source_type=source_type,
        job_id=job_id
    )
    
    return IngestionResponse(
        job_id=job_id,
        status="processing",
        message=f"Document ingestion started with job_id: {job_id}"
    )


@app.post("/query", response_model=ChatResponse)
@limiter.limit("30/minute")
async def query_documents(
    request: Request,
    query_request: QueryRequest,
    api_key: str = Depends(validate_api_key)
):
    try:
        response = await app.chat_service.query(
            query=query_request.query,
            highlight_text=query_request.highlight_text,
            session_id=query_request.session_id,
            document_url=query_request.document_url
        )

        return ChatResponse(
            response=response["response"],
            sources=response["sources"],
            session_id=response["session_id"],
            query=query_request.query
        )
    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        raise HTTPException(status_code=500, detail="Error processing query")


@app.post("/chat", response_model=ChatResponse)
@limiter.limit("30/minute")
async def chat_with_bot(
    request: Request,
    chat_request: ChatRequest,
    api_key: str = Depends(validate_api_key)
):
    try:
        response = await app.chat_service.chat(
            message=chat_request.message,
            session_id=chat_request.session_id,
            highlight_text=chat_request.highlight_text,
            document_url=chat_request.document_url
        )

        return ChatResponse(
            response=response["response"],
            sources=response["sources"],
            session_id=response["session_id"],
            query=chat_request.message
        )
    except Exception as e:
        logger.error(f"Error processing chat: {str(e)}")
        raise HTTPException(status_code=500, detail="Error processing chat")


@app.get("/ingest/{job_id}")
async def get_ingestion_status(
    request: Request,
    job_id: str,
    api_key: str = Depends(validate_api_key)
):
    status = await app.ingestion_service.get_job_status(job_id)
    if status is None:
        raise HTTPException(status_code=404, detail="Job ID not found")
    
    return {"job_id": job_id, "status": status}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)