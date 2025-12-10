from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
import asyncio
import logging
from contextlib import asynccontextmanager

from rag_backend.rag_agent import RAGAgent
from rag_backend.database import get_db, init_db
from rag_backend.models import QueryRequest, QueryResponse, SelectedTextQueryRequest, AddDocumentRequest

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Initializing database...")
    init_db()
    logger.info("Database initialized")

    # Initialize RAG Agent
    global rag_agent
    rag_agent = RAGAgent()
    logger.info("RAG Agent initialized")

    yield

    # Shutdown
    logger.info("Shutting down...")

app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG-based chatbot for Docusaurus book website",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose headers for streaming
    expose_headers=["*"],
)

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running!"}

@app.post("/api/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """Handle normal queries with RAG retrieval"""
    try:
        response = await rag_agent.query(request.query, request.history)
        return QueryResponse(response=response, sources=[])
    except Exception as e:
        logger.error(f"Error in query endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/query-selected", response_model=QueryResponse)
async def query_selected_endpoint(request: SelectedTextQueryRequest):
    """Handle queries with selected text (no RAG retrieval)"""
    try:
        response = await rag_agent.query_with_selected_text(request.selected_text, request.query, request.history)
        return QueryResponse(response=response, sources=[])
    except Exception as e:
        logger.error(f"Error in query-selected endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/add-document")
async def add_document_endpoint(request: AddDocumentRequest):
    """Add a document to the vector store"""
    try:
        await rag_agent.add_document(request.document_content, request.document_title, request.document_source)
        return {"message": "Document added successfully"}
    except Exception as e:
        logger.error(f"Error in add-document endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)