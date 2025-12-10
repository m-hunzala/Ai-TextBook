from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
from dotenv import load_dotenv
from .database import db
from .models import QueryRequest, QueryResponse, SelectedTextQueryRequest, AddDocumentRequest, SourceClickRequest, AnalyticsResponse, SubagentRequest, SubagentResponse, SubagentListResponse
from .datastore import datastore

# Import subagents
from .subagents import registry

# Import auth
from .auth import auth_router
from .middleware import verify_better_auth_token, get_user_id_from_token
from fastapi import Request

# Import personalization
from .personalization import personalization_router

# Import translation
from .translation import translation_router

# Load environment variables
load_dotenv()

app = FastAPI(title="RAG Backend",
              description="Retrieval Augmented Generation backend with Qdrant and Gemini embeddings",
              version="1.0.0")

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

@app.on_event("startup")
async def startup_event():
    from .database import init_db
    init_db()

# Include auth routes
app.include_router(auth_router)

# Include personalization routes
app.include_router(personalization_router)

# Include translation routes
app.include_router(translation_router)

# Define Pydantic models for request/response
class QueryRequest(BaseModel):
    query: str
    top_k: int = 5
    context_ids: list[str] = None

class QueryResponse(BaseModel):
    passages: list[dict]
    combined_context: str

class AnswerRequest(BaseModel):
    query: str
    selected_text: str = None

class AnswerResponse(BaseModel):
    answer: str
    retrieved_context: list[dict] = []
    trace: dict = {}

class SourceClickResponse(BaseModel):
    status: str

class EmbedUpsertResponse(BaseModel):
    status: str
    documents_processed: int

# Import other modules
from .auth import verify_api_key
from .rag import RAGSystem

# Initialize RAG system
rag_system = RAGSystem()

@app.get("/")
async def root():
    return {"message": "RAG Backend API"}

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest, current_user: dict = Depends(verify_better_auth_token)):
    """
    Query the vector store to retrieve relevant passages
    """
    try:
        # Extract user ID from token for logging
        user_id = current_user.get('user_id')
        session_id = None  # This could come from session management

        # For now, we'll pass the user ID to the RAG system for logging purposes
        # The RAG system already supports user_id parameter
        passages, combined_context = await rag_system.query(
            query=request.query,
            top_k=request.top_k,
            context_ids=request.context_ids
        )
        return QueryResponse(passages=passages, combined_context=combined_context)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/embed-upsert")
async def embed_upsert_endpoint(current_user: dict = Depends(verify_better_auth_token)):
    """
    Run the embedding pipeline for all documents
    """
    try:
        result = await rag_system.embed_and_upsert()
        return EmbedUpsertResponse(status="success", documents_processed=result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/answer", response_model=AnswerResponse)
async def answer_endpoint(request: AnswerRequest, current_user: dict = Depends(verify_better_auth_token)):
    """
    Generate an answer using RAG - either with selected text or full retrieval
    """
    try:
        user_id = current_user.get('user_id')
        answer, retrieved_context, trace = await rag_system.answer(
            query=request.query,
            selected_text=request.selected_text,
            user_id=user_id  # Pass user ID for logging
        )
        return AnswerResponse(answer=answer, retrieved_context=retrieved_context, trace=trace)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/answer-stream")
async def answer_stream_endpoint(request: AnswerRequest, current_user: dict = Depends(verify_better_auth_token)):
    """
    Generate an answer using RAG with streaming response
    """
    from fastapi.responses import StreamingResponse
    import json

    async def generate_stream():
        try:
            async for chunk in rag_system.answer_streaming(
                query=request.query,
                selected_text=request.selected_text,
                user_id=current_user.get('user_id')  # Pass user ID for logging
            ):
                # Yield each chunk as a server-sent event
                yield f"data: {json.dumps(chunk)}\n\n"
        except Exception as e:
            error_chunk = {
                "type": "error",
                "message": str(e)
            }
            yield f"data: {json.dumps(error_chunk)}\n\n"
        finally:
            # Send end event
            yield "data: [DONE]\n\n"

    return StreamingResponse(generate_stream(), media_type="text/event-stream")

@app.post("/log-source-click", response_model=SourceClickResponse)
async def log_source_click_endpoint(request: SourceClickRequest, current_user: dict = Depends(verify_better_auth_token)):
    """
    Log when a user clicks on a source link
    """
    try:
        user_id = current_user.get('user_id')
        await rag_system.log_source_click(
            query_id=request.query_id,
            source_url=request.source_url,
            source_title=request.source_title,
            user_id=user_id,  # Use authenticated user ID
            session_id=request.session_id
        )
        return SourceClickResponse(status="success")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/analytics", response_model=AnalyticsResponse)
async def analytics_endpoint(current_user: dict = Depends(verify_better_auth_token)):
    """
    Fetch analytics data: top queries and sources
    """
    try:
        top_queries = await datastore.get_top_queries()
        top_sources = await datastore.get_top_sources()
        total_queries = await datastore.get_query_count()
        total_clicks = await datastore.get_click_count()

        return AnalyticsResponse(
            top_queries=top_queries,
            top_sources=top_sources,
            total_queries=total_queries,
            total_clicks=total_clicks
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/subagents", response_model=SubagentListResponse)
async def list_subagents(current_user: dict = Depends(verify_better_auth_token)):
    """
    List all available subagents
    """
    try:
        subagent_names = registry.list_subagents()
        subagents_info = []
        for name in subagent_names:
            description = registry.get_subagent_description(name)
            subagents_info.append({
                "name": name,
                "description": description
            })
        return SubagentListResponse(subagents=subagents_info)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/subagent-call", response_model=SubagentResponse)
async def call_subagent(request: SubagentRequest, current_user: dict = Depends(verify_better_auth_token)):
    """
    Call a specific subagent with a query
    """
    try:
        result = await registry.execute_subagent(request.subagent_name, request.query, request.context)
        return SubagentResponse(result=result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# New endpoints for the chatbot widget
@app.post("/api/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest, current_user: dict = Depends(verify_better_auth_token)):
    """Handle normal queries with RAG retrieval"""
    try:
        # Extract user ID from token for logging
        user_id = current_user.get('user_id')

        # For now, we'll use the existing rag_system for the query
        # In a real implementation, this would call a method that returns the response format we need
        answer, retrieved_context, trace = await rag_system.answer(
            query=request.query,
            selected_text=None,
            user_id=user_id
        )
        return QueryResponse(response=answer, sources=[])
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/query-selected", response_model=QueryResponse)
async def query_selected_endpoint(request: SelectedTextQueryRequest, current_user: dict = Depends(verify_better_auth_token)):
    """Handle queries with selected text (no RAG retrieval)"""
    try:
        # Extract user ID from token for logging
        user_id = current_user.get('user_id')

        # Use the selected text as context for the query
        answer, retrieved_context, trace = await rag_system.answer(
            query=request.query,
            selected_text=request.selected_text,
            user_id=user_id
        )
        return QueryResponse(response=answer, sources=[])
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/add-document")
async def add_document_endpoint(request: AddDocumentRequest, current_user: dict = Depends(verify_better_auth_token)):
    """Add a document to the vector store"""
    try:
        # Extract user ID from token for logging
        user_id = current_user.get('user_id')

        # Add document to the RAG system
        # This would call a method in the rag_system to add the document
        # For now, I'll add a placeholder - you'll need to implement this in the rag_system
        await rag_system.embed_and_upsert_single_document(
            content=request.document_content,
            title=request.document_title,
            source=request.document_source
        )
        return {"message": "Document added successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)