from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
import logging
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http import models
import uvicorn
import markdown
import re
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(title="AI Book RAG System", version="1.0.0")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class RAGQuery(BaseModel):
    query: str
    highlight_text: Optional[str] = None
    document_url: Optional[str] = None

class RAGResponse(BaseModel):
    response: str
    sources: List[dict] = []

class IndexRequest(BaseModel):
    docs_path: str = "./docs"

# Initialize clients
openai_api_key = os.getenv("OPENAI_API_KEY")
if not openai_api_key:
    logger.warning("OPENAI_API_KEY not set. The RAG functionality will not work properly.")
    openai_client = None
else:
    openai_client = OpenAI(api_key=openai_api_key)

# Initialize Qdrant client for vector search
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
if not qdrant_url or not qdrant_api_key:
    logger.warning("QDRANT_URL or QDRANT_API_KEY not set. The RAG functionality will not work properly.")
    qdrant_client = None
else:
    qdrant_client = QdrantClient(
        url=qdrant_url,  # For Qdrant Cloud
        api_key=qdrant_api_key,
        timeout=10
    )

# Collection name for document embeddings
COLLECTION_NAME = "book_content"

def extract_text_from_markdown(file_path):
    """Extract text content from a markdown file."""
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
    
    # Remove frontmatter
    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)
    
    # Convert markdown to plain text
    html = markdown.markdown(content)
    # Remove HTML tags to get plain text
    plain_text = re.sub(r'<[^>]+>', '', html)
    
    return plain_text.strip()

def split_text(text, max_length=1000, overlap=100):
    """Split text into chunks."""
    chunks = []
    start = 0
    
    while start < len(text):
        end = start + max_length
        chunk = text[start:end]
        
        # If we're not at the end, try to break at a sentence boundary
        if end < len(text):
            # Find the last sentence ending before max_length
            last_period = chunk.rfind('. ')
            last_exclamation = chunk.rfind('! ')
            last_question = chunk.rfind('? ')
            last_space = chunk.rfind(' ')
            
            break_point = max(
                last_period if last_period != -1 else 0,
                last_exclamation if last_exclamation != -1 else 0,
                last_question if last_question != -1 else 0,
                last_space if last_space != -1 else max_length
            )
            
            if break_point > max_length // 2:  # Only break if we're not cutting too short
                chunk = text[start:start + break_point + 1]
                end = start + break_point + 1
        
        chunks.append(chunk.strip())
        start = end - overlap if end < len(text) else end
        
    return chunks

def get_embedding(text):
    """Get embedding for a text using OpenAI API."""
    if not openai_client:
        logger.error("OpenAI client not initialized. Please set OPENAI_API_KEY environment variable.")
        return None
    
    try:
        response = openai_client.embeddings.create(
            input=text,
            model="text-embedding-ada-002"
        )
        return response.data[0].embedding
    except Exception as e:
        logger.error(f"Error getting embedding: {e}")
        return None

@app.get("/")
async def read_root():
    return {"message": "FastAPI backend for AI Book RAG System"}

@app.post("/api/rag", response_model=RAGResponse)
async def rag_endpoint(query: RAGQuery):
    if not openai_client or not qdrant_client:
        error_msg = "OpenAI and/or Qdrant clients not initialized. Please set OPENAI_API_KEY, QDRANT_URL, and QDRANT_API_KEY environment variables."
        logger.error(error_msg)
        raise HTTPException(status_code=500, detail=error_msg)
    
    try:
        # Prepare the query - if highlight_text is provided, use it as context
        search_text = query.highlight_text if query.highlight_text else query.query
        
        # Search in Qdrant for relevant documents
        search_results = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_text=search_text,
            limit=5,  # Retrieve top 5 most relevant results
            with_payload=True
        )
        
        # Prepare context from search results
        context_parts = []
        sources = []
        
        for result in search_results:
            payload = result.payload
            content = payload.get("content", "")
            title = payload.get("title", "")
            url = payload.get("url", "")
            
            context_parts.append(content)
            
            sources.append({
                "title": title,
                "url": url,
                "content": content[:200] + "..." if len(content) > 200 else content,
                "score": result.score
            })
        
        # Combine the context for the LLM
        context = "\n\n".join(context_parts)
        
        # If we have selected text, emphasize it in the prompt
        if query.highlight_text:
            user_prompt = f"""
            The user has selected the following text: "{query.highlight_text}"
            
            Based on this selection and the following documentation context, please answer the user's query: {query.query}
            
            Documentation context:
            {context}
            """
        else:
            user_prompt = f"""
            Based on the following documentation, please answer the user's query: {query.query}
            
            Documentation context:
            {context}
            """
        
        # Generate response using OpenAI
        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",  # You can use gpt-4 if preferred
            messages=[
                {
                    "role": "system", 
                    "content": "You are an AI assistant helping users understand the Physical AI Book content. Provide accurate and helpful information based on the documentation provided. Always be concise and cite relevant sources when possible."
                },
                {
                    "role": "user",
                    "content": user_prompt
                }
            ],
            max_tokens=1000,
            temperature=0.3
        )
        
        # Extract the AI's response
        ai_response = response.choices[0].message.content
        
        return RAGResponse(response=ai_response, sources=sources)
        
    except Exception as e:
        logger.error(f"Error in RAG endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing RAG query: {str(e)}")

@app.post("/api/index")
async def index_documents(index_request: IndexRequest):
    """Index all markdown documents in the specified directory."""
    if not openai_client or not qdrant_client:
        error_msg = "OpenAI and/or Qdrant clients not initialized. Please set OPENAI_API_KEY, QDRANT_URL, and QDRANT_API_KEY environment variables."
        logger.error(error_msg)
        raise HTTPException(status_code=500, detail=error_msg)
    
    try:
        # Create collection if it doesn't exist
        try:
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE)
            )
        except Exception:
            # Collection might already exist
            pass
        
        docs_path = Path(index_request.docs_path)
        if not docs_path.exists():
            raise HTTPException(status_code=404, detail=f"Directory {index_request.docs_path} not found")
        
        markdown_files = list(docs_path.rglob("*.md")) + list(docs_path.rglob("*.mdx"))
        
        indexed_count = 0
        
        for file_path in markdown_files:
            # Convert file path to URL (this is a simplified approach)
            relative_path = Path(file_path).relative_to(Path('.').resolve())
            url = f"/docs/{relative_path.with_suffix('').as_posix()}"
            
            # Extract text from markdown
            content = extract_text_from_markdown(file_path)
            
            # Split content into chunks
            chunks = split_text(content)
            
            # Prepare points for insertion
            points = []
            for i, chunk in enumerate(chunks):
                if len(chunk.strip()) < 10:  # Skip very short chunks
                    continue
                    
                # Get embedding for the chunk
                embedding = get_embedding(chunk)
                if embedding is None:
                    continue
                    
                # Create a point for Qdrant with a unique ID
                point_id = f"{file_path.name}_{i}_{len(points)}"
                point = models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "content": chunk,
                        "title": Path(file_path).stem,
                        "url": url,
                        "source_file": str(file_path)
                    }
                )
                points.append(point)
            
            if points:
                # Add points to collection
                qdrant_client.upsert(
                    collection_name=COLLECTION_NAME,
                    points=points
                )
                indexed_count += len(points)
                logger.info(f"Indexed {len(points)} chunks from {file_path}")
        
        return {"message": f"Successfully indexed {indexed_count} chunks from {len(markdown_files)} files", "indexed_count": indexed_count}
        
    except Exception as e:
        logger.error(f"Error in indexing endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error indexing documents: {str(e)}")

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)