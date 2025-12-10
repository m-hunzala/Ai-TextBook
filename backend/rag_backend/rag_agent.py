import os
import asyncio
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from openai import AsyncOpenAI
from dotenv import load_dotenv
import tiktoken
from langchain.text_splitter import RecursiveCharacterTextSplitter

load_dotenv()

class RAGAgent:
    def __init__(self):
        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_url or not qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

        self.qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            timeout=10
        )

        # Initialize OpenAI client
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")

        self.openai_client = AsyncOpenAI(api_key=openai_api_key)

        # Initialize text splitter
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=600,
            chunk_overlap=100,
            length_function=len,
            separators=["\n\n", "\n", " ", ""]
        )

        # Create collection if it doesn't exist
        self.collection_name = "book_chunks"
        try:
            self.qdrant_client.get_collection(self.collection_name)
        except:
            # Create collection with dense vector configuration
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
            )

        # Get encoding for token counting
        self.encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")

    async def add_document(self, document_content: str, document_title: str, document_source: str):
        """Add a document to the vector store"""
        # Split document into chunks
        chunks = self.text_splitter.split_text(document_content)

        # Prepare points for Qdrant
        points = []
        for i, chunk in enumerate(chunks):
            # Create embedding using OpenAI
            response = await self.openai_client.embeddings.create(
                model="text-embedding-3-small",
                input=chunk
            )
            embedding = response.data[0].embedding

            # Create point
            point = models.PointStruct(
                id=len(points) + i,  # This should be unique, in real implementation use UUID
                vector=embedding,
                payload={
                    "content": chunk,
                    "document_title": document_title,
                    "document_source": document_source,
                    "chunk_id": i
                }
            )
            points.append(point)

        # Upsert points to Qdrant
        if points:
            # In a real implementation, we'd use UUIDs for IDs to ensure uniqueness
            # For now, we'll use a simple approach with a random ID generator
            import uuid
            for point in points:
                point.id = str(uuid.uuid4())

            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )

    async def query(self, query_text: str, history: Optional[List[dict]] = None) -> str:
        """Query the RAG system"""
        # Create embedding for query
        response = await self.openai_client.embeddings.create(
            model="text-embedding-3-small",
            input=query_text
        )
        query_embedding = response.data[0].embedding

        # Search in Qdrant
        search_results = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=5,
            with_payload=True
        )

        # Format context from search results
        context_parts = []
        sources = []
        for result in search_results:
            content = result.payload.get("content", "")
            title = result.payload.get("document_title", "Unknown")
            source = result.payload.get("document_source", "Unknown")

            context_parts.append(content)
            sources.append({
                "title": title,
                "source": source,
                "score": result.score
            })

        context = "\n\n".join(context_parts)

        # Create system message with context
        system_message = f"""You are an AI assistant for a book website. Use the following context to answer the user's question. If the context doesn't contain the information needed, say so.

Context:
{context}

If you use information from the context, please cite the source. Be helpful and provide accurate answers based on the provided context."""

        # Prepare messages for OpenAI
        messages = [{"role": "system", "content": system_message}]

        # Add history if provided
        if history:
            for msg in history:
                messages.append({"role": msg["role"], "content": msg["content"]})

        # Add the current query
        messages.append({"role": "user", "content": query_text})

        # Get response from OpenAI
        completion = await self.openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=messages,
            stream=True
        )

        # Stream response back
        full_response = ""
        async for chunk in completion:
            if chunk.choices and chunk.choices[0].delta.content:
                content = chunk.choices[0].delta.content
                full_response += content

        return full_response

    async def query_with_selected_text(self, selected_text: str, query: str, history: Optional[List[dict]] = None) -> str:
        """Query with selected text (no RAG retrieval)"""
        # Create system message with selected text as context
        system_message = f"""You are an AI assistant for a book website. The user has selected the following text and asked a question about it:

Selected text:
{selected_text}

Answer the user's question based on the selected text and your general knowledge."""

        # Prepare messages for OpenAI
        messages = [{"role": "system", "content": system_message}]

        # Add history if provided
        if history:
            for msg in history:
                messages.append({"role": msg["role"], "content": msg["content"]})

        # Add the current query
        messages.append({"role": "user", "content": query})

        # Get response from OpenAI
        completion = await self.openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=messages,
            stream=True
        )

        # Stream response back
        full_response = ""
        async for chunk in completion:
            if chunk.choices and chunk.choices[0].delta.content:
                content = chunk.choices[0].delta.content
                full_response += content

        return full_response

    def count_tokens(self, text: str) -> int:
        """Count tokens in text"""
        return len(self.encoding.encode(text))