import os
import asyncio
from qdrant_client import QdrantClient
from qdrant_client.http import models
from openai import OpenAI
import markdown
from pathlib import Path
import re

class DocumentIndexer:
    def __init__(self, qdrant_url, qdrant_api_key, openai_api_key):
        self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key, timeout=10)
        self.openai_client = OpenAI(api_key=openai_api_key)
        self.collection_name = "book_content"
        
    def create_collection(self):
        """Create the Qdrant collection if it doesn't exist."""
        try:
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE)  # OpenAI embedding size
            )
            print(f"Collection '{self.collection_name}' created successfully.")
        except Exception as e:
            print(f"Collection might already exist: {e}")
    
    def extract_text_from_markdown(self, file_path):
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
    
    def split_text(self, text, max_length=1000, overlap=100):
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
    
    def get_embedding(self, text):
        """Get embedding for a text using OpenAI API."""
        try:
            response = self.openai_client.embeddings.create(
                input=text,
                model="text-embedding-ada-002"
            )
            return response.data[0].embedding
        except Exception as e:
            print(f"Error getting embedding: {e}")
            return None
    
    def index_document(self, file_path, url):
        """Index a single document by splitting into chunks and adding to Qdrant."""
        print(f"Indexing document: {file_path}")
        
        # Extract text from markdown
        content = self.extract_text_from_markdown(file_path)
        
        # Split content into chunks
        chunks = self.split_text(content)
        
        # Prepare points for insertion
        points = []
        for i, chunk in enumerate(chunks):
            if len(chunk.strip()) < 10:  # Skip very short chunks
                continue
                
            # Get embedding for the chunk
            embedding = self.get_embedding(chunk)
            if embedding is None:
                continue
                
            # Create a point for Qdrant
            point = models.PointStruct(
                id=len(points),  # Will be updated later to avoid conflicts
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
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            print(f"Indexed {len(points)} chunks from {file_path}")
        else:
            print(f"No valid chunks to index from {file_path}")
    
    def index_all_documents(self, docs_dir):
        """Index all markdown documents in the specified directory."""
        self.create_collection()
        
        docs_path = Path(docs_dir)
        markdown_files = list(docs_path.rglob("*.md")) + list(docs_path.rglob("*.mdx"))
        
        print(f"Found {len(markdown_files)} markdown files to index")
        
        for file_path in markdown_files:
            # Convert file path to URL (this is a simplified approach)
            relative_path = file_path.relative_to(docs_path.parent)
            url = f"/docs/{relative_path.with_suffix('').as_posix()}"
            
            try:
                self.index_document(file_path, url)
            except Exception as e:
                print(f"Error indexing {file_path}: {e}")
        
        print("Indexing complete!")

if __name__ == "__main__":
    import sys
    import argparse
    
    parser = argparse.ArgumentParser(description="Index Docusaurus documentation for RAG")
    parser.add_argument("--docs-dir", default="./docs", help="Path to Docusaurus docs directory")
    args = parser.parse_args()
    
    # Initialize with environment variables
    indexer = DocumentIndexer(
        qdrant_url=os.getenv("QDRANT_URL"),
        qdrant_api_key=os.getenv("QDRANT_API_KEY"),
        openai_api_key=os.getenv("OPENAI_API_KEY")
    )
    
    indexer.index_all_documents(args.docs_dir)