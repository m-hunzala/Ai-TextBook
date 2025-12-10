#!/usr/bin/env python3
"""
Script to extract markdown docs, chunk text, embed, and upsert to Qdrant.
This implements the document processing pipeline as specified in the requirements.
"""
import asyncio
import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add the current directory to path if needed
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from ingestion_pipeline import IngestionPipeline

async def main():
    print("Starting document ingestion pipeline...")
    print("This script will:")
    print("1. Extract markdown documents from your Docusaurus site")
    print("2. Chunk the documents (500-token chunks with 100-token overlap)")
    print("3. Generate embeddings using your configured provider (Gemini or OpenAI)")
    print("4. Upsert to Qdrant vector store")
    print("")
    
    pipeline = IngestionPipeline()
    processed_count = await pipeline.run_ingestion_pipeline()
    
    print(f"\nPipeline completed! Processed {processed_count} document chunks.")

if __name__ == "__main__":
    asyncio.run(main())