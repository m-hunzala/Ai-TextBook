import asyncio
from utils import extract_docusaurus_docs
from chunker import chunk_documents
from vector_store import QdrantVectorStore
from typing import List, Dict

class IngestionPipeline:
    def __init__(self):
        self.vector_store = QdrantVectorStore()
    
    async def run_ingestion_pipeline(self) -> int:
        """
        Run the complete ingestion pipeline:
        1. Extract documents from Docusaurus markdown files
        2. Chunk the documents
        3. Generate embeddings
        4. Store in Qdrant
        """
        print("Starting document ingestion pipeline...")
        
        # Step 1: Extract documents
        print("Extracting documents...")
        documents = extract_docusaurus_docs()
        print(f"Found {len(documents)} documents")
        
        if not documents:
            print("No documents found to process")
            return 0
        
        # Step 2: Chunk documents
        print("Chunking documents...")
        chunks = chunk_documents(documents)
        print(f"Created {len(chunks)} chunks")
        
        # Step 3: Store in Qdrant
        print("Storing embeddings in Qdrant...")
        processed_count = await self.vector_store.upsert_vectors(chunks)
        
        print(f"Ingestion pipeline completed. Processed {processed_count} chunks")
        return processed_count

# Function to run the pipeline from command line if needed
async def run_ingestion():
    pipeline = IngestionPipeline()
    await pipeline.run_ingestion_pipeline()

if __name__ == "__main__":
    asyncio.run(run_ingestion())