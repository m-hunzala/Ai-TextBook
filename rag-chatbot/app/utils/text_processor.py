import uuid
from typing import List, Dict
from app.utils.embeddings import get_embeddings
import asyncio

async def process_book_text(text: str, chunk_size: int = 500) -> List[Dict]:
    """
    Process book text by splitting into chunks and generating embeddings
    """
    # Split text into chunks
    chunks = []
    paragraphs = text.split('\n\n')  # Split by paragraphs first
    
    current_chunk = ""
    chunk_index = 0
    
    for paragraph in paragraphs:
        # If adding this paragraph would exceed chunk size, start a new chunk
        if len(current_chunk) + len(paragraph) > chunk_size and current_chunk:
            chunks.append({
                'id': str(uuid.uuid4()),
                'text': current_chunk.strip(),
                'chunk_index': chunk_index
            })
            current_chunk = paragraph
            chunk_index += 1
        else:
            if current_chunk:
                current_chunk += "\n\n" + paragraph
            else:
                current_chunk = paragraph
    
    # Add the last chunk if it has content
    if current_chunk.strip():
        chunks.append({
            'id': str(uuid.uuid4()),
            'text': current_chunk.strip(),
            'chunk_index': chunk_index
        })
    
    # Generate embeddings for all chunks at once
    texts = [chunk['text'] for chunk in chunks]
    embeddings = await get_embeddings(texts)
    
    # Add embeddings to chunks
    for i, chunk in enumerate(chunks):
        chunk['embedding'] = embeddings[i]
    
    return chunks

def chunk_text(text: str, chunk_size: int = 500) -> List[str]:
    """
    Simple function to chunk text without embeddings (for testing)
    """
    sentences = text.split('. ')
    chunks = []
    current_chunk = ""
    
    for sentence in sentences:
        sentence = sentence.strip()
        if not sentence:
            continue
            
        if len(current_chunk) + len(sentence) <= chunk_size:
            current_chunk += sentence + ". "
        else:
            if current_chunk.strip():
                chunks.append(current_chunk.strip())
            current_chunk = sentence + ". "
    
    if current_chunk.strip():
        chunks.append(current_chunk.strip())
    
    return chunks