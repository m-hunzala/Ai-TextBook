import tiktoken
from typing import List, Dict
import re

def count_tokens(text: str, model_name: str = "gpt-3.5-turbo") -> int:
    """
    Count the number of tokens in a text using tiktoken
    """
    encoding = tiktoken.encoding_for_model(model_name)
    return len(encoding.encode(text))

def chunk_text(content: str, max_tokens: int = 500, overlap_tokens: int = 100, 
               model_name: str = "gpt-3.5-turbo") -> List[Dict[str, str]]:
    """
    Chunk text into overlapping segments based on token count
    """
    encoding = tiktoken.encoding_for_model(model_name)
    tokens = encoding.encode(content)
    
    chunks = []
    start_idx = 0
    
    while start_idx < len(tokens):
        # Determine the end position
        end_idx = start_idx + max_tokens
        
        # If we're at the end, just take the remaining tokens
        if end_idx >= len(tokens):
            end_idx = len(tokens)
        
        # Decode the chunk
        chunk_tokens = tokens[start_idx:end_idx]
        chunk_text = encoding.decode(chunk_tokens)
        
        chunks.append({
            'text': chunk_text,
            'start_token': start_idx,
            'end_token': end_idx,
            'token_count': len(chunk_tokens)
        })
        
        # Move to the next chunk, accounting for overlap
        start_idx = end_idx - overlap_tokens
        
        # If we've reached the end, break
        if end_idx == len(tokens):
            break
    
    return chunks

def chunk_documents(documents: List[Dict], max_tokens: int = 500, 
                   overlap_tokens: int = 100, model_name: str = "gpt-3.5-turbo") -> List[Dict]:
    """
    Chunk a list of documents into overlapping segments
    Each chunk will have metadata from the original document
    """
    all_chunks = []
    
    for doc in documents:
        content = doc.get('content', '')
        if not content.strip():
            continue
            
        chunks = chunk_text(content, max_tokens, overlap_tokens, model_name)
        
        for i, chunk in enumerate(chunks):
            chunk_metadata = {
                'id': f"{doc.get('id', 'unknown')}_{i}",
                'original_id': doc.get('id', ''),
                'title': doc.get('title', ''),
                'source': doc.get('source', ''),
                'tags': doc.get('tags', []),
                'chunk_index': i,
                'total_chunks': len(chunks),
                'start_token': chunk['start_token'],
                'end_token': chunk['end_token'],
                'token_count': chunk['token_count']
            }
            
            all_chunks.append({
                'id': chunk_metadata['id'],
                'text': chunk['text'],
                'metadata': chunk_metadata
            })
    
    return all_chunks