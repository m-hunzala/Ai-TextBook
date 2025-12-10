from pydantic import BaseModel
from typing import Optional

class IngestRequest(BaseModel):
    title: str
    author: str
    text: str
    content_hash: Optional[str] = None

class QueryRequest(BaseModel):
    query: str
    top_k: int = 5  # Number of similar chunks to retrieve

class QuerySelectedRequest(BaseModel):
    query: str
    selected_text: str