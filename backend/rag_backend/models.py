from pydantic import BaseModel
from typing import List, Optional


class QueryRequest(BaseModel):
    query: str
    history: Optional[List[dict]] = []


class SelectedTextQueryRequest(BaseModel):
    selected_text: str
    query: str
    history: Optional[List[dict]] = []


class QueryResponse(BaseModel):
    response: str
    sources: List[dict]


class AddDocumentRequest(BaseModel):
    document_content: str
    document_title: str
    document_source: str


class QueryLog(BaseModel):
    query: str
    user_id: Optional[str] = None
    timestamp: Optional[str] = None
    session_id: Optional[str] = None

class ClickedSource(BaseModel):
    query_id: int
    source_url: str
    source_title: Optional[str] = None
    user_id: Optional[str] = None
    timestamp: Optional[str] = None
    session_id: Optional[str] = None

class SourceClickRequest(BaseModel):
    query_id: int
    source_url: str
    source_title: Optional[str] = None
    user_id: Optional[str] = None
    session_id: Optional[str] = None

class AnalyticsResponse(BaseModel):
    top_queries: list[dict]
    top_sources: list[dict]
    total_queries: int
    total_clicks: int

class SubagentRequest(BaseModel):
    subagent_name: str
    query: str
    context: Optional[dict] = None

class SubagentResponse(BaseModel):
    result: dict

class SubagentListResponse(BaseModel):
    subagents: list[dict]