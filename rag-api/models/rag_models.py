"""Pydantic models for RAG API."""
from pydantic import BaseModel
from typing import Optional, List


class MarkdownDocument(BaseModel):
    """Represents a markdown document."""
    file_path: str
    content: str


class Section(BaseModel):
    """Represents a section within a markdown document."""
    header: str
    text: str
    file_path: str
    level: int  # Header level (1-6)


class Chunk(BaseModel):
    """Represents a chunk of text for embedding."""
    file_path: str
    header: str
    chunk_index: int
    text: str


class SearchResult(BaseModel):
    """Represents a search result from Qdrant."""
    score: float
    file_path: str
    header: str
    chunk_text: str
    chunk_index: int


class RAGQueryRequest(BaseModel):
    """Request model for RAG query endpoint."""
    question: str
    top_k: Optional[int] = 5


class RAGQueryResponse(BaseModel):
    """Response model for RAG query endpoint."""
    answer: str
    sources: List[str]
    chunks_used: int


class HighlightQueryRequest(BaseModel):
    """Request model for highlight query endpoint."""
    question: str
    selected_text: str


class HighlightQueryResponse(BaseModel):
    """Response model for highlight query endpoint."""
    answer: str
    source_context: str


class IngestSummary(BaseModel):
    """Summary of ingestion process."""
    total_documents: int
    total_chunks: int
    status: str


class GitHubWebhookRequest(BaseModel):
    """Request model for GitHub webhook."""
    branch: Optional[str] = None
    commit: Optional[str] = None


class GitHubWebhookResponse(BaseModel):
    """Response model for GitHub webhook."""
    message: str

