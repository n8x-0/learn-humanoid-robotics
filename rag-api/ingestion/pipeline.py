"""Ingestion pipeline for processing markdown documents."""
from typing import List

from utils.markdown_reader import load_markdown_documents, parse_markdown_sections
from ingestion.chunker import chunk_sections
from services.embedding import embedding_service
from services.qdrant_client import qdrant_service
from models.rag_models import IngestSummary, MarkdownDocument, Section, Chunk
from settings import settings


def run_ingestion(base_docs_path: str = None) -> IngestSummary:
    """
    Run the complete ingestion pipeline.
    
    Args:
        base_docs_path: Path to markdown documents (defaults to settings)
        
    Returns:
        IngestSummary with ingestion results
    """
    if base_docs_path is None:
        base_docs_path = settings.book_docs_path
    
    print(f"Starting ingestion from: {base_docs_path}")
    
    # Step 1: Load documents
    print("Step 1: Loading markdown documents...")
    documents = load_markdown_documents(base_docs_path)
    print(f"Loaded {len(documents)} documents")
    
    # Step 2: Parse sections and chunk
    print("Step 2: Parsing sections and chunking...")
    all_chunks: List[Chunk] = []
    for doc in documents:
        sections = parse_markdown_sections(doc)
        chunks = chunk_sections(sections)
        all_chunks.extend(chunks)
    print(f"Created {len(all_chunks)} chunks from {len(documents)} documents")
    
    # Step 3: Embed chunks
    print("Step 3: Embedding chunks...")
    chunk_texts = [chunk.text for chunk in all_chunks]
    embeddings = embedding_service.embed_texts(chunk_texts)
    print(f"Generated {len(embeddings)} embeddings")
    
    # Step 4: Upsert to Qdrant
    print("Step 4: Upserting to Qdrant...")
    qdrant_service.upsert_chunks(all_chunks, embeddings)
    print("Upsert complete")
    
    return IngestSummary(
        total_documents=len(documents),
        total_chunks=len(all_chunks),
        status="completed"
    )

