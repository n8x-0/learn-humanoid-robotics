"""Query router for RAG queries."""
from fastapi import APIRouter, HTTPException
from typing import List

from models.rag_models import RAGQueryRequest, RAGQueryResponse
from services.embedding import embedding_service
from services.qdrant_client import qdrant_service
from services.llm import llm_service

router = APIRouter(prefix="/query", tags=["query"])


@router.post("", response_model=RAGQueryResponse)
async def query(request: RAGQueryRequest) -> RAGQueryResponse:
    """
    Handle RAG query requests.
    
    Args:
        request: RAGQueryRequest with question and optional top_k
        
    Returns:
        RAGQueryResponse with answer, sources, and chunks_used
    """
    try:
        # Step 1: Embed question
        query_vector = embedding_service.embed_query(request.question)
        
        # Step 2: Search Qdrant
        top_k = request.top_k or 5
        search_results = qdrant_service.search(query_vector, top_k=top_k)
        
        if not search_results:
            return RAGQueryResponse(
                answer="I couldn't find any relevant information in the textbook to answer your question.",
                sources=[],
                chunks_used=0
            )
        
        # Step 3: Build context from search results
        contexts = [result.chunk_text for result in search_results]
        
        # Step 4: Generate answer using LLM
        answer = llm_service.generate_rag_answer(request.question, contexts)
        
        # Step 5: Extract unique sources
        sources = list(set([result.file_path for result in search_results]))
        
        return RAGQueryResponse(
            answer=answer,
            sources=sources,
            chunks_used=len(search_results)
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

