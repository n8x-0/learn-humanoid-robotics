"""Highlight query router for selected text queries."""
from fastapi import APIRouter, HTTPException

from models.rag_models import HighlightQueryRequest, HighlightQueryResponse
from services.llm import llm_service

router = APIRouter(prefix="/highlight_query", tags=["highlight"])


@router.post("", response_model=HighlightQueryResponse)
async def highlight_query(request: HighlightQueryRequest) -> HighlightQueryResponse:
    """
    Handle highlight query requests (selected text only).
    
    Args:
        request: HighlightQueryRequest with question and selected_text
        
    Returns:
        HighlightQueryResponse with answer and source_context
    """
    try:
        # Generate answer using ONLY the selected text
        answer = llm_service.generate_highlight_answer(
            request.question,
            request.selected_text
        )
        
        return HighlightQueryResponse(
            answer=answer,
            source_context=request.selected_text
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing highlight query: {str(e)}")

