"""Ingest router for manual ingestion."""
from fastapi import APIRouter, HTTPException

from models.rag_models import IngestSummary
from ingestion.pipeline import run_ingestion

router = APIRouter(prefix="/ingest_book", tags=["ingest"])


@router.post("", response_model=IngestSummary)
async def ingest_book() -> IngestSummary:
    """
    Manually trigger document ingestion.
    
    Returns:
        IngestSummary with ingestion results
    """
    try:
        summary = run_ingestion()
        return summary
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error during ingestion: {str(e)}")

