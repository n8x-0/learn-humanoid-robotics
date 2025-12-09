"""Webhook router for GitHub Actions integration."""
from fastapi import APIRouter, HTTPException, Header, Depends, BackgroundTasks
from typing import Optional
import logging

from ..models.rag_models import GitHubWebhookRequest, GitHubWebhookResponse
from ..ingestion.pipeline import run_ingestion
from ..settings import settings

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/webhook", tags=["webhook"])


def verify_github_secret(authorization: Optional[str] = Header(None)) -> bool:
    """
    Verify GitHub webhook secret from Authorization header.
    
    Args:
        authorization: Authorization header value
        
    Returns:
        True if valid, raises HTTPException if invalid
    """
    if not settings.github_webhook_secret:
        # If no secret is configured, allow all requests (for development)
        return True
    
    if not authorization:
        raise HTTPException(status_code=401, detail="Missing Authorization header")
    
    # Extract token from "Bearer <token>" format
    if not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Invalid Authorization format")
    
    token = authorization[7:]  # Remove "Bearer " prefix
    
    if token != settings.github_webhook_secret:
        raise HTTPException(status_code=401, detail="Invalid webhook secret")
    
    return True


def run_ingestion_background():
    """Background task to run ingestion."""
    try:
        logger.info("Starting background ingestion from GitHub webhook")
        summary = run_ingestion()
        logger.info(f"Ingestion completed: {summary}")
    except Exception as e:
        logger.error(f"Error during background ingestion: {e}", exc_info=True)


@router.post("/github", response_model=GitHubWebhookResponse)
async def github_webhook(
    request: GitHubWebhookRequest,
    background_tasks: BackgroundTasks,
    verified: bool = Depends(verify_github_secret)
) -> GitHubWebhookResponse:
    """
    Handle GitHub webhook requests to trigger ingestion.
    
    Args:
        request: GitHubWebhookRequest with branch and commit info
        background_tasks: FastAPI background tasks
        verified: Dependency to verify webhook secret
        
    Returns:
        GitHubWebhookResponse confirming ingestion was triggered
    """
    try:
        logger.info(f"GitHub webhook triggered: branch={request.branch}, commit={request.commit}")
        
        # Trigger ingestion in background
        # In production with Ingest, this would be:
        # ingest_client.trigger("rag.ingest_book", {})
        background_tasks.add_task(run_ingestion_background)
        
        return GitHubWebhookResponse(
            message="Ingestion triggered successfully"
        )
    except Exception as e:
        logger.error(f"Error triggering ingestion: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Error triggering ingestion: {str(e)}")

