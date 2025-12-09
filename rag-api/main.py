"""Main FastAPI application."""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import logging

from .settings import settings
from .routers import query_router, highlight_router, webhook_router, ingest_router

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="Humanoid Robotics RAG API",
    description="RAG backend for Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(query_router.router)
app.include_router(highlight_router.router)
app.include_router(webhook_router.router)
app.include_router(ingest_router.router)


@app.get("/health")
async def health():
    """Health check endpoint."""
    return {"status": "OK"}


@app.on_event("startup")
async def startup_event():
    """Log startup information."""
    logger.info("Starting Humanoid Robotics RAG API")
    logger.info(f"Collection: {settings.qdrant_collection_name}")
    logger.info(f"Number of CORS origins configured: {len(settings.cors_origins_list)}")
    
    # Validate required settings
    if not settings.openai_api_key:
        logger.warning("OPENAI_API_KEY not set - embedding and LLM services will fail")
    if not settings.qdrant_url:
        logger.warning("QDRANT_URL not set - Qdrant service will fail")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

