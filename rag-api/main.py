"""Main FastAPI application."""
from fastapi import FastAPI, Request, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
import logging

from settings import settings
from routers import query_router, highlight_router, webhook_router, ingest_router

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class APIKeyMiddleware(BaseHTTPMiddleware):
    """Middleware to validate API key for protected endpoints."""
    
    async def dispatch(self, request: Request, call_next):
        # Skip API key check for health endpoint and docs
        if request.url.path in ["/health", "/docs", "/openapi.json", "/redoc"]:
            return await call_next(request)
        
        # If API key is not configured, allow all requests
        if not settings.api_key:
            return await call_next(request)
        
        # Check for API key in headers
        api_key = request.headers.get("X-API-Key") or request.headers.get("Authorization")
        
        # Handle Bearer token format
        if api_key and api_key.startswith("Bearer "):
            api_key = api_key[7:]
        
        if not api_key or api_key != settings.api_key:
            return JSONResponse(
                status_code=status.HTTP_401_UNAUTHORIZED,
                content={"detail": "Invalid or missing API key. Provide X-API-Key header or Authorization: Bearer <key>"}
            )
        
        return await call_next(request)


# Create FastAPI app
app = FastAPI(
    title="Humanoid Robotics RAG API",
    description="RAG backend for Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# Add API key middleware first (before CORS)
app.add_middleware(APIKeyMiddleware)

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

