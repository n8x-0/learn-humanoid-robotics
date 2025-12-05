"""
Entry point for running the RAG API server.
This file allows running: python -m rag-api.main
Or: python rag-api/main.py
"""

import uvicorn

if __name__ == "__main__":
    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )
    