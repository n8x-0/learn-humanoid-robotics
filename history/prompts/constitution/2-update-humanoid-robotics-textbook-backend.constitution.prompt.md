---
id: 1
title: Physical AI & Humanoid Robotics Backend For RAG Chatbot Constitution
stage: constitution
date: 2025-12-09
surface: agent
model: gemini
feature: none
branch: 002-rag-backend-implementation
user: user
command: /sp.constitution
labels: ["governance", "constitution"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Mission
Build a production-grade Retrieval-Augmented Generation (RAG) backend for a humanoid robotics book built with Docusaurus. The backend must ingest all Markdown files, embed them, store embeddings in Qdrant, and provide three RAG endpoints to be consumed by a React-based chatbot inside the Docusaurus site.

## Core Principles
1. Accuracy — responses must come strictly from the humanoid robotics book.
2. Safety — no hallucinations; cite retrieved sources.
3. Modularity — separate ingestion, querying, and Qdrant client code.
4. Observability — all costly or fragile operations must be orchestrated through Ingest steps.
5. Maintainability — code must be clean, typed, documented, and follow FastAPI + Pydantic standards.
6. Automation — ingestion must be triggered by GitHub Actions when Markdown files change.

## Technologies Required
- Python 3.11  
- FastAPI  
- Ingest (Python SDK)  
- Qdrant (remote persistent instance)  
- OpenAI embeddings + GPT model  
- Markdown loader + markdown-aware chunking  
- GitHub Actions webhook trigger  
- Pydantic v2  
- Async Python everywhere possible  

## Constraints
- No local Docker Qdrant instance; use an existing remote Qdrant URL & API key.
- Automatic ingestion IS NOT local file watching; it must be called via GitHub Actions webhook.
- Ingestion must re-embed and upsert all documents into Qdrant.
- Frontend exists inside `/book` and must NOT be modified by backend tasks.
- Markdown lives at: `/book/docs/**/*.md`.

## Output Standards
- All code must be working, runnable, and structured into a clean folder layout.
- All endpoints must validate input and output with Pydantic models.
- Components must be separated:
    - `ingestion/`
    - `services/qdrant_client.py`
    - `services/embedding.py`
    - `routers/`
    - `models/`
    - `main.py`
- Logging and error handling must be detailed.
- The backend must be ready for deployment (Gunicorn or Uvicorn workers)."

## Helpful tool
- Use context7 mcp server that is given to you to get the latest docs for technical knowledge and implementation