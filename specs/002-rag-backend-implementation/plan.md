# Humanoid Robotics RAG Backend — Implementation Plan

This plan describes the **exact tasks** required to build the backend RAG system using:
- Python 3.11
- FastAPI
- Ingest
- Qdrant (remote)
- OpenAI (embeddings + GPT)
- Markdown loader + chunker
- GitHub Actions ingestion trigger

All tasks are organized into logical phases with clear deliverables.

---

# 1. Project Initialization

## 1.1 Create project structure
Create folder tree:

```
rag-api/
  main.py
  settings.py
  routers/
  ingestion/
  services/
  utils/
  models/
  requirements.txt
```

## 1.2 Add dependencies
Add required packages:

- fastapi  
- uvicorn  
- ingest  
- qdrant-client  
- openai  
- pydantic  
- python-dotenv  
- tiktoken  
- markdown / mistune  
- anyio  
- httpx  

---

# 2. Configuration Layer

## 2.1 Implement `settings.py`
Contains:
- Qdrant URL + API key
- OpenAI key
- Ingest app ID
- Allowed GitHub webhook secret
- Project constants (chunk sizes, overlap)

## 2.2 Load `.env` variables
Use `python-dotenv`.

## 2.3 Implement a global settings object
Simplify config access across project.

---

# 3. Ingest Client + FastAPI App

## 3.1 Initialize Ingest client
- Assign app ID: `humanoid_rag_app`
- Set `is_production = False`
- Use PydanticSerializer

## 3.2 Integrate Ingest with FastAPI
In `main.py`:
- create FastAPI instance
- mount ingest router
- include other routers

Deliverable: `main.py` boots API + ingest sidecar endpoints.

---

# 4. Markdown Reading + Parsing

## 4.1 Implement `markdown_reader.py`
Responsibilities:
- Recursively glob `/book/docs/**/*.md`
- Read UTF-8 content
- Return list of MarkdownDocument objects:
  - file_path
  - content

## 4.2 Implement header extraction
Parse markdown structure using:
- regex for headers (`#`, `##`, etc.)
- extract section-to-text mapping

Deliverable: Markdown parser that keeps structure for metadata.

---

# 5. Chunking System

## 5.1 Implement `chunker.py`
Responsibilities:
- Split by headers first
- Subsplit into token-limited chunks (800–1000 tokens)
- Add 100-token overlap
- Produce Chunk objects:
  - chunk_text
  - file_path
  - header
  - chunk_index

## 5.2 Token counting
Use tiktoken for precise chunking.

Deliverable: Deterministic markdown-aware chunker.

---

# 6. Embedding Service

## 6.1 Implement `embedding.py`
Functions:
- `embed_texts(list[str]) -> list[list[float]]`
- `embed_query(str) -> list[float]`

Should:
- batch requests
- use OpenAI `text-embedding-3-large`
- handle errors + retries

Deliverable: Fully functional embedding utility.

---

# 7. LLM Service

## 7.1 Implement `llm.py`
Functions:
- `generate_answer(question, context_chunks) -> answer`
- `generate_highlight_answer(question, selected_text) -> answer`

Should:
- Build RAG prompt template
- Use `gpt-4o-mini` or equivalent
- Enforce "no hallucination" policies

Deliverable: Central GPT wrapper.

---

# 8. Qdrant Service Layer

## 8.1 Implement `qdrant_client.py`
Responsibilities:
- Connect to remote Qdrant using URL + API key
- Create collection if not exists (`humanoid_book`)
- Upsert points
- Search using cosine similarity
- Return metadata in structured objects

## 8.2 Qdrant model definitions
Use:
- vector size 3072
- payload: { file_path, header, chunk, chunk_index }

Deliverable: Full Qdrant integration.

---

# 9. Ingestion Pipeline

## 9.1 Implement `pipeline.py`
Orchestrates:
```text
read markdown → parse → chunk → embed → upsert
```

## 9.2 Steps inside Ingest workflow
Define function `ingest_book()` using:

- Step 1: `load_documents`
- Step 2: `chunk_documents`
- Step 3: `embed_documents`
- Step 4: `upsert_documents`

Return summary.

Deliverable: End-to-end ingestion pipeline callable by Ingest.

---

# 10. API Routers

## 10.1 `/webhook/github`
Responsibilities:
- Validate shared secret from GitHub Actions
- Trigger `ingest_book()` using Ingest event
- Return `{ "message": "ingestion triggered" }`

## 10.2 `/ingest_book`
Manual trigger endpoint (optional).

## 10.3 `/query`
Flow:
1. Validate input  
2. Embed question  
3. Query Qdrant  
4. Build prompt  
5. Call LLM  
6. Return answer + sources

## 10.4 `/highlight_query`
Flow:
1. Validate input  
2. Use `selected_text` directly  
3. Call LLM (restricted prompt)  
4. Return answer

Deliverable: A fully structured `routers/` directory.

---

# 11. Data Models

## 11.1 `rag_models.py`
Pydantic models for:

- MarkdownDocument
- Chunk
- EmbeddingResult
- SearchResult
- RAGQueryRequest
- HighlightQueryRequest
- RAGQueryResponse
- HighlightQueryResponse
- IngestResponse

Deliverable: Clean, typed data schemas.

---

# 12. Middleware + Utilities

## 12.1 CORS configuration
Allow Docusaurus frontend domain.

## 12.2 Logging
Log:
- ingestion events
- embedding calls
- Qdrant queries
- errors & retries

---

# 13. GitHub Actions Integration

## 13.1 Add workflow file `.github/workflows/ingest.yml`

Example:

```yaml
on:
  push:
    paths:
      - 'book/docs/**/*.md'

jobs:
  notify-backend:
    runs-on: ubuntu-latest
    steps:
      - name: Call backend ingestion webhook
        run: |
          curl -X POST https://your-backend-url/webhook/github \
          -H "Authorization: Bearer ${{ secrets.RAG_WEBHOOK_SECRET }}" \
          -H "Content-Type: application/json" \
          -d '{ "branch": "${{ github.ref }}", "commit": "${{ github.sha }}" }'
```

Deliverable: Auto-ingestion trigger on every docs update.

---

# 14. Testing Tasks

## 14.1 Unit tests
- Markdown parser
- Chunker
- Qdrant search
- Embedding service

## 14.2 Integration tests
- Full ingestion run on test Markdown set
- Query endpoint produces expected answer

## 14.3 Manual testing in Ingest UI
- Verify each step
- Check retries + logging

---

# 15. Finalization

## 15.1 Deployment configuration
- Uvicorn/Gunicorn server runner
- Environment variable template `.env.example`

## 15.2 README documentation
Must include:
- Setup instructions
- Running local server
- Triggering ingestion manually
- Example curl queries

---

# Summary of Deliverables

✔️ Full project structure  
✔️ Markdown loader  
✔️ Markdown-aware chunker  
✔️ OpenAI embedding + LLM service  
✔️ Qdrant client  
✔️ Ingest ingestion workflow  
✔️ FastAPI endpoints (query, highlight, webhook, ingest)  
✔️ GitHub Actions integration  
✔️ Complete Pydantic models  
✔️ Logging, error handling, CORS  
✔️ Deployment-ready application  

---

# END OF PLAN
