# Humanoid Robotics RAG Backend — Technical Specification

## 1. Overview

This project implements a production-grade Retrieval-Augmented Generation (RAG) backend for a humanoid robotics book authored in Docusaurus.

The system:

- Indexes all Markdown files inside `/book/docs/**/*.md`
- Stores chunk embeddings in a remote Qdrant vector database
- Exposes three public FastAPI endpoints:
  - `/query` — general RAG question answering
  - `/highlight_query` — answer based only on selected text
  - `/webhook/github` — GitHub Actions triggers ingestion
- Uses **Ingest** to orchestrate ingestion workflows
- Uses **OpenAI embeddings + GPT model** for embeddings + answers
- Integrates with a React chatbot UI inside the Docusaurus site

The backend code lives inside:

```
rag-api/
```

---

## 2. System Architecture

```
GitHub Actions  ─── triggers ───>  POST /webhook/github
                                       │
                                       ▼
                               ingest_book() workflow
                                       │
                                       ▼
/book/docs/**/*.md → load → chunk → embed → upsert → Qdrant
                                       ▲
                                       │
Frontend (Docusaurus React UI) ─── /query
                                   └── /highlight_query
```

Key Technologies:

- **Python 3.11**
- **FastAPI**
- **Ingest**
- **OpenAI (Embeddings + GPT)**
- **Remote Qdrant instance**
- **Markdown-aware parser + smart chunker**

---

## 3. Functional Requirements

### 3.1 Ingestion Workflow

Triggered by GitHub Actions → `/webhook/github`.

The workflow must:

1. Discover Markdown files at `/book/docs/**/*.md`
2. Parse each file:
   - Raw text  
   - Headers / subheaders  
   - File path metadata  
3. Chunk content:
   - Chunk size ~800–1000 tokens
   - Overlap ~100 tokens
   - Chunk metadata: `file_path`, `header`, `chunk_index`
4. Embed chunks using OpenAI `text-embedding-3-large`
5. Upsert vectors into Qdrant with payload:
   - file_path
   - header
   - chunk_index
   - chunk
6. Entire ingestion must run inside **Ingest** with steps:
   - load_documents
   - chunk_documents
   - embed_documents
   - upsert_chunks

The backend must return a summary:

```json
{
  "total_documents": X,
  "total_chunks": Y,
  "status": "completed"
}
```

---

### 3.2 Query: `/query`

General question answering using RAG:

Input:
```json
{
  "question": "Explain the humanoid gait cycle."
}
```

Steps:
1. Embed question  
2. Query Qdrant (top_k configurable)  
3. Build context block from returned chunks  
4. Build prompt (context + question)  
5. Run GPT model (gpt-4o-mini or equivalent)  
6. Return answer, sources, and #chunks used  

Output:
```json
{
  "answer": "...",
  "sources": [
    "docs/locomotion/gait.md"
  ],
  "chunks_used": 4
}
```

---

### 3.3 Highlight Query: `/highlight_query`

Used when the user selects text in the Docusaurus UI.

Input:
```json
{
  "question": "What does this mean?",
  "selected_text": "The robot uses an inverted pendulum dynamic model..."
}
```

Behavior:
- Do **not** perform vector search
- Treat `selected_text` as the only allowed context
- Build a strict GPT system prompt:  
  “Use ONLY the provided text. Do not add external knowledge.”
- Return answer + original context

Output:
```json
{
  "answer": "...",
  "source_context": "The robot uses an inverted pendulum dynamic model..."
}
```

---

### 3.4 GitHub Webhook: `/webhook/github`

Triggered by GitHub Actions whenever markdown changes in:

```
book/docs/**/*.md
```

Responsibilities:

- Validate secret token  
- Trigger Ingest function: `rag.ingest_book`  
- Return 200 OK immediately  

---

## 4. Qdrant Schema

Collection name: `humanoid_book`

```yaml
vector_size: 3072
distance: cosine
payload_schema:
  file_path: string
  header: string
  chunk: string
  chunk_index: int
```

---

## 5. Ingest Workflow Steps (Detailed)

### Step 1: load_documents
- Recursively list all markdown files under `/book/docs/**/*.md`
- Read contents
- Parse headings and sections
- Store document structures

### Step 2: chunk_documents
- Apply markdown-aware chunker:
  - Split by headers
  - Merge small segments
  - Enforce max token size ~1000
  - Overlap tokens: ~100
- Output `Chunk` objects containing:
  - chunk text
  - metadata
  - chunk index per file

### Step 3: embed_documents
- Pass list of chunk texts into OpenAI embeddings
- Batch embed for efficiency

### Step 4: upsert_chunks
- Create Qdrant points:
  - id (uuid)
  - vector
  - payload
- Upsert to remote Qdrant

---

## 6. API Endpoints (Full List)

### 1. `POST /ingest_book`
Used internally or for manual ingestion.

### 2. `POST /webhook/github`
GitHub → backend
Triggers Ingest → ingestion workflow runs asynchronously.

### 3. `POST /query`
General RAG answering.

### 4. `POST /highlight_query`
Highlights-based answering.

---

## 7. Directories and Files

```
rag-api/
  main.py
  settings.py
  routers/
    ingest_router.py
    query_router.py
    highlight_router.py
    webhook_router.py
  ingestion/
    loader.py
    chunker.py
    pipeline.py
  services/
    qdrant_client.py
    embedding.py
    llm.py
  utils/
    markdown_reader.py
  models/
    rag_models.py
  requirements.txt
```

---

## 8. GitHub Actions Integration Specification

Workflow YAML MUST trigger on markdown changes:

```yaml
on:
  push:
    paths:
      - 'book/docs/**/*.md'
```

Then call:

```
POST https://yourbackend.com/webhook/github
Authorization: Bearer ${{ secrets.RAG_SECRET }}
Content-Type: application/json
```

Payload includes:

```json
{
  "branch": "${{ github.ref }}",
  "commit": "${{ github.sha }}"
}
```

---

## 9. Non-Functional Requirements

### Performance
- Qdrant search under 100ms
- Embeddings batched
- Ingest steps concurrent where possible

### Reliability
- Ingest must retry embedding and upserting on error
- Logs must fully reflect workflow state
- Ingest dashboard must show each step clearly

### Security
- GitHub webhook secret must be validated
- OpenAI API key must be in env
- No raw filesystem exposure

### Maintainability
- Fully modular file structure
- Pydantic models for all input/output types
- Type hints everywhere
- Docstrings required

---

## 10. Acceptance Criteria

- ✓ Markdown ingestion runs automatically via GitHub Actions  
- ✓ All chunks stored in Qdrant with correct metadata  
- ✓ `/query` returns accurate, source-linked answers  
- ✓ `/highlight_query` uses ONLY the selected text  
- ✓ Ingest workflow is visible in Ingest UI with steps  
- ✓ No endpoint fails silently  
- ✓ React UI can reliably call backend  

---

# END OF SPECIFICATION
