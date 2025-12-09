# Humanoid Robotics RAG Backend

Production-grade Retrieval-Augmented Generation (RAG) backend for the Physical AI & Humanoid Robotics textbook.

## Features

- **Document Ingestion**: Automatically indexes all Markdown files from `/book/docs/**/*.md`
- **Vector Storage**: Stores embeddings in Qdrant vector database
- **RAG Queries**: Answer questions using the full textbook corpus with citations
- **Highlight Queries**: Answer questions using only selected text
- **GitHub Integration**: Automatic ingestion triggered by GitHub Actions

## Setup

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Copy `.env.example` to `.env` and fill in your values:

```bash
cp .env.example .env
```

Required variables:
- `OPENAI_API_KEY`: Your OpenAI API key
- `QDRANT_URL`: Your Qdrant instance URL
- `QDRANT_API_KEY`: Your Qdrant API key (if required)

Optional variables:
- `API_KEY`: API key to protect endpoints (if set, all requests must include `X-API-Key` header)
- `GITHUB_WEBHOOK_SECRET`: Secret for GitHub webhook authentication
- `BOOK_DOCS_PATH`: Path to markdown documents (default: `../book/docs`)

### 3. Run the Server

```bash
python main.py
```

Or using uvicorn directly:

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`

## API Endpoints

### Health Check

```bash
GET /health
```

Returns: `{"status": "OK"}`

### RAG Query

```bash
POST /query
Content-Type: application/json

{
  "question": "What is ROS 2?",
  "top_k": 5
}
```

Response:
```json
{
  "answer": "...",
  "sources": ["docs/ros2/intro.md"],
  "chunks_used": 4
}
```

### Highlight Query

```bash
POST /highlight_query
Content-Type: application/json

{
  "question": "What does this mean?",
  "selected_text": "The robot uses an inverted pendulum dynamic model..."
}
```

Response:
```json
{
  "answer": "...",
  "source_context": "The robot uses an inverted pendulum dynamic model..."
}
```

### Manual Ingestion

```bash
POST /ingest_book
```

Triggers manual document ingestion. Returns:
```json
{
  "total_documents": 50,
  "total_chunks": 250,
  "status": "completed"
}
```

### GitHub Webhook

```bash
POST /webhook/github
Authorization: Bearer <your_webhook_secret>
Content-Type: application/json

{
  "branch": "main",
  "commit": "abc123"
}
```

Triggers ingestion when markdown files are updated via GitHub Actions.

## Project Structure

```
rag-api/
├── main.py                 # FastAPI application entry point
├── settings.py             # Configuration and settings
├── requirements.txt        # Python dependencies
├── models/
│   └── rag_models.py      # Pydantic models
├── routers/
│   ├── query_router.py    # RAG query endpoint
│   ├── highlight_router.py # Highlight query endpoint
│   ├── webhook_router.py  # GitHub webhook endpoint
│   └── ingest_router.py   # Manual ingestion endpoint
├── services/
│   ├── embedding.py       # OpenAI embedding service
│   ├── llm.py             # OpenAI LLM service
│   └── qdrant_client.py   # Qdrant client service
├── ingestion/
│   ├── chunker.py         # Markdown chunking logic
│   └── pipeline.py        # Ingestion pipeline
└── utils/
    └── markdown_reader.py  # Markdown file reading utilities
```

## Development

### Running Locally

1. Ensure you have Python 3.11+
2. Install dependencies: `pip install -r requirements.txt`
3. Set up `.env` file with your API keys
4. Run: `python main.py`

### Testing Endpoints

Use curl or any HTTP client:

```bash
# Health check (no API key required)
curl http://localhost:8000/health

# Query (with API key if configured)
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-api-key-here" \
  -d '{"question": "What is ROS 2?"}'

# Or using Authorization header
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your-api-key-here" \
  -d '{"question": "What is ROS 2?"}'

# Highlight query
curl -X POST http://localhost:8000/highlight_query \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-api-key-here" \
  -d '{"question": "Explain this", "selected_text": "ROS 2 is a framework..."}'
```

**Note**: If `API_KEY` is set in your `.env`, all endpoints (except `/health`, `/docs`, `/openapi.json`) require the API key in the `X-API-Key` header or `Authorization: Bearer <key>` header.

## GitHub Actions Integration

To set up automatic ingestion on markdown file changes, create `.github/workflows/ingest.yml`:

```yaml
name: Trigger RAG Ingestion

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
          curl -X POST ${{ secrets.RAG_BACKEND_URL }}/webhook/github \
            -H "Authorization: Bearer ${{ secrets.RAG_WEBHOOK_SECRET }}" \
            -H "Content-Type: application/json" \
            -d '{"branch": "${{ github.ref }}", "commit": "${{ github.sha }}" }'
```

## Notes

- The ingestion process can take several minutes depending on the number of documents
- Embeddings are generated using OpenAI's `text-embedding-3-large` model (3072 dimensions)
- Chunks are limited to ~1000 tokens with 100 token overlap
- The system uses cosine similarity for vector search in Qdrant

