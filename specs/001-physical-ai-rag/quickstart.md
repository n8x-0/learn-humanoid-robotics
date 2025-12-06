# Quickstart Guide for RAG Backend API

This guide provides instructions to quickly set up and run the RAG Backend API locally.

## Prerequisites

*   Python 3.10+
*   `uv` (or `pip` and `venv`)
*   Access to Qdrant Cloud instance (with API key and URL)
*   Access to Neon Postgres database (with connection string)
*   Git

## 1. Clone the Repository

```bash
git clone https://github.com/your-repo/learn-humanoid-robotics.git
cd learn-humanoid-robotics/rag-api
```

## 2. Set Up Environment Variables

Create a `.env` file in the `rag-api/` directory with the following content:

```ini
QDRANT_URL="YOUR_QDRANT_CLOUD_URL"
QDRANT_API_KEY="YOUR_QDRANT_API_KEY"
NEON_POSTGRES_URL="YOUR_NEON_POSTGRES_CONNECTION_STRING"
GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
# Add any other necessary environment variables (e.g., CORS origins)
CORS_ORIGINS="http://localhost:3000,http://127.0.0.1:3000"
```

Ensure you replace the placeholder values with your actual credentials.

## 3. Install Dependencies

Using `uv` (recommended):

```bash
uv sync
```

Alternatively, using `pip`:

```bash
python -m venv .venv
./.venv/Scripts/activate # On Windows
source ./.venv/bin/activate # On Linux/macOS
pip install -r requirements.txt
```

## 4. Run the FastAPI Application

```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

The API will be accessible at `http://localhost:8000`.

## 5. Test the Health Endpoint

Open your browser or use `curl`:

```bash
curl http://localhost:8000/health
```

You should see a response like: `{"status":"OK"}`.

## 6. Ingest Documents (Admin/Dev Only)

To index documents, you can send a POST request to the `/ingest` endpoint. This is typically done via a script or an admin interface.

Example `curl` command (replace with actual document data):

```bash
curl -X POST http://localhost:8000/ingest \
     -H "Content-Type: application/json" \
     -d 
           "documents": [
             {
               "doc_id": "foundations-week1",
               "title": "Foundations - Week 1",
               "url": "/docs/foundations/week1",
               "content": "Content of week 1 goes here..."
             }
           ]
         }"
```

## 7. Chat with the RAG Bot

Use the `/chat` or `/chat/selection` endpoints to interact with the RAG chatbot.

Example `curl` for full corpus chat:

```bash
curl -X POST http://localhost:8000/chat \
     -H "Content-Type: application/json" \
     -d '{"query": "What is a humanoid robot?"}'
```

