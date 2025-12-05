# RAG API

FastAPI backend for the Physical AI & Humanoid Robotics textbook RAG chatbot.

## Setup

1. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   # Or with uv:
   uv pip install -r requirements.txt
   ```

2. **Configure environment variables**:
   Create a `.env` file in the `rag-api` directory:
   ```env
   # OpenRouter Configuration (Primary LLM)
   OPENROUTER_API_KEY=your_openrouter_api_key
   OPENROUTER_MODEL=openai/gpt-4o-mini
   OPENROUTER_REFERRER=https://github.com/your-username/hackathon_2_claude
   
   # Optional: OpenAI API key for embeddings (if not using OpenRouter for embeddings)
   OPENAI_API_KEY=your_openai_key_for_embeddings
   EMBEDDING_MODEL=text-embedding-3-small
   USE_OPENROUTER_EMBEDDINGS=false
   
   # Qdrant Configuration
   QDRANT_URL=https://your-cluster.qdrant.io
   QDRANT_API_KEY=your_qdrant_api_key
   
   # Neon Database
   NEON_DATABASE_URL=postgresql://user:password@host/database?sslmode=require
   
   # CORS
   CORS_ORIGINS=http://localhost:3000
   RATE_LIMIT_PER_MINUTE=60
   ADMIN_API_KEY=your_admin_key_optional
   ```

   **Note**: The `.env` file should be in the `rag-api/` directory. The application will automatically load it.

3. **Run the server**:
   ```bash
   # Option 1: Using uvicorn directly
   uvicorn app.main:app --reload
   
   # Option 2: Using the main.py entry point
   python main.py
   
   # Option 3: Using Python module
   python -m app.main
   ```

   The server will start at `http://localhost:8000`

## OpenRouter Configuration

This API uses **OpenRouter** instead of direct OpenAI API. OpenRouter provides access to multiple LLM providers including OpenAI models.

### Getting OpenRouter API Key

1. Sign up at [https://openrouter.ai](https://openrouter.ai)
2. Get your API key from the dashboard
3. Add credits to your account
4. Set `OPENROUTER_API_KEY` in your `.env` file

### Model Selection

The default model is `openai/gpt-4o-mini` which provides GPT-4o mini via OpenRouter. You can change this by setting `OPENROUTER_MODEL`:

- `openai/gpt-4o-mini` - GPT-4o mini (default, cost-effective)
- `openai/gpt-4o` - GPT-4o (more capable)
- `openai/gpt-4-turbo` - GPT-4 Turbo
- `anthropic/claude-3.5-sonnet` - Claude 3.5 Sonnet
- See [OpenRouter Models](https://openrouter.ai/models) for full list

### Embeddings

By default, embeddings use OpenAI's `text-embedding-3-small` model. You have two options:

1. **Use OpenAI for embeddings** (recommended):
   - Set `OPENAI_API_KEY` in `.env`
   - Keep `USE_OPENROUTER_EMBEDDINGS=false`

2. **Use OpenRouter for embeddings**:
   - Set `USE_OPENROUTER_EMBEDDINGS=true`
   - Set `EMBEDDING_MODEL` to an OpenRouter-supported embedding model

## API Endpoints

### `GET /health`
Health check endpoint. Returns status of database, Qdrant, and OpenRouter connections.

**Response**:
```json
{
  "status": "ok",
  "database": "ok",
  "qdrant": "ok",
  "openai": "ok (OpenRouter: openai/gpt-4o-mini)"
}
```

### `POST /chat`
Full corpus Q&A endpoint with tool-driven pattern.

**Request**:
```json
{
  "query": "What is ROS 2?",
  "user_id": "optional-user-id"
}
```

**Response**:
```json
{
  "answer": "...",
  "citations": [
    {
      "text": "...",
      "url": "...",
      "section_id": "..."
    }
  ]
}
```

### `POST /chat/selection`
Selected text only mode with selection constraint tool.

**Request**:
```json
{
  "query": "What does this mean?",
  "selection_text": "Selected text from the page...",
  "user_id": "optional-user-id"
}
```

### `POST /ingest`
Trigger document ingestion (admin only). Requires `ADMIN_API_KEY` header.

## Tool-Driven Patterns

The API implements tool-driven patterns as specified:

1. **Retrieval Tool**: Automatically retrieves relevant chunks from Qdrant
2. **Citation Formatter Tool**: Formats citations from retrieved context
3. **Selection Constraint Tool**: Enforces that answers come ONLY from selected text

## Development

- API docs: `http://localhost:8000/docs`
- Alternative docs: `http://localhost:8000/redoc`

## Troubleshooting

### OpenRouter API Key Issues

- Make sure you have credits in your OpenRouter account
- Verify the API key is correct
- Check that the model name is correct (e.g., `openai/gpt-4o-mini`)

### Embeddings Issues

- If using OpenAI for embeddings, ensure `OPENAI_API_KEY` is set
- If using OpenRouter for embeddings, set `USE_OPENROUTER_EMBEDDINGS=true`

### .env file not loading

1. Make sure `.env` is in the `rag-api/` directory
2. Check that `python-dotenv` is installed: `pip install python-dotenv`
3. Verify the file has no syntax errors (no spaces around `=`)
4. Check the console output - it should show where the .env was loaded from

## Deployment

See `infra/README.md` for deployment instructions.
