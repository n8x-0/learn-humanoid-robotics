# Humanoid Robotics RAG Backend — Task Tree (for Speckit)

/*
This file defines the TASKS to implement the RAG backend.

Use it with Speckit (or manually) to drive `/sp.implement`.
Each task is small, explicit, and has clear dependencies.
*/

tasks:

  - id: project_init
    title: Initialize RAG backend project
    description: >
      Create the rag-api project structure, add basic files, and install dependencies.
    subtasks:
      - id: project_init_structure
        title: Create base folder structure
        description: >
          Create the main folders and placeholder files.
        steps:
          - Create `rag-api/`
          - Inside `rag-api/`, create:
            - `main.py`
            - `settings.py`
            - `requirements.txt`
            - `routers/`
            - `ingestion/`
            - `services/`
            - `utils/`
            - `models/`
      - id: project_init_requirements
        title: Define Python dependencies
        description: >
          Fill requirements.txt with all required packages.
        steps:
          - Add core dependencies:
            - fastapi
            - uvicorn[standard]
            - ingest
            - qdrant-client
            - openai
            - pydantic
            - python-dotenv
            - anyio
            - httpx
            - tiktoken
            - markdown or mistune
          - Ensure versions are compatible with Python 3.11
      - id: project_init_env
        title: Setup environment management
        description: >
          Define environment variable structure and .env template.
        steps:
          - Create `.env.example` with:
            - `OPENAI_API_KEY=`
            - `QDRANT_URL=`
            - `QDRANT_API_KEY=`
            - `INGEST_APP_ID=humanoid_rag_app`
            - `GITHUB_WEBHOOK_SECRET=`
          - Document how to copy `.env.example` to `.env` for local dev.

  - id: config_layer
    title: Implement configuration and settings
    description: >
      Centralize environment config and constants for the whole app.
    dependencies: [project_init]
    subtasks:
      - id: settings_module
        title: Implement settings.py
        description: >
          Create a Pydantic-based Settings class and load environment variables.
        steps:
          - Use Pydantic BaseSettings or similar pattern.
          - Define fields:
            - `openai_api_key: str`
            - `qdrant_url: str`
            - `qdrant_api_key: str`
            - `ingest_app_id: str`
            - `github_webhook_secret: str`
            - Chunking constants:
              - `chunk_max_tokens: int = 1000`
              - `chunk_overlap_tokens: int = 100`
              - `qdrant_collection_name: str = "humanoid_book"`
          - Load `.env` via python-dotenv (if not provided by environment).
      - id: config_validation
        title: Validate configuration at startup
        description: >
          Ensure missing critical env variables are detected early.
        steps:
          - In `main.py`, import Settings.
          - Instantiate Settings at startup.
          - If any critical field is missing, log and raise an error.

  - id: ingest_and_fastapi_bootstrap
    title: Setup Ingest client and FastAPI app
    description: >
      Integrate Ingest with FastAPI and expose the internal ingest routes.
    dependencies: [config_layer]
    subtasks:
      - id: ingest_client_setup
        title: Create Ingest client
        description: >
          Configure the Ingest client with the right app ID and serializer.
        steps:
          - Import Ingest in `main.py`.
          - Initialize ingest client:
            - `app_id=settings.ingest_app_id`
            - `logger` using `logging.getLogger("uvicorn")`
            - `is_production=False` for now
            - `serializer=IngestPydanticSerializer` or equivalent
      - id: fastapi_app_factory
        title: Build FastAPI app instance
        description: >
          Configure FastAPI, include routers, and mount Ingest.
        steps:
          - Create `app = FastAPI(...)` in `main.py`.
          - Import routers:
            - `routers.ingest_router`
            - `routers.query_router`
            - `routers.highlight_router`
            - `routers.webhook_router`
          - Include each router with appropriate prefixes.
          - Use `ingest.fastapi.serve` to expose Ingest endpoints at `/api/ingest`.

  - id: markdown_io
    title: Markdown reading and parsing
    description: >
      Implement utilities to read and parse Docusaurus Markdown files from /book/docs/**/*.md.
    dependencies: [project_init]
    subtasks:
      - id: markdown_reader
        title: Implement utils/markdown_reader.py
        description: >
          Read all markdown files recursively and return structured documents.
        steps:
          - Define a `MarkdownDocument` Pydantic model in `models/rag_models.py` with:
            - `file_path: str`
            - `content: str`
          - In `utils/markdown_reader.py`:
            - Implement function `load_markdown_documents(base_path: str) -> list[MarkdownDocument]`
            - Use `glob` or `pathlib` to find `/book/docs/**/*.md` (path must be configurable via settings or parameter).
      - id: markdown_header_parser
        title: Implement header extraction
        description: >
          Extract headers (#, ##, ###) and map them to text regions for metadata.
        steps:
          - Implement a function `parse_markdown_sections(document: MarkdownDocument) -> list[Section]`
          - Define `Section` model in `rag_models.py`:
            - `header: str`
            - `text: str`
          - Use regex or markdown library to split by headings.
          - Ensure section metadata is preserved: file_path + header.

  - id: chunking_system
    title: Chunking markdown into RAG chunks
    description: >
      Create a markdown-aware chunker that respects headers and token limits.
    dependencies: [markdown_io]
    subtasks:
      - id: chunk_model
        title: Define Chunk model
        description: >
          Represent each chunk as a structured object for downstream use.
        steps:
          - In `models/rag_models.py` define:
            - `Chunk` with fields:
              - `file_path: str`
              - `header: str`
              - `chunk_index: int`
              - `text: str`
      - id: token_counter
        title: Implement token counting with tiktoken
        description: >
          Provide a function to count tokens for accurate chunking.
        steps:
          - Add function `count_tokens(text: str) -> int` in `utils/token_utils.py` or inside `chunker.py`.
          - Use tiktoken with a GPT-4 compatible encoding.
      - id: chunker_impl
        title: Implement ingestion/chunker.py
        description: >
          Use sections to create token-bounded overlapping chunks.
        steps:
          - Implement `chunk_sections(sections: list[Section], max_tokens: int, overlap: int) -> list[Chunk]`
          - Algorithm:
            - Iterate over sections in order.
            - Build rolling chunks up to `max_tokens`.
            - Ensure 100-token overlap between consecutive chunks.
            - Set `chunk_index` per file.
          - Return list of `Chunk`.

  - id: embedding_service
    title: Implement OpenAI embedding service
    description: >
      Provide embedding functions for documents and queries.
    dependencies: [config_layer]
    subtasks:
      - id: embedding_module
        title: Create services/embedding.py
        description: >
          Wrap OpenAI embedding API in simple functions.
        steps:
          - Initialize OpenAI client with `settings.openai_api_key`.
          - Implement `embed_texts(texts: list[str]) -> list[list[float]]`:
            - Use model `text-embedding-3-large`.
            - Batch input texts.
          - Implement `embed_query(text: str) -> list[float]` that calls `embed_texts([text])[0]`.
          - Handle errors and retries (simple exponential backoff or Ingest steps later).

  - id: llm_service
    title: Implement GPT-based LLM service
    description: >
      Provide convenience wrappers to generate RAG answers and highlight-based answers.
    dependencies: [config_layer]
    subtasks:
      - id: llm_module
        title: Create services/llm.py
        description: >
          Wrap OpenAI chat/completions for RAG and highlight modes.
        steps:
          - Initialize OpenAI chat client.
          - Implement `generate_rag_answer(question: str, contexts: list[str]) -> str`:
            - Prepare prompt:
              - System: "You are a helpful assistant that answers strictly based on the provided humanoid robotics book context."
              - User: include concatenated context and question.
          - Implement `generate_highlight_answer(question: str, selected_text: str) -> str`:
            - System: "You must ONLY use the provided selected text. If answer is not contained, say you cannot answer."
          - Use `gpt-4o-mini` or similar from OpenAI.

  - id: qdrant_service
    title: Implement Qdrant client and schema
    description: >
      Connect to remote Qdrant, manage collection, and implement search/upsert.
    dependencies: [config_layer]
    subtasks:
      - id: qdrant_client
        title: Create services/qdrant_client.py
        description: >
          Abstract Qdrant operations in a clean class or module.
        steps:
          - Initialize Qdrant client with `settings.qdrant_url` and `settings.qdrant_api_key`.
          - Ensure collection creation:
            - name from `settings.qdrant_collection_name`
            - vector_size=3072
            - distance="cosine"
          - Implement:
            - `upsert_chunks(chunks: list[Chunk], vectors: list[list[float]])`
              - Generate UUIDs for each point.
              - Payload: file_path, header, chunk, chunk_index.
            - `search(query_vector: list[float], top_k: int) -> list[SearchResult]`
              - Define `SearchResult` model in `rag_models.py` including:
                - score
                - file_path
                - header
                - chunk_text
                - chunk_index

  - id: ingestion_pipeline
    title: Implement full ingestion pipeline using Ingest
    description: >
      Wire markdown reading, chunking, embedding, and Qdrant upsert into a single Ingest workflow.
    dependencies: [markdown_io, chunking_system, embedding_service, qdrant_service, ingest_and_fastapi_bootstrap]
    subtasks:
      - id: ingestion_models
        title: Add ingestion response/output models
        description: >
          Define Pydantic models for ingestion results.
        steps:
          - In `rag_models.py` define:
            - `IngestSummary`:
              - `total_documents: int`
              - `total_chunks: int`
              - `status: str`
      - id: ingestion_pipeline_impl
        title: Create ingestion/pipeline.py
        description: >
          Pure Python pipeline functions without Ingest decorator.
        steps:
          - Implement:
            - `run_ingestion(base_docs_path: str) -> IngestSummary`
          - Pipeline:
            - Load documents with `load_markdown_documents`.
            - Parse sections.
            - Chunk sections.
            - Embed chunk texts.
            - Upsert to Qdrant.
            - Return `IngestSummary`.
      - id: ingest_function
        title: Create Ingest function `ingest_book`
        description: >
          Wrap the pipeline in an Ingest function with steps.
        steps:
          - In e.g. `ingestion/ingest_functions.py`:
            - Use `@ingest_client.create_function(...)`.
            - Trigger name: `"rag.ingest_book"`.
            - Inside function, use `ctx.step.run` for:
              - "load_documents"
              - "chunk_documents"
              - "embed_documents"
              - "upsert_documents"
            - Return `IngestSummary` as JSON-serializable object.

  - id: routers_and_endpoints
    title: Implement FastAPI routers and endpoints
    description: >
      Expose all required HTTP endpoints for webhook, ingestion, and querying.
    dependencies: [ingestion_pipeline, qdrant_service, embedding_service, llm_service]
    subtasks:
      - id: webhook_router
        title: Implement /webhook/github endpoint
        description: >
          Receive GitHub webhook and trigger ingestion via Ingest.
        steps:
          - Create `routers/webhook_router.py` with APIRouter.
          - POST `/webhook/github`:
            - Validate `Authorization` header (Bearer secret).
            - Enqueue/trigger Ingest function `ingest_book`.
            - Immediately return `{ "message": "ingestion started" }`.
      - id: ingest_router
        title: Implement /ingest_book manual endpoint
        description: >
          Optional manual ingestion trigger for local testing.
        steps:
          - Create `routers/ingest_router.py`.
          - POST `/ingest_book`:
            - Directly call `run_ingestion()` or trigger Ingest function.
            - Return `IngestSummary`.
      - id: query_router
        title: Implement /query endpoint
        description: >
          General RAG query endpoint.
        steps:
          - Create `RAGQueryRequest` and `RAGQueryResponse` in `rag_models.py`.
          - In `routers/query_router.py`:
            - POST `/query`:
              - Validate payload.
              - Embed question using `embed_query`.
              - Call Qdrant `search`.
              - Collect top chunks texts and metadata.
              - Call `generate_rag_answer`.
              - Return answer + sources + chunks_used.
      - id: highlight_router
        title: Implement /highlight_query endpoint
        description: >
          Answer using only selected_text as context.
        steps:
          - Create `HighlightQueryRequest` and `HighlightQueryResponse` in `rag_models.py`.
          - In `routers/highlight_router.py`:
            - POST `/highlight_query`:
              - Validate payload.
              - Call `generate_highlight_answer(question, selected_text)`.
              - Return answer + source_context.

  - id: cors_logging
    title: Setup CORS and logging
    description: >
      Allow Docusaurus frontend to call backend and ensure good logging.
    dependencies: [ingest_and_fastapi_bootstrap]
    subtasks:
      - id: cors_middleware
        title: Add CORS middleware
        description: >
          Configure allowed origins in main.py.
        steps:
          - Use `fastapi.middleware.cors.CORSMiddleware`.
          - Allow origin(s) of your Docusaurus site.
      - id: logging_config
        title: Configure logging
        description: >
          Ensure logs are emitted for ingestion, queries, and errors.
        steps:
          - Configure root logger with level INFO/DEBUG.
          - Add logs in ingestion steps and query endpoints.

  - id: github_actions
    title: Configure GitHub Actions ingestion workflow
    description: >
      Set up GitHub workflow to POST to /webhook/github when docs change.
    dependencies: [routers_and_endpoints]
    subtasks:
      - id: gha_workflow_file
        title: Create .github/workflows/ingest.yml
        description: >
          Define CI job that sends POST to /webhook/github.
        steps:
          - Trigger on:
            - `push` with paths `book/docs/**/*.md`.
          - Use curl to POST to backend:
            - URL: `https://your-backend-url/webhook/github`
            - Headers:
              - `Authorization: Bearer ${{ secrets.RAG_WEBHOOK_SECRET }}`
              - `Content-Type: application/json`
            - Body:
              - `{ "branch": "${{ github.ref }}", "commit": "${{ github.sha }}" }`
      - id: gha_secrets
        title: Configure GitHub secrets
        description: >
          Add secret to GitHub for webhook auth.
        steps:
          - In repo settings, add `RAG_WEBHOOK_SECRET`.
          - Document secret usage.

  - id: testing_and_docs
    title: Testing and documentation
    description: >
      Add tests, usage docs, and local run instructions.
    dependencies: [routers_and_endpoints, github_actions]
    subtasks:
      - id: unit_tests
        title: Add basic unit tests
        description: >
          Implement small tests for core utilities.
        steps:
          - Test markdown reader.
          - Test chunker behavior.
          - Test token counter.
      - id: integration_tests
        title: Add integration tests
        description: >
          Test ingestion and query behavior end-to-end with a small sample.
        steps:
          - Create few sample markdown files.
          - Run ingestion.
          - Call `/query` and assert answer structure is correct.
      - id: readme
        title: Write README
        description: >
          Document how to run and use the backend.
        steps:
          - Include:
            - Setup
            - Environment variables
            - Running server locally
            - Triggering ingestion manually
            - Example calls for `/query` and `/highlight_query`.

# END OF TASK TREE
