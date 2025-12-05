# Physical AI & Humanoid Robotics Textbook + RAG Chatbot

A production-grade Docusaurus-based textbook with an embedded RAG chatbot for interactive learning.

## Project Structure

```
/
├── book/                # Docusaurus site
├── rag-api/             # FastAPI backend
├── infra/               # Scripts for Qdrant/Neon setup + ingestion
├── scripts/             # Doc chunking + embedding + upload
└── README.md            # This file
```

## Quick Start

### Prerequisites

- Node.js (v18+)
- Python (3.11+)
- uv (Python package manager)
- npm or yarn

### Local Development

1. **Clone the repository**
   ```bash
   git clone <repo-url>
   cd hackathon_2_claude
   ```

2. **Set up environment variables**
   ```bash
   cp .env.example .env
   # Edit .env with your API keys and configuration
   ```

3. **Start the Docusaurus book**
   ```bash
   cd book
   npm install
   npm start
   ```
   The book will be available at `http://localhost:3000`

4. **Start the RAG API** (in a separate terminal)
   ```bash
   cd rag-api
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   uv pip install -r requirements.txt
   uvicorn app.main:app --reload
   ```
   The API will be available at `http://localhost:8000`

5. **Index documents** (first time setup)
   ```bash
   python scripts/index_docs.py --base-url http://localhost:3000
   ```

## Development Commands

### Book (Docusaurus)
- `npm start` - Start development server
- `npm run build` - Build for production
- `npm run serve` - Serve production build locally

### RAG API (FastAPI)
- `uvicorn app.main:app --reload` - Start development server
- `pytest` - Run tests

### Scripts
- `python scripts/index_docs.py --base-url <url>` - Index documents for RAG

## Deployment

### Docusaurus (GitHub Pages)
The book is automatically deployed to GitHub Pages via GitHub Actions when pushing to the `main` branch.

### RAG API
See `infra/README.md` for deployment instructions.

## Environment Variables

See `.env.example` for required environment variables:
- `OPENAI_API_KEY` - For embeddings and LLM orchestration
- `QDRANT_URL` - Qdrant Cloud endpoint
- `QDRANT_API_KEY` - Qdrant API key
- `NEON_DATABASE_URL` - Neon Postgres connection string
- `RAG_API_BASE_URL` - Backend API URL (for frontend)

## Features

- 📚 Complete textbook with chapters, labs, and assessments
- 🤖 RAG chatbot with full corpus and selected text modes
- 🔗 Citation links to source sections
- 🔍 Search functionality
- 📊 Mermaid diagrams for visual learning

## License

[Add license information]

