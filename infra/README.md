# Infrastructure Setup

This directory contains scripts and documentation for setting up the RAG chatbot infrastructure.

## Prerequisites

- Neon Postgres database account
- Qdrant Cloud account (or self-hosted Qdrant)
- Python 3.11+
- uv package manager (recommended) or pip

## Setup Steps

### 1. Neon Postgres Setup

1. Create a Neon account at [https://neon.tech](https://neon.tech)
2. Create a new project
3. Copy the connection string
4. Run the schema setup:

```bash
# Using psql
psql <your-connection-string> -f schema.sql

# Or using Python
python setup_db.py
```

### 2. Qdrant Setup

1. Create a Qdrant Cloud account at [https://cloud.qdrant.io](https://cloud.qdrant.io)
2. Create a new cluster
3. Copy the cluster URL and API key
4. Run the collection setup:

```bash
python setup_qdrant.py
```

### 3. Environment Variables

Create a `.env` file in the `rag-api` directory:

```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=postgresql://user:password@host/database?sslmode=require
```

## Deployment

### RAG API Deployment Options

1. **Container Deployment** (Recommended)
   - Build Docker image
   - Deploy to container platform (Railway, Render, Fly.io, etc.)

2. **Serverless** (Alternative)
   - Deploy to AWS Lambda, Google Cloud Functions, or Azure Functions
   - Note: May require adjustments for long-running operations

3. **VPS/Cloud Instance**
   - Deploy to DigitalOcean, AWS EC2, etc.
   - Use systemd or supervisor for process management

### Example Docker Deployment

```bash
cd rag-api
docker build -t rag-api .
docker run -p 8000:8000 --env-file .env rag-api
```

## Monitoring

- Health endpoint: `GET /health`
- API documentation: `GET /docs` (Swagger UI)
- Alternative docs: `GET /redoc`

