"""
FastAPI application for RAG chatbot backend.
Provides endpoints for full corpus Q&A and selected text only mode.
"""

import os
from dotenv import load_dotenv
from typing import List, Optional
from fastapi import FastAPI, HTTPException, Depends, Header
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field
import psycopg2
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue
from openai import OpenAI
import time
from collections import defaultdict

load_dotenv()

    # Initialize FastAPI app
app = FastAPI(
    title="Physical AI Textbook RAG API",
    description="RAG chatbot API for Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)
app.state.app = app # Store the app instance in its own state

    # CORS configuration
cors_origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configuration
# OpenRouter configuration (using OpenRouter instead of OpenAI directly)
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
OPENROUTER_BASE_URL = "https://openrouter.ai/api/v1"
OPENROUTER_MODEL = os.getenv("OPENROUTER_MODEL", "openai/gpt-4o-mini")  # GPT-4o mini via OpenRouter

# For embeddings, we can still use OpenAI or use OpenRouter's embedding models
# If you want to use OpenRouter for embeddings too, set EMBEDDING_MODEL
EMBEDDING_MODEL = os.getenv("EMBEDDING_MODEL", "text-embedding-3-small")
USE_OPENROUTER_EMBEDDINGS = os.getenv("USE_OPENROUTER_EMBEDDINGS", "false").lower() == "true"

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
NEON_DATABASE_URL = os.getenv("NEON_DATABASE_URL")
QDRANT_COLLECTION = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_textbook")
RATE_LIMIT_PER_MINUTE = int(os.getenv("RATE_LIMIT_PER_MINUTE", "60"))

# Initialize OpenAI client configured for OpenRouter
if OPENROUTER_API_KEY:
    openai_client = OpenAI(
        api_key=OPENROUTER_API_KEY,
        base_url=OPENROUTER_BASE_URL,
        default_headers={
            "HTTP-Referer": os.getenv("OPENROUTER_REFERRER", "https://github.com/your-username/hackathon_2_claude"),
            "X-Title": "Physical AI Textbook RAG"
        }
    )
else:
    openai_client = None

qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY) if QDRANT_URL and QDRANT_API_KEY else None

# Rate limiting (simple in-memory implementation)
rate_limit_store = defaultdict(list)


def get_db_connection():
    """Get database connection"""
    if not NEON_DATABASE_URL:
        raise HTTPException(status_code=500, detail="Database not configured")
    return psycopg2.connect(NEON_DATABASE_URL)


def check_rate_limit(api_key: Optional[str] = None):
    """Simple rate limiting check"""
    identifier = api_key or "anonymous"
    now = time.time()
    
    # Clean old entries
    rate_limit_store[identifier] = [
        t for t in rate_limit_store[identifier] if now - t < 60
    ]
    
    # Check limit
    if len(rate_limit_store[identifier]) >= RATE_LIMIT_PER_MINUTE:
        raise HTTPException(
            status_code=429,
            detail="Rate limit exceeded. Please try again later."
        )
    
    # Add current request
    rate_limit_store[identifier].append(now)


# Request/Response models
class ChatRequest(BaseModel):
    query: str = Field(..., description="User's question")
    user_id: Optional[str] = Field(None, description="Optional user ID for logging")


class SelectionChatRequest(BaseModel):
    query: str = Field(..., description="User's question")
    selection_text: str = Field(..., description="Selected text from the page")
    user_id: Optional[str] = Field(None, description="Optional user ID for logging")


class Citation(BaseModel):
    text: str
    url: str
    section_id: str


class ChatResponse(BaseModel):
    answer: str
    citations: List[Citation]


class HealthResponse(BaseModel):
    status: str
    database: str
    qdrant: str
    openai: str  # Actually OpenRouter now


# Endpoints
@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint"""
    db_status = "ok"
    qdrant_status = "ok"
    openai_status = "ok"
    # Check database
    print(f"Checking database: {NEON_DATABASE_URL}")
    try:
        conn = get_db_connection()
        conn.close()
    except Exception as e:
        db_status = f"error: {str(e)}"
    
    # Check Qdrant
    if qdrant_client:
        try:
            collections = qdrant_client.get_collections()
            qdrant_status = "ok"
        except Exception as e:
            qdrant_status = f"error: {str(e)}"
    else:
        qdrant_status = "not configured"
    
    # Check OpenRouter/OpenAI
    if openai_client:
        try:
            # Simple check - just verify client is initialized
            openai_status = f"ok (OpenRouter: {OPENROUTER_MODEL})"
        except Exception as e:
            openai_status = f"error: {str(e)}"
    else:
        openai_status = "not configured"
    
    overall_status = "ok" if all(s == "ok" for s in [db_status, qdrant_status, openai_status]) else "degraded"
    
    return HealthResponse(
        status=overall_status,
        database=db_status,
        qdrant=qdrant_status,
        openai=openai_status
    )


@app.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    x_api_key: Optional[str] = Header(None)
):
    """
    Chat endpoint for full corpus Q&A.
    Retrieves relevant chunks from Qdrant and generates answer with citations.
    """
    check_rate_limit(x_api_key)
    
    if not openai_client or not qdrant_client:
        raise HTTPException(
            status_code=500,
            detail="OpenAI or Qdrant not configured"
        )
    
    try:
        # Generate query embedding
        # Note: For embeddings, you may need to use OpenAI directly or OpenRouter's embedding models
        # OpenRouter supports some embedding models, but text-embedding-3-small might need OpenAI
        # You can set OPENAI_API_KEY for embeddings only if needed
        embedding_client = openai_client
        if USE_OPENROUTER_EMBEDDINGS:
            embedding_client = openai_client
        else:
            # Fallback to OpenAI for embeddings if OPENAI_API_KEY is set
            openai_key = os.getenv("OPENAI_API_KEY")
            if openai_key:
                embedding_client = OpenAI(api_key=openai_key)
            else:
                embedding_client = openai_client
        
        query_embedding = embedding_client.embeddings.create(
            model=EMBEDDING_MODEL,
            input=request.query
        ).data[0].embedding
        
        # Search Qdrant
        search_results = qdrant_client.search(
            collection_name=QDRANT_COLLECTION,
            query_vector=query_embedding,
            limit=5
        )
        
        # Format context with citations
        context_parts = []
        citations = []
        
        for result in search_results:
            payload = result.payload
            text = payload.get('text', '')
            url = payload.get('url', '')
            section_id = payload.get('section_id', '')
            
            context_parts.append(f"[{section_id}]: {text}")
            citations.append(Citation(
                text=text[:200] + "..." if len(text) > 200 else text,
                url=url,
                section_id=section_id
            ))
        
        context = "\n\n".join(context_parts)
        
        # Generate answer using OpenRouter with tool-driven pattern
        system_prompt = """You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
Answer questions based on the provided context from the textbook.
Always cite your sources using the section IDs provided in the context.
If the context doesn't contain enough information, say so."""

        user_prompt = f"""Context from textbook:
{context}

Question: {request.query}

Answer the question based on the context above. Include citations in your answer."""

        # Define tools for tool-driven pattern (OpenAI Agents SDK style)
        tools = [
            {
                "type": "function",
                "function": {
                    "name": "format_citations",
                    "description": "Format citations from retrieved context",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "citations": {
                                "type": "array",
                                "items": {
                                    "type": "object",
                                    "properties": {
                                        "section_id": {"type": "string"},
                                        "url": {"type": "string"},
                                        "text": {"type": "string"}
                                    }
                                }
                            }
                        },
                        "required": ["citations"]
                    }
                }
            }
        ]
        
        response = openai_client.chat.completions.create(
            model=OPENROUTER_MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            tools=tools,
            tool_choice="auto",  # Let the model decide when to use tools
            temperature=0.7
        )
        
        answer = response.choices[0].message.content
        
        # Log interaction (optional)
        if request.user_id:
            try:
                conn = get_db_connection()
                cursor = conn.cursor()
                cursor.execute("""
                    INSERT INTO chat_logs (user_id, query, response, mode, citations)
                    VALUES (%s, %s, %s, %s, %s)
                """, (
                    request.user_id,
                    request.query,
                    answer,
                    "full_corpus",
                    [{"url": c.url, "section_id": c.section_id} for c in citations]
                ))
                conn.commit()
                cursor.close()
                conn.close()
            except Exception as e:
                # Log error but don't fail the request
                print(f"Error logging chat: {e}")
        
        return ChatResponse(answer=answer, citations=citations)
        
    except Exception as e:
        print(str(e))
        raise HTTPException(status_code=500, detail=f"Error processing chat: {str(e)}")


@app.post("/chat/selection", response_model=ChatResponse)
async def chat_selection(request: SelectionChatRequest, x_api_key: Optional[str] = Header(None)):
    """
    Chat endpoint for selected text only mode.
    Answers questions using ONLY the provided selection text, no retrieval from Qdrant.
    """
    check_rate_limit(x_api_key)
    
    if not openai_client:
        raise HTTPException(
            status_code=500,
            detail="OpenAI not configured"
        )
    
    try:
        # Generate answer using ONLY the selection text
        system_prompt = """You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
You MUST answer questions using ONLY the provided selected text.
Do NOT use any knowledge outside of the selected text.
If the selected text doesn't contain enough information to answer the question,
explicitly state that you cannot answer based on the provided text alone."""
        
        user_prompt = f"""Selected text from textbook:
{request.selection_text}

Question: {request.query}

Answer the question using ONLY the selected text above. Do not use any outside knowledge.
If you cannot answer from the selected text, say so explicitly."""

        # For selection mode, use selection constraint tool
        selection_tools = [
            {
                "type": "function",
                "function": {
                    "name": "enforce_selection_only",
                    "description": "Enforce that answers must come ONLY from the provided selected text",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "can_answer": {
                                "type": "boolean",
                                "description": "Whether the question can be answered from the selected text"
                            },
                            "answer": {
                                "type": "string",
                                "description": "The answer using ONLY the selected text, or explanation if cannot answer"
                            }
                        },
                        "required": ["can_answer", "answer"]
                    }
                }
            }
        ]
        
        response = openai_client.chat.completions.create(
            model=OPENROUTER_MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            tools=selection_tools,
            tool_choice="auto",
            temperature=0.7
        )
        
        # Extract answer from response
        message = response.choices[0].message
        answer = message.content
        
        # If tool was called, extract the answer from tool call
        if message.tool_calls:
            for tool_call in message.tool_calls:
                if tool_call.function.name == "enforce_selection_only":
                    import json
                    tool_args = json.loads(tool_call.function.arguments)
                    answer = tool_args.get("answer", answer)
        
        # Create a citation pointing to "selection"
        citations = [Citation(
            text=request.selection_text[:200] + "..." if len(request.selection_text) > 200 else request.selection_text,
            url="#selection",
            section_id="selection"
        )]
        
        # Log interaction (optional)
        if request.user_id:
            try:
                conn = get_db_connection()
                cursor = conn.cursor()
                cursor.execute("""
                    INSERT INTO chat_logs (user_id, query, response, mode, citations)
                    VALUES (%s, %s, %s, %s, %s)
                """, (
                    request.user_id,
                    request.query,
                    answer,
                    "selection",
                    [{"url": "#selection", "section_id": "selection"}]
                ))
                conn.commit()
                cursor.close()
                conn.close()
            except Exception as e:
                print(f"Error logging chat: {e}")
        
        return ChatResponse(answer=answer, citations=citations)
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat: {str(e)}")


@app.post("/ingest")
async def ingest_documents(
    x_api_key: Optional[str] = Header(None)
):
    """
    Trigger document ingestion (admin/dev only).
    In production, this should be protected with proper authentication.
    """
    # Simple API key check (replace with proper auth in production)
    admin_key = os.getenv("ADMIN_API_KEY")
    if admin_key and x_api_key != admin_key:
        raise HTTPException(status_code=403, detail="Unauthorized")
    
    # This endpoint would trigger the indexing script
    # For now, return instructions
    return JSONResponse(
        content={
            "message": "Ingestion endpoint. Run: python scripts/index_docs.py --base-url <url>",
            "status": "manual_ingestion_required"
        }
    )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

