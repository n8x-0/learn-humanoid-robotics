"""Application settings and configuration."""
from pydantic_settings import BaseSettings
from typing import Optional
from dotenv import load_dotenv

load_dotenv()

class Settings(BaseSettings):
    """Application settings loaded from environment variables."""
    
    # OpenAI Configuration
    openai_api_key: Optional[str] = None
    
    # Qdrant Configuration
    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "humanoid_book"
    
    # Ingest Configuration
    ingest_app_id: str = "humanoid_rag_app"
    
    # GitHub Webhook Configuration
    github_webhook_secret: Optional[str] = None
    
    # Chunking Configuration
    chunk_max_tokens: int = 1000
    chunk_overlap_tokens: int = 100
    
    # Embedding Configuration
    embedding_model: str = "text-embedding-3-large"
    embedding_dimension: int = 3072
    
    # LLM Configuration
    llm_provider: str = "openai"  # "openai" or "openrouter"
    llm_model: str = "gpt-4o-mini"
    openrouter_api_key: Optional[str] = None
    
    # Book Path Configuration
    book_docs_path: str = "../book/docs"
    
    # CORS Configuration - can be comma-separated string from env
    cors_origins: str = "http://localhost:3000,http://localhost:5173,http://localhost:3001"
    
    @property
    def cors_origins_list(self) -> list[str]:
        """Parse CORS origins from comma-separated string to list."""
        return [origin.strip() for origin in self.cors_origins.split(",") if origin.strip()]
    
    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False


# Global settings instance
settings = Settings()

