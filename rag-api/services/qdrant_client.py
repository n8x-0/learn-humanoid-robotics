"""Qdrant client service."""
import uuid
from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

from ..settings import settings
from ..models.rag_models import Chunk, SearchResult


class QdrantService:
    """Service for interacting with Qdrant vector database."""
    
    def __init__(self):
        """Initialize Qdrant client."""
        if not settings.qdrant_url:
            raise ValueError("QDRANT_URL is required but not set in environment variables")
        
        if settings.qdrant_api_key:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key
            )
        else:
            self.client = QdrantClient(url=settings.qdrant_url)
        
        self.collection_name = settings.qdrant_collection_name
        self._ensure_collection()
    
    def _ensure_collection(self):
        """Ensure the collection exists, create if it doesn't."""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]
            
            if self.collection_name not in collection_names:
                # Create collection
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=settings.embedding_dimension,
                        distance=Distance.COSINE
                    )
                )
                print(f"Created collection: {self.collection_name}")
        except Exception as e:
            print(f"Error ensuring collection: {e}")
            raise
    
    def upsert_chunks(self, chunks: List[Chunk], vectors: List[List[float]]):
        """
        Upsert chunks with their embeddings to Qdrant.
        
        Args:
            chunks: List of Chunk objects
            vectors: List of embedding vectors corresponding to chunks
        """
        if len(chunks) != len(vectors):
            raise ValueError("Number of chunks must match number of vectors")
        
        points = []
        for chunk, vector in zip(chunks, vectors):
            point = PointStruct(
                id=str(uuid.uuid4()),
                vector=vector,
                payload={
                    "file_path": chunk.file_path,
                    "header": chunk.header,
                    "chunk": chunk.text,
                    "chunk_index": chunk.chunk_index
                }
            )
            points.append(point)
        
        # Upsert in batches
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            self.client.upsert(
                collection_name=self.collection_name,
                points=batch,
                wait=True
            )
    
    def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        score_threshold: Optional[float] = None
    ) -> List[SearchResult]:
        """
        Search for similar vectors in Qdrant.
        
        Args:
            query_vector: Query embedding vector
            top_k: Number of results to return
            score_threshold: Minimum similarity score (optional)
            
        Returns:
            List of SearchResult objects
        """
        # Use query_points method (the correct API method)
        query_params = {
            "collection_name": self.collection_name,
            "query": query_vector,
            "limit": top_k
        }
        
        if score_threshold is not None:
            query_params["score_threshold"] = score_threshold
        
        # query_points returns a QueryResponse object with .points attribute
        query_result = self.client.query_points(**query_params)
        
        # Extract the points from the query result
        results = query_result.points
        
        search_results = []
        for result in results:
            payload = result.payload
            search_results.append(SearchResult(
                score=result.score,
                file_path=payload.get("file_path", ""),
                header=payload.get("header", ""),
                chunk_text=payload.get("chunk", ""),
                chunk_index=payload.get("chunk_index", 0)
            ))
        
        return search_results


# Global instance
qdrant_service = QdrantService()

