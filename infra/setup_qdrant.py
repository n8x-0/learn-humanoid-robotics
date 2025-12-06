#!/usr/bin/env python3
"""
Setup script for Qdrant vector database.
Run this script to create the collection for document embeddings.
"""

import os
import sys

try:
    from qdrant_client import QdrantClient
    from qdrant_client.models import Distance, VectorParams, CollectionStatus
except ImportError:
    print("Error: qdrant-client not installed. Install it with: pip install qdrant-client")
    sys.exit(1)


def setup_qdrant_collection(
    url: str,
    api_key: str,
    collection_name: str = "physical_ai_textbook",
    vector_size: int = 768  # Gemini text-embedding-004 dimension
):
    """Create Qdrant collection for document embeddings"""
    try:
        client = QdrantClient(url=url, api_key=api_key)
        
        # Check if collection exists
        collections = client.get_collections().collections
        collection_names = [col.name for col in collections]
        
        if collection_name in collection_names:
            print(f"✓ Collection '{collection_name}' already exists")
            return
        
        # Create collection
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=vector_size,
                distance=Distance.COSINE
            )
        )
        
        print(f"✓ Collection '{collection_name}' created successfully")
        print(f"  - Vector size: {vector_size}")
        print(f"  - Distance metric: COSINE")
        
    except Exception as e:
        print(f"Error setting up Qdrant: {e}")
        sys.exit(1)


if __name__ == "__main__":
    # Get credentials from environment or command line
    url = os.getenv("QDRANT_URL")
    api_key = os.getenv("QDRANT_API_KEY")
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_textbook")
    
    if not url or not api_key:
        if len(sys.argv) >= 3:
            url = sys.argv[1]
            api_key = sys.argv[2]
            if len(sys.argv) > 3:
                collection_name = sys.argv[3]
        else:
            print("Usage: python setup_qdrant.py <url> <api_key> [collection_name]")
            print("Or set QDRANT_URL and QDRANT_API_KEY environment variables")
            sys.exit(1)
    
    setup_qdrant_collection(url, api_key, collection_name)

