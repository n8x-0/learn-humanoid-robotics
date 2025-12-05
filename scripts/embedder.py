#!/usr/bin/env python3
"""
Embed chunks and upload to Qdrant.
Reads chunks from Neon Postgres, generates embeddings, and stores in Qdrant.
"""

import os
import sys
from typing import List, Dict
import psycopg2
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
try:
    from openai import OpenAI
except ImportError:
    print("Error: openai not installed. Install it with: pip install openai")
    sys.exit(1)


class Embedder:
    """Generate embeddings and upload to Qdrant"""
    
    def __init__(
        self,
        db_connection_string: str,
        qdrant_url: str,
        qdrant_api_key: str,
        openai_api_key: str,
        collection_name: str = "physical_ai_textbook",
        model: str = "text-embedding-3-small"
    ):
        self.db_conn = psycopg2.connect(db_connection_string)
        self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        self.openai_client = OpenAI(api_key=openai_api_key)
        self.collection_name = collection_name
        self.model = model
        self.embedding_dim = 1536  # text-embedding-3-small dimension
    
    def get_chunks(self, limit: int = None) -> List[Dict]:
        """Get chunks from database that need embedding"""
        cursor = self.db_conn.cursor()
        
        query = """
            SELECT chunk_id, doc_id, section_id, text_content, url
            FROM chunks
            ORDER BY doc_id, chunk_index
        """
        
        if limit:
            query += f" LIMIT {limit}"
        
        cursor.execute(query)
        rows = cursor.fetchall()
        cursor.close()
        
        return [
            {
                'chunk_id': row[0],
                'doc_id': row[1],
                'section_id': row[2],
                'text_content': row[3],
                'url': row[4]
            }
            for row in rows
        ]
    
    def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for text using OpenAI"""
        # Clean text (remove markdown syntax, extra whitespace)
        text = self._clean_text(text)
        
        response = self.openai_client.embeddings.create(
            model=self.model,
            input=text
        )
        
        return response.data[0].embedding
    
    def _clean_text(self, text: str) -> str:
        """Clean text for embedding"""
        # Remove markdown headers
        import re
        text = re.sub(r'^#{1,6}\s+', '', text, flags=re.MULTILINE)
        # Remove code blocks (keep content)
        text = re.sub(r'```[\s\S]*?```', '', text)
        # Remove inline code markers
        text = re.sub(r'`([^`]+)`', r'\1', text)
        # Normalize whitespace
        text = ' '.join(text.split())
        return text.strip()
    
    def upload_to_qdrant(self, chunks: List[Dict], batch_size: int = 100):
        """Upload chunks with embeddings to Qdrant"""
        total = len(chunks)
        print(f"Processing {total} chunks...")
        
        for i in range(0, total, batch_size):
            batch = chunks[i:i + batch_size]
            print(f"Processing batch {i // batch_size + 1} ({i + 1}-{min(i + batch_size, total)})...")
            
            points = []
            for chunk in batch:
                # Generate embedding
                embedding = self.generate_embedding(chunk['text_content'])
                
                # Create point
                point = PointStruct(
                    id=chunk['chunk_id'],
                    vector=embedding,
                    payload={
                        'doc_id': chunk['doc_id'],
                        'section_id': chunk['section_id'],
                        'text': chunk['text_content'][:1000],  # Store first 1000 chars
                        'url': chunk['url']
                    }
                )
                points.append(point)
            
            # Upload batch
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            
            print(f"  ✓ Uploaded {len(points)} points")
    
    def close(self):
        """Close connections"""
        self.db_conn.close()


def main():
    """Main function for CLI usage"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate embeddings and upload to Qdrant')
    parser.add_argument('--db-url', type=str,
                       default=os.getenv('NEON_DATABASE_URL'),
                       help='Neon database connection string')
    parser.add_argument('--qdrant-url', type=str,
                       default=os.getenv('QDRANT_URL'),
                       help='Qdrant URL')
    parser.add_argument('--qdrant-api-key', type=str,
                       default=os.getenv('QDRANT_API_KEY'),
                       help='Qdrant API key')
    parser.add_argument('--openai-api-key', type=str,
                       default=os.getenv('OPENAI_API_KEY'),
                       help='OpenAI API key')
    parser.add_argument('--collection', type=str,
                       default='physical_ai_textbook',
                       help='Qdrant collection name')
    parser.add_argument('--limit', type=int,
                       help='Limit number of chunks to process')
    
    args = parser.parse_args()
    
    # Validate required arguments
    required = {
        'db_url': args.db_url,
        'qdrant_url': args.qdrant_url,
        'qdrant_api_key': args.qdrant_api_key,
        'openai_api_key': args.openai_api_key
    }
    
    missing = [k for k, v in required.items() if not v]
    if missing:
        print(f"Error: Missing required arguments: {', '.join(missing)}")
        print("Set them via command line or environment variables")
        sys.exit(1)
    
    embedder = Embedder(
        db_connection_string=args.db_url,
        qdrant_url=args.qdrant_url,
        qdrant_api_key=args.qdrant_api_key,
        openai_api_key=args.openai_api_key,
        collection_name=args.collection
    )
    
    # Get chunks
    chunks = embedder.get_chunks(limit=args.limit)
    
    if not chunks:
        print("No chunks found in database. Run doc_parser.py first.")
        sys.exit(1)
    
    # Upload to Qdrant
    embedder.upload_to_qdrant(chunks)
    
    print(f"\n✓ Successfully uploaded {len(chunks)} chunks to Qdrant")
    embedder.close()


if __name__ == "__main__":
    main()

