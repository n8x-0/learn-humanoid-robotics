#!/usr/bin/env python3
"""
Main indexing script that orchestrates document parsing and embedding.
Usage: python scripts/index_docs.py --base-url <gh-pages-url>
"""

import os
import sys
import argparse
from pathlib import Path

# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent))

from doc_parser import DocumentParser
from embedder import Embedder


def main():
    parser = argparse.ArgumentParser(
        description='Index Docusaurus documents for RAG chatbot'
    )
    parser.add_argument(
        '--base-url',
        type=str,
        required=True,
        help='Base URL for generated document URLs (e.g., https://username.github.io)'
    )
    parser.add_argument(
        '--docs-dir',
        type=str,
        default='book/docs',
        help='Path to docs directory (default: book/docs)'
    )
    parser.add_argument(
        '--db-url',
        type=str,
        default=os.getenv('NEON_DATABASE_URL'),
        help='Neon database connection string'
    )
    parser.add_argument(
        '--qdrant-url',
        type=str,
        default=os.getenv('QDRANT_URL'),
        help='Qdrant URL'
    )
    parser.add_argument(
        '--qdrant-api-key',
        type=str,
        default=os.getenv('QDRANT_API_KEY'),
        help='Qdrant API key'
    )
    parser.add_argument(
        '--openai-api-key',
        type=str,
        default=os.getenv('OPENAI_API_KEY'),
        help='OpenAI API key'
    )
    parser.add_argument(
        '--collection',
        type=str,
        default='physical_ai_textbook',
        help='Qdrant collection name'
    )
    parser.add_argument(
        '--skip-embedding',
        action='store_true',
        help='Skip embedding step (only parse and store chunks)'
    )
    
    args = parser.parse_args()
    
    # Validate required arguments
    if not args.db_url:
        print("Error: NEON_DATABASE_URL not set and --db-url not provided")
        sys.exit(1)
    
    if not args.skip_embedding:
        if not args.qdrant_url or not args.qdrant_api_key:
            print("Error: QDRANT_URL and QDRANT_API_KEY required for embedding")
            sys.exit(1)
        if not args.openai_api_key:
            print("Error: OPENAI_API_KEY required for embedding")
            sys.exit(1)
    
    docs_dir = Path(args.docs_dir)
    if not docs_dir.exists():
        print(f"Error: Docs directory not found: {docs_dir}")
        sys.exit(1)
    
    print("=" * 60)
    print("Indexing Documents for RAG Chatbot")
    print("=" * 60)
    print(f"Base URL: {args.base_url}")
    print(f"Docs directory: {docs_dir}")
    print()
    
    # Step 1: Parse documents
    print("Step 1: Parsing documents...")
    parser_obj = DocumentParser(args.base_url, args.db_url)
    
    md_files = list(docs_dir.rglob('*.md')) + list(docs_dir.rglob('*.mdx'))
    print(f"Found {len(md_files)} markdown files")
    
    total_chunks = 0
    for md_file in md_files:
        print(f"  Processing: {md_file.relative_to(docs_dir)}")
        chunks = parser_obj.parse_file(md_file)
        
        if chunks:
            doc_id = chunks[0]['doc_id']
            title = md_file.stem
            url = chunks[0]['url'].rsplit('#', 1)[0]
            parser_obj.store_document(doc_id, title, url)
            parser_obj.store_chunks(chunks)
            total_chunks += len(chunks)
            print(f"    → Created {len(chunks)} chunks")
    
    parser_obj.close()
    print(f"\n✓ Parsed {len(md_files)} files, created {total_chunks} chunks")
    print()
    
    # Step 2: Generate embeddings and upload to Qdrant
    if not args.skip_embedding:
        print("Step 2: Generating embeddings and uploading to Qdrant...")
        embedder = Embedder(
            db_connection_string=args.db_url,
            qdrant_url=args.qdrant_url,
            qdrant_api_key=args.qdrant_api_key,
            openai_api_key=args.openai_api_key,
            collection_name=args.collection
        )
        
        chunks = embedder.get_chunks()
        print(f"Found {len(chunks)} chunks to embed")
        
        embedder.upload_to_qdrant(chunks)
        embedder.close()
        print(f"\n✓ Successfully indexed {len(chunks)} chunks")
    else:
        print("Step 2: Skipped (--skip-embedding flag set)")
    
    print()
    print("=" * 60)
    print("Indexing complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()

