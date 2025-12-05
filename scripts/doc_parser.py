#!/usr/bin/env python3
"""
Document parser for chunking Docusaurus markdown/MDX files by headings.
Generates stable chunk IDs and stores metadata in Neon Postgres.
"""

import os
import re
import hashlib
from pathlib import Path
from typing import List, Dict, Optional
import psycopg2
from psycopg2.extras import execute_values


class DocumentParser:
    """Parse markdown/MDX documents and chunk by headings"""
    
    def __init__(self, base_url: str, db_connection_string: str):
        self.base_url = base_url.rstrip('/')
        self.db_conn = psycopg2.connect(db_connection_string)
        self.heading_pattern = re.compile(r'^(#{1,6})\s+(.+)$', re.MULTILINE)
    
    def parse_file(self, file_path: Path) -> List[Dict]:
        """Parse a markdown/MDX file and return chunks"""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Extract frontmatter if present
        frontmatter = self._extract_frontmatter(content)
        title = frontmatter.get('title', file_path.stem)
        
        # Remove frontmatter from content
        content = self._remove_frontmatter(content)
        
        # Generate stable doc_id from file path
        doc_id = self._generate_doc_id(file_path)
        
        # Generate URL
        url = self._generate_url(file_path)
        
        # Chunk by headings
        chunks = self._chunk_by_headings(
            doc_id=doc_id,
            title=title,
            content=content,
            url=url,
            file_path=file_path
        )
        
        return chunks
    
    def _extract_frontmatter(self, content: str) -> Dict:
        """Extract YAML frontmatter from content"""
        frontmatter = {}
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                import yaml
                try:
                    frontmatter = yaml.safe_load(parts[1]) or {}
                except:
                    pass
        return frontmatter
    
    def _remove_frontmatter(self, content: str) -> str:
        """Remove YAML frontmatter from content"""
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                return parts[2].strip()
        return content
    
    def _generate_doc_id(self, file_path: Path) -> str:
        """Generate stable doc_id from file path"""
        # Use relative path from docs directory
        rel_path = str(file_path).replace('\\', '/')
        if '/docs/' in rel_path:
            rel_path = rel_path.split('/docs/')[1]
        # Remove extension and create stable ID
        rel_path = rel_path.replace('.md', '').replace('.mdx', '')
        return rel_path.replace('/', '_')
    
    def _generate_url(self, file_path: Path) -> str:
        """Generate URL for document"""
        rel_path = str(file_path).replace('\\', '/')
        if '/docs/' in rel_path:
            rel_path = rel_path.split('/docs/')[1]
        rel_path = rel_path.replace('.md', '').replace('.mdx', '')
        return f"{self.base_url}/docs/{rel_path}"
    
    def _chunk_by_headings(self, doc_id: str, title: str, content: str, url: str, file_path: Path) -> List[Dict]:
        """Chunk content by headings"""
        chunks = []
        
        # Find all headings
        headings = list(self.heading_pattern.finditer(content))
        
        if not headings:
            # No headings, create single chunk
            chunk_id = self._generate_chunk_id(doc_id, "intro", 0)
            chunks.append({
                'chunk_id': chunk_id,
                'doc_id': doc_id,
                'section_id': 'intro',
                'text_content': content.strip(),
                'chunk_index': 0,
                'url': url
            })
            return chunks
        
        # Process sections between headings
        for i, heading_match in enumerate(headings):
            heading_level = len(heading_match.group(1))
            heading_text = heading_match.group(2).strip()
            heading_start = heading_match.start()
            
            # Find end of section (next heading of same or higher level, or end of content)
            section_end = len(content)
            if i + 1 < len(headings):
                next_heading = headings[i + 1]
                # Find next heading at same or higher level
                for j in range(i + 1, len(headings)):
                    if len(headings[j].group(1)) <= heading_level:
                        section_end = headings[j].start()
                        break
            
            # Extract section content
            section_content = content[heading_start:section_end].strip()
            
            # Generate stable section_id from heading
            section_id = self._slugify(heading_text)
            
            # Create chunk
            chunk_id = self._generate_chunk_id(doc_id, section_id, len(chunks))
            chunks.append({
                'chunk_id': chunk_id,
                'doc_id': doc_id,
                'section_id': section_id,
                'text_content': section_content,
                'chunk_index': len(chunks),
                'url': f"{url}#{section_id}"
            })
        
        return chunks
    
    def _slugify(self, text: str) -> str:
        """Convert text to URL-friendly slug"""
        text = text.lower()
        text = re.sub(r'[^\w\s-]', '', text)
        text = re.sub(r'[-\s]+', '-', text)
        return text.strip('-')
    
    def _generate_chunk_id(self, doc_id: str, section_id: str, index: int) -> str:
        """Generate stable chunk ID"""
        combined = f"{doc_id}_{section_id}_{index}"
        return hashlib.md5(combined.encode()).hexdigest()[:16]
    
    def store_document(self, doc_id: str, title: str, url: str, content: Optional[str] = None):
        """Store document metadata in database"""
        cursor = self.db_conn.cursor()
        cursor.execute("""
            INSERT INTO documents (doc_id, title, url, content)
            VALUES (%s, %s, %s, %s)
            ON CONFLICT (doc_id) DO UPDATE
            SET title = EXCLUDED.title,
                url = EXCLUDED.url,
                content = EXCLUDED.content,
                updated_at = CURRENT_TIMESTAMP
        """, (doc_id, title, url, content))
        self.db_conn.commit()
        cursor.close()
    
    def store_chunks(self, chunks: List[Dict]):
        """Store chunks in database"""
        if not chunks:
            return
        
        cursor = self.db_conn.cursor()
        
        # Prepare data for bulk insert
        values = [
            (
                chunk['chunk_id'],
                chunk['doc_id'],
                chunk['section_id'],
                chunk['text_content'],
                chunk['chunk_index'],
                chunk['url']
            )
            for chunk in chunks
        ]
        
        execute_values(
            cursor,
            """
            INSERT INTO chunks (chunk_id, doc_id, section_id, text_content, chunk_index, url)
            VALUES %s
            ON CONFLICT (chunk_id) DO UPDATE
            SET text_content = EXCLUDED.text_content,
                chunk_index = EXCLUDED.chunk_index,
                url = EXCLUDED.url,
                updated_at = CURRENT_TIMESTAMP
            """,
            values
        )
        
        self.db_conn.commit()
        cursor.close()
    
    def close(self):
        """Close database connection"""
        self.db_conn.close()


def main():
    """Main function for CLI usage"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Parse and chunk Docusaurus documents')
    parser.add_argument('--docs-dir', type=str, default='book/docs',
                       help='Path to docs directory')
    parser.add_argument('--base-url', type=str, required=True,
                       help='Base URL for generated document URLs')
    parser.add_argument('--db-url', type=str,
                       default=os.getenv('NEON_DATABASE_URL'),
                       help='Neon database connection string')
    
    args = parser.parse_args()
    
    if not args.db_url:
        print("Error: NEON_DATABASE_URL not set and --db-url not provided")
        sys.exit(1)
    
    docs_dir = Path(args.docs_dir)
    if not docs_dir.exists():
        print(f"Error: Docs directory not found: {docs_dir}")
        sys.exit(1)
    
    parser_obj = DocumentParser(args.base_url, args.db_url)
    
    # Find all markdown files
    md_files = list(docs_dir.rglob('*.md')) + list(docs_dir.rglob('*.mdx'))
    
    print(f"Found {len(md_files)} markdown files")
    
    total_chunks = 0
    for md_file in md_files:
        print(f"Processing: {md_file}")
        chunks = parser_obj.parse_file(md_file)
        
        if chunks:
            # Store document
            doc_id = chunks[0]['doc_id']
            title = md_file.stem
            url = chunks[0]['url'].rsplit('#', 1)[0]  # Remove anchor
            parser_obj.store_document(doc_id, title, url)
            
            # Store chunks
            parser_obj.store_chunks(chunks)
            total_chunks += len(chunks)
            print(f"  → Created {len(chunks)} chunks")
    
    print(f"\n✓ Total chunks created: {total_chunks}")
    parser_obj.close()


if __name__ == "__main__":
    import sys
    main()

