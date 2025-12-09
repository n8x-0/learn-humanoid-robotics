"""Utilities for reading and parsing markdown files."""
import os
from pathlib import Path
from typing import List
import re

from ..models.rag_models import MarkdownDocument, Section


def load_markdown_documents(base_path: str) -> List[MarkdownDocument]:
    """
    Recursively load all markdown files from the specified directory.
    
    Args:
        base_path: Base directory path containing markdown files
        
    Returns:
        List of MarkdownDocument objects
    """
    documents = []
    base = Path(base_path)
    
    # Find all .md files recursively
    for md_file in base.rglob("*.md"):
        try:
            with open(md_file, "r", encoding="utf-8") as f:
                content = f.read()
            
            # Get relative path from base
            relative_path = str(md_file.relative_to(base))
            documents.append(MarkdownDocument(
                file_path=relative_path,
                content=content
            ))
        except Exception as e:
            print(f"Error reading {md_file}: {e}")
            continue
    
    return documents


def parse_markdown_sections(document: MarkdownDocument) -> List[Section]:
    """
    Parse a markdown document into sections based on headers.
    
    Args:
        document: MarkdownDocument to parse
        
    Returns:
        List of Section objects
    """
    sections = []
    lines = document.content.split("\n")
    current_header = "Introduction"
    current_level = 0
    current_text = []
    
    for line in lines:
        # Check for headers (#, ##, ###, etc.)
        header_match = re.match(r"^(#{1,6})\s+(.+)$", line)
        
        if header_match:
            # Save previous section if it has content
            if current_text:
                sections.append(Section(
                    header=current_header,
                    text="\n".join(current_text).strip(),
                    file_path=document.file_path,
                    level=current_level
                ))
                current_text = []
            
            # Start new section
            current_level = len(header_match.group(1))
            current_header = header_match.group(2).strip()
        else:
            # Add line to current section
            if line.strip() or current_text:  # Include empty lines if we have content
                current_text.append(line)
    
    # Add final section
    if current_text:
        sections.append(Section(
            header=current_header,
            text="\n".join(current_text).strip(),
            file_path=document.file_path,
            level=current_level
        ))
    
    # If no sections were found, treat entire document as one section
    if not sections:
        sections.append(Section(
            header="Introduction",
            text=document.content.strip(),
            file_path=document.file_path,
            level=1
        ))
    
    return sections

