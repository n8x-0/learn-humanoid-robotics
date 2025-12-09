"""Markdown-aware chunking utilities."""
import re
import tiktoken
from typing import List

from ..models.rag_models import Section, Chunk
from ..settings import settings


# Initialize tokenizer (using cl100k_base for GPT-4)
_encoding = tiktoken.get_encoding("cl100k_base")


def count_tokens(text: str) -> int:
    """
    Count tokens in text using tiktoken.
    
    Args:
        text: Text to count tokens for
        
    Returns:
        Number of tokens
    """
    return len(_encoding.encode(text))


def chunk_sections(
    sections: List[Section],
    max_tokens: int = None,
    overlap_tokens: int = None
) -> List[Chunk]:
    """
    Chunk sections into token-bounded chunks with overlap.
    
    Args:
        sections: List of Section objects to chunk
        max_tokens: Maximum tokens per chunk (defaults to settings)
        overlap_tokens: Overlap tokens between chunks (defaults to settings)
        
    Returns:
        List of Chunk objects
    """
    if max_tokens is None:
        max_tokens = settings.chunk_max_tokens
    if overlap_tokens is None:
        overlap_tokens = settings.chunk_overlap_tokens
    
    chunks = []
    chunk_index = 0
    
    for section in sections:
        section_text = section.text
        section_tokens = count_tokens(section_text)
        
        # If section fits in one chunk, add it as-is
        if section_tokens <= max_tokens:
            chunks.append(Chunk(
                file_path=section.file_path,
                header=section.header,
                chunk_index=chunk_index,
                text=section_text
            ))
            chunk_index += 1
        else:
            # Split section into multiple chunks
            # First, split by paragraphs to maintain some structure
            paragraphs = section_text.split("\n\n")
            current_chunk_text = []
            current_chunk_tokens = 0
            
            for para in paragraphs:
                para_tokens = count_tokens(para)
                
                # If paragraph alone exceeds max, split it by sentences
                if para_tokens > max_tokens:
                    # Save current chunk if it has content
                    if current_chunk_text:
                        chunks.append(Chunk(
                            file_path=section.file_path,
                            header=section.header,
                            chunk_index=chunk_index,
                            text="\n\n".join(current_chunk_text)
                        ))
                        chunk_index += 1
                        current_chunk_text = []
                        current_chunk_tokens = 0
                    
                    # Split paragraph by sentences
                    sentences = re.split(r'(?<=[.!?])\s+', para)
                    for sentence in sentences:
                        sent_tokens = count_tokens(sentence)
                        
                        if current_chunk_tokens + sent_tokens > max_tokens:
                            # Save current chunk
                            if current_chunk_text:
                                chunks.append(Chunk(
                                    file_path=section.file_path,
                                    header=section.header,
                                    chunk_index=chunk_index,
                                    text="\n\n".join(current_chunk_text)
                                ))
                                chunk_index += 1
                                
                                # Start new chunk with overlap
                                if overlap_tokens > 0 and current_chunk_text:
                                    # Get last few sentences for overlap
                                    overlap_text = []
                                    overlap_count = 0
                                    for sent in reversed(current_chunk_text):
                                        sent_toks = count_tokens(sent)
                                        if overlap_count + sent_toks <= overlap_tokens:
                                            overlap_text.insert(0, sent)
                                            overlap_count += sent_toks
                                        else:
                                            break
                                    current_chunk_text = overlap_text
                                    current_chunk_tokens = overlap_count
                                else:
                                    current_chunk_text = []
                                    current_chunk_tokens = 0
                        
                        current_chunk_text.append(sentence)
                        current_chunk_tokens += sent_tokens
                else:
                    # Check if adding paragraph would exceed limit
                    if current_chunk_tokens + para_tokens > max_tokens:
                        # Save current chunk
                        if current_chunk_text:
                            chunks.append(Chunk(
                                file_path=section.file_path,
                                header=section.header,
                                chunk_index=chunk_index,
                                text="\n\n".join(current_chunk_text)
                            ))
                            chunk_index += 1
                            
                            # Start new chunk with overlap
                            if overlap_tokens > 0 and current_chunk_text:
                                overlap_text = []
                                overlap_count = 0
                                for para in reversed(current_chunk_text):
                                    para_toks = count_tokens(para)
                                    if overlap_count + para_toks <= overlap_tokens:
                                        overlap_text.insert(0, para)
                                        overlap_count += para_toks
                                    else:
                                        break
                                current_chunk_text = overlap_text
                                current_chunk_tokens = overlap_count
                            else:
                                current_chunk_text = []
                                current_chunk_tokens = 0
                    
                    current_chunk_text.append(para)
                    current_chunk_tokens += para_tokens
            
            # Add final chunk if it has content
            if current_chunk_text:
                chunks.append(Chunk(
                    file_path=section.file_path,
                    header=section.header,
                    chunk_index=chunk_index,
                    text="\n\n".join(current_chunk_text)
                ))
                chunk_index += 1
    
    return chunks

