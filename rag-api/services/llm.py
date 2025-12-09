"""LLM service for generating answers."""
from typing import List
from openai import OpenAI

from settings import settings


class LLMService:
    """Service for generating answers using OpenAI or OpenRouter."""
    
    def __init__(self):
        """Initialize LLM client (OpenAI or OpenRouter)."""
        if settings.llm_provider == "openrouter":
            if not settings.openrouter_api_key:
                raise ValueError("OPENROUTER_API_KEY is required when using OpenRouter")
            # OpenRouter is OpenAI-compatible, just change base_url
            # Add optional headers for app attribution (recommended)
            default_headers = {
                "HTTP-Referer": "https://github.com/n8x-0/learn-humanoid-robotics",  # Optional: for rankings
                "X-Title": "Humanoid Robotics RAG Chatbot",  # Optional: app name
            }
            self.client = OpenAI(
                api_key=settings.openrouter_api_key,
                base_url="https://openrouter.ai/api/v1",
                default_headers=default_headers
            )
            # Use a free model if default model is OpenAI-specific
            # if settings.llm_model.startswith("gpt-"):
            #     # Use a valid free model on OpenRouter
            #     self.model = "google/gemini-flash-1.5"  # Free Gemini model on OpenRouter
            # else:
            self.model = settings.llm_model
        else:
            # Default to OpenAI
            if not settings.openai_api_key:
                raise ValueError("OPENAI_API_KEY is required when using OpenAI")
            self.client = OpenAI(api_key=settings.openai_api_key)
            self.model = settings.llm_model
    
    def generate_rag_answer(self, question: str, contexts: List[str]) -> str:
        """
        Generate an answer using RAG with provided contexts.
        
        Args:
            question: User's question
            contexts: List of context chunks from retrieval
            
        Returns:
            Generated answer
        """
        # Build context block
        context_block = "\n\n".join([
            f"[Context {i+1}]:\n{ctx}" for i, ctx in enumerate(contexts)
        ])
        
        # Build prompt
        system_prompt = """You are a helpful assistant that answers questions about humanoid robotics using the provided context from the textbook. 

Your goal is to provide helpful, informative answers based on the context provided. Use the context to answer the question as best you can. If the context contains relevant information, use it to provide a comprehensive answer. Only say you cannot answer if the context truly contains no relevant information at all."""

        user_prompt = f"""Based on the following context from the textbook, please answer the question:

{context_block}

Question: {question}

Provide a helpful answer using the information from the context above."""
        
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7,
            max_tokens=1000
        )
        
        return response.choices[0].message.content
    
    def generate_highlight_answer(self, question: str, selected_text: str) -> str:
        """
        Generate an answer using ONLY the selected text as context.
        
        Args:
            question: User's question
            selected_text: Selected text that must be used as the only context
            
        Returns:
            Generated answer
        """
        system_prompt = """You are a helpful assistant that answers questions based on the provided selected text. 

Your goal is to provide helpful answers using the selected text. Use the information in the selected text to answer the question. You can explain, summarize, or clarify what the selected text says. Only say you cannot answer if the selected text truly contains no information relevant to the question at all."""

        user_prompt = f"""Here is some selected text from the textbook:

{selected_text}

Question: {question}

Please answer the question based on the selected text above. Explain what the selected text says about the topic."""
        
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7,
            max_tokens=1000
        )
        
        return response.choices[0].message.content


# Global instance
llm_service = LLMService()

