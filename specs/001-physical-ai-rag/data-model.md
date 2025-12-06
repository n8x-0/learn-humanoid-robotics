# Data Model for RAG Backend

## Core Entities

### Document
Represents a textbook chapter or section. This is the primary unit of content that will be chunked and indexed.
- `doc_id` (string): Stable, unique identifier for the document (e.g., derived from file path or URL).
- `title` (string): Title of the chapter/section.
- `url` (string): URL to the document in the Docusaurus site.
- `content` (string): Full text content of the document.

### Chunk
A smaller, semantically coherent part of a Document, used for vector storage and retrieval.
- `chunk_id` (string): Unique identifier for the chunk.
- `doc_id` (string): Foreign key, links to the parent Document.
- `section_id` (string): Stable identifier for the section within the document (e.g., heading ID), crucial for citation links.
- `text_content` (string): The actual text content of this chunk.
- `embedding` (vector): Numerical representation of `text_content` for semantic search.

## Optional/Bonus Features Entities (for future consideration)

### User
Represents a user of the system, primarily for personalization and authentication.
- `user_id` (string): Unique identifier for the user.
- `profile` (JSON/object): Stores background information (software/hardware experience).

### Chat Log
Records of chatbot interactions.
- `log_id` (string): Unique identifier for the chat log entry.
- `user_id` (string): Foreign key, links to the User (if authenticated).
- `timestamp` (datetime): When the interaction occurred.
- `query` (string): The user's question.
- `response` (string): The chatbot's answer.
- `mode` (enum: 'full_corpus', 'selection'): The mode in which the chatbot was used.

### Translation
Stores translated content for chapters/sections.
- `translation_id` (string): Unique identifier for the translation.
- `section_id` (string): Foreign key, links to the Chunk's section_id.
- `locale` (string): Language locale (e.g., 'ur-PK' for Urdu).
- `translated_text` (string): The translated content.