# Implementation Checklist

## Docusaurus Book (`/book`)
- [ ] **FR-001**: The Docusaurus site MUST be titled "Physical AI & Humanoid Robotics".
- [ ] **FR-002**: The textbook MUST include Preface, Foundations, ROS 2, Digital Twin, NVIDIA Isaac, Humanoid Development, Conversational Robotics, Capstone Guide, Appendix, and Glossary sections.
- [ ] **FR-003**: Every chapter MUST include learning objectives, a hands-on lab or exercise, a mini-quiz or checkpoint, and references/further reading.
- [ ] **FR-004**: The book MUST include a "Lab Setup" chapter detailing prerequisites (Ubuntu 22.04, ROS 2 Humble/Iron, Gazebo, Isaac Sim).
- [ ] **FR-005**: The book MUST include "Safety & Ethics" notes relevant to physical systems.
- [ ] **FR-006**: The Docusaurus site MUST be deployed via GitHub Actions to GitHub Pages.
- [ ] **FR-007**: The Docusaurus site MUST use versioned documentation and provide clean navigation (navbar/sidebars).
- [ ] **FR-008**: The Docusaurus site MUST include a search functionality (local search plugin or Algolia).

## RAG Chatbot Frontend (in `/book`)
- [ ] **FR-009**: The RAG chatbot MUST be embedded as a chat widget on docs pages.
- [ ] **FR-010**: The chat widget MUST include a mode toggle: "Ask the book" (full corpus) and "Ask selected text only".
- [ ] **FR-011**: The chatbot responses MUST include citation links that open the cited section.
- [ ] **FR-012**: In "selected text only" mode, the chatbot MUST receive the user-highlighted text as its sole context.

## RAG Backend (`/rag-api`)
- [x] **FR-013**: The RAG backend MUST expose a POST `/chat` endpoint for full corpus Q&A.
- [x] **FR-014**: The RAG backend MUST expose a POST `/chat/selection` endpoint for Q&A restricted to selected text.
- [x] **FR-015**: The RAG backend MUST expose a POST `/ingest` endpoint for document indexing (admin/dev only).
- [x] **FR-016**: The RAG backend MUST expose a GET `/health` endpoint.
- [ ] **FR-017**: Vector storage for retrieval MUST use Qdrant Cloud.
- [ ] **FR-018**: Metadata (documents, chunks, users, chat logs) MUST be stored in Neon Serverless Postgres.
- [ ] **FR-020**: Retrieval MUST include vector retrieval.
- [x] **FR-021**: LLM orchestration MUST use the OpenAI Agents SDK, configured via a custom-developed adapter to use a Gemini API key.
- [x] **FR-022**: API keys for the backend MUST be stored in environment variables, not in the repository.
- [x] **FR-023**: CORS MUST be properly configured to allow the frontend domain.

## Infrastructure & Scripts (`/infra`, `/scripts`)
- [ ] **FR-019**: Document chunking MUST be performed by heading sections, preserving stable `doc_id`, `section_id`, and `url`.
- [ ] **FR-026**: The repository MUST have an `/infra` directory for Qdrant/Neon setup and ingestion scripts.
- [ ] **FR-027**: The repository MUST have a `/scripts` directory for doc chunking, embedding, and upload.

## Project Structure & Documentation
- [x] **FR-024**: The repository MUST have a `/book` directory for the Docusaurus site.
- [x] **FR-025**: The repository MUST have a `/rag-api` directory for the FastAPI backend.
- [ ] **FR-028**: The root `README.md` MUST include one-command local development instructions.

## Non-Functional Requirements
- [ ] **NFR-001**: The system SHOULD be designed to handle up to 50 textbook chapters/sections and 100 concurrent RAG chatbot users.
