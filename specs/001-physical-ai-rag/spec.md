# Feature Specification: Physical AI & Humanoid Robotics Textbook + RAG Chatbot

**Feature Branch**: `001-physical-ai-rag`
**Created**: 2025-12-05
**Status**: Draft
**Input**: User description: "# Specification — Physical AI & Humanoid Robotics Textbook + RAG Chatbot\n\n## A) Book (Docusaurus)\n### A1. Structure\n- Title: Physical AI & Humanoid Robotics\n- Sections:\n  1) Preface & How to Use This Book\n  2) Foundations (Weeks 1–2)\n  3) ROS 2 Nervous System (Weeks 3–5)\n  4) Digital Twin (Gazebo & Unity) (Weeks 6–7)\n  5) NVIDIA Isaac Platform (Weeks 8–10)\n  6) Humanoid Development (Weeks 11–12)\n  7) Conversational Robotics (Week 13)\n  8) Capstone Guide + Rubrics\n  9) Appendix: Hardware Tracks (On-Prem vs Cloud “Ether Lab”)\n  10) Glossary + References\n\n### A2. Content Requirements\n- Must include the provided module outlines, weekly breakdown, outcomes, and hardware requirements.\n- Every chapter includes objectives, lab, checkpoint quiz, and references.\n- Include a “Lab Setup” chapter: Ubuntu 22.04, ROS 2 Humble/Iron, Gazebo, Isaac Sim prerequisites.\n- Include “Safety & Ethics” notes for physical systems.\n\n### A3. Publishing\n- Deploy via GitHub Actions to GitHub Pages.\n- Use versioned docs and clean navbar/sidebars.\n- Add search (local search plugin or Algolia if available).\n\n## B) RAG Chatbot (Embedded)\n### B1. UX Requirements\n- Chat widget embedded on docs pages with:\n  - Mode toggle:\n    - “Ask the book” (full corpus)\n    - “Ask selected text only”\n  - Citation links that open the cited section.\n- Selected-text mode:\n  - User highlights text on page and clicks “Ask from selection”\n  - The chatbot receives selected text as the only allowed context.\n\n### B2. Backend Requirements\n- FastAPI service exposes:\n  - POST `/chat` (full corpus)\n  - POST `/chat/selection` (restrict to selection text)\n  - POST `/ingest` (admin/dev only) to index docs\n  - GET `/health`\n- Storage:\n  - Qdrant Cloud (vectors)\n  - Neon Postgres (metadata: documents, chunks, users, chat logs)\n- Retrieval:\n  - Chunk documents by heading sections, keep stable `doc_id`, `section_id`, `url`.\n  - Hybrid retrieval recommended (BM25-ish optional later), but minimum: vector retrieval + rerank (optional).\n- LLM orchestration:\n  - Use OpenAI Agents/ChatKit SDKs in a tool-driven pattern:\n    - retrieval tool\n    - citation formatter tool\n    - selection constraint tool for selection mode\n- Security:\n  - API key in env vars\n  - CORS allow frontend domain\n  - Rate limit basic protection (optional)\n\n## C) Bonus (Optional but Scaffold)\n### C1. Better Auth\n- Add auth via Better Auth.\n- At signup: collect background (software + hardware experience).\n- Store profile answers in Neon.\n\n### C2. Personalization\n- “Personalize this chapter” button:\n  - Calls backend to produce an adapted summary/explanation based on user profile.\n  - Saves personalized version in DB (or caches).\n\n### C3. Urdu Translation\n- “Translate to Urdu” button:\n  - On click: request translation for the current section\n  - Display side-by-side or toggled view\n  - Cache translations in DB keyed by section + locale.\n\n## D) Repo Layout\n- `/book` — Docusaurus site\n- `/rag-api` — FastAPI backend\n- `/infra` — scripts for Qdrant/Neon setup + ingestion\n- `/scripts` — doc chunking + embedding + upload\n- Root `README.md` with one-command local dev instructions\n\n## E) Acceptance Tests\n- `npm run build` in `/book` succeeds.\n- GitHub Pages deploy workflow runs successfully.\n- RAG:\n  - Can answer “What is ROS 2?” with citations.\n  - Selection mode refuses to use outside knowledge and only cites selection text.\n  - Health endpoint returns OK."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Read Textbook Content (Priority: P1)

Users can navigate and read the "Physical AI & Humanoid Robotics" textbook content, organized into defined sections and chapters.

**Why this priority**: Core functionality; provides fundamental educational value without any additional features.

**Independent Test**: The Docusaurus site builds successfully and all major sections/chapters are accessible and display content as intended.

**Acceptance Scenarios**:

1.  **Given** a user accesses the GitHub Pages site, **When** they navigate through the table of contents, **Then** all chapters and sections load correctly.
2.  **Given** a user is on any chapter page, **When** they scroll through the content, **Then** learning objectives, labs, checkpoint quizzes, and references are present.

---

### User Story 2 - Ask the Book Chatbot (Priority: P1)

Users can ask questions to an embedded RAG chatbot, which provides answers based on the full textbook corpus with citations.

**Why this priority**: Primary value-add of the RAG chatbot, enabling interactive learning and quick information retrieval.

**Independent Test**: The RAG chatbot responds accurately to questions from the book, including citations, and the health endpoint returns OK.

**Acceptance Scenarios**:

1.  **Given** a user is on a textbook page with the chat widget open, **When** they type a question related to the book's content (e.g., "What is ROS 2?"), **Then** the chatbot provides a relevant answer with clickable citations that navigate to the correct section.
2.  **Given** the RAG API is deployed, **When** a GET request is made to `/health`, **Then** it returns an "OK" status.

---

### User Story 3 - Ask Selected Text Only Chatbot (Priority: P1)

Users can highlight specific text on a page and ask the chatbot a question, constraining the chatbot to only use the selected text as context.

**Why this priority**: Essential feature for targeted learning and understanding specific passages within the textbook, and a key differentiator.

**Independent Test**: The chatbot, in "selected text only" mode, answers questions solely from the provided selection and refuses to use outside knowledge.

**Acceptance Scenarios**:

1.  **Given** a user highlights a section of text on a page and selects "Ask from selection", **When** they ask a question relevant only to the selected text, **Then** the chatbot provides an answer using only the selected text as context and does not cite outside knowledge.
2.  **Given** a user highlights text and asks a question, **When** the chatbot is in "selected text only" mode, **Then** it explicitly indicates if it cannot answer from the given text.

---

### Edge Cases

- What happens when the RAG backend is unresponsive or returns an error? (Chatbot UI should gracefully handle and inform the user).
- How does the system handle very large sections of text for chunking during ingestion? (Ensure robust chunking strategy to maintain context and stable IDs).
- What if a citation link leads to a section that has been moved or renamed? (Stable IDs and robust URL generation are critical).
- How is security handled for the `/ingest` endpoint to prevent unauthorized access? (Authentication/authorization required, likely admin/dev only).

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The Docusaurus site MUST be titled "Physical AI & Humanoid Robotics".
- **FR-002**: The textbook MUST include Preface, Foundations, ROS 2, Digital Twin, NVIDIA Isaac, Humanoid Development, Conversational Robotics, Capstone Guide, Appendix, and Glossary sections.
- **FR-003**: Every chapter MUST include learning objectives, a hands-on lab or exercise, a mini-quiz or checkpoint, and references/further reading.
- **FR-004**: The book MUST include a "Lab Setup" chapter detailing prerequisites (Ubuntu 22.04, ROS 2 Humble/Iron, Gazebo, Isaac Sim).
- **FR-005**: The book MUST include "Safety & Ethics" notes relevant to physical systems.
- **FR-006**: The Docusaurus site MUST be deployed via GitHub Actions to GitHub Pages.
- **FR-007**: The Docusaurus site MUST use versioned documentation and provide clean navigation (navbar/sidebars).
- **FR-008**: The Docusaurus site MUST include a search functionality (local search plugin or Algolia).
- **FR-009**: The RAG chatbot MUST be embedded as a chat widget on docs pages.
- **FR-010**: The chat widget MUST include a mode toggle: "Ask the book" (full corpus) and "Ask selected text only".
- **FR-011**: The chatbot responses MUST include citation links that open the cited section.
- **FR-012**: In "selected text only" mode, the chatbot MUST receive the user-highlighted text as its sole context.
- **FR-013**: The RAG backend MUST expose a POST `/chat` endpoint for full corpus Q&A.
- **FR-014**: The RAG backend MUST expose a POST `/chat/selection` endpoint for Q&A restricted to selected text.
- **FR-015**: The RAG backend MUST expose a POST `/ingest` endpoint for document indexing (admin/dev only).
- **FR-016**: The RAG backend MUST expose a GET `/health` endpoint.
- **FR-017**: Vector storage for retrieval MUST use Qdrant Cloud.
- **FR-018**: Metadata (documents, chunks, users, chat logs) MUST be stored in Neon Serverless Postgres.
- **FR-019**: Document chunking MUST be performed by heading sections, preserving stable `doc_id`, `section_id`, and `url`.
- **FR-020**: Retrieval MUST include vector retrieval.
- **FR-021**: LLM orchestration MUST use OpenAI Agents/ChatKit SDKs with tool-driven patterns (retrieval, citation formatter, selection constraint).
- **FR-022**: API keys for the backend MUST be stored in environment variables, not in the repository.
- **FR-023**: CORS MUST be properly configured to allow the frontend domain.
- **FR-024**: The repository MUST have a `/book` directory for the Docusaurus site.
- **FR-025**: The repository MUST have a `/rag-api` directory for the FastAPI backend.
- **FR-026**: The repository MUST have an `/infra` directory for Qdrant/Neon setup and ingestion scripts.
- **FR-027**: The repository MUST have a `/scripts` directory for doc chunking, embedding, and upload.
- **FR-028**: The root `README.md` MUST include one-command local development instructions.

### Key Entities *(include if feature involves data)*

- **Document**: Represents a textbook chapter or section. Key attributes: `doc_id` (stable), `title`, `url`, `content`.
- **Chunk**: A smaller, semantically coherent part of a Document. Key attributes: `chunk_id`, `doc_id`, `section_id` (stable), `text_content`, `embedding`.
- **User**: (Optional, for Bonus features) Represents a user of the system. Key attributes: `user_id`, `profile` (software/hardware background).
- **Chat Log**: (Optional) Records of chatbot interactions. Key attributes: `log_id`, `user_id`, `timestamp`, `query`, `response`, `mode` (full corpus/selection).
- **Translation**: (Optional) Stored translations. Key attributes: `translation_id`, `section_id`, `locale`, `translated_text`.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: `npm run build` in the `/book` directory MUST succeed without errors.
- **SC-002**: The GitHub Pages deployment workflow MUST run successfully, making the textbook publicly accessible.
- **SC-003**: The RAG chatbot, in "Ask the book" mode, MUST correctly answer 95% of questions based on the textbook corpus with accurate citations.
- **SC-004**: The RAG chatbot, in "Ask selected text only" mode, MUST correctly answer 95% of questions using *only* the provided selection as context and explicitly refuse outside knowledge.
- **SC-005**: The `/rag-api/health` endpoint MUST return a 200 OK status within 50ms.
- **SC-006**: The chatbot's citation links MUST reliably navigate to the correct section within the Docusaurus site.

## Clarifications (Optional but Scaffold)

### Bonus Features

- **Auth**:
  - **FR-C1.1**: User authentication MUST be integrated via Better Auth.
  - **FR-C1.2**: User signup process MUST collect background information (software and hardware experience).
  - **FR-C1.3**: User profile answers MUST be stored in Neon Postgres.
- **Personalization**:
  - **FR-C2.1**: A "Personalize this chapter" button MUST be present on chapter pages.
  - **FR-C2.2**: Clicking the personalization button MUST trigger a backend call to produce an adapted summary/explanation based on the user's profile.
  - **FR-C2.3**: Personalized versions MUST be saved in the database or cached.
- **Urdu Translation**:
  - **FR-C3.1**: A "Translate to Urdu" button MUST be present on chapter pages.
  - **FR-C3.2**: Clicking the translation button MUST request a translation for the current section.
  - **FR-C3.3**: Translated content MUST be displayed side-by-side or in a toggled view.
  - **FR-C3.4**: Translations MUST be cached in the database, keyed by section and locale.
