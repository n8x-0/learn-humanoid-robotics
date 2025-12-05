# Implementation Plan: Physical AI & Humanoid Robotics Textbook + RAG Chatbot

**Branch**: `001-physical-ai-rag` | **Date**: 2025-12-05 | **Spec**: specs/001-physical-ai-rag/spec.md
**Input**: Feature specification from `/specs/001-physical-ai-rag/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The plan outlines the phased implementation for the "Physical AI & Humanoid Robotics Textbook + RAG Chatbot" feature. It covers repository bootstrapping, Docusaurus book development, ingestion and chunking pipelines, RAG API development, frontend chat widget embedding, deployment, and optional bonus tracks. The core goal is to deliver a functional Docusaurus-based textbook with an embedded RAG chatbot that can answer questions from the book, including a "selected text only" mode, and is deployable via GitHub Pages.

## Technical Context

**Package Manager**: uv for python, npm for JavaScript.
**Language/Version**: Python (FastAPI), JavaScript/TypeScript (Docusaurus) - Specific versions will be determined during Phase 0.
**Primary Dependencies**: FastAPI, Neon Serverless Postgres, Qdrant Cloud, OpenAI Agents/ChatKit SDKs, Docusaurus, GitHub Actions.
**Storage**: Neon Serverless Postgres (metadata: documents, chunks, users, chat logs), Qdrant Cloud (vectors).
**Testing**: `npm run build` for Docusaurus, API health checks (`/rag-api/health`), RAG chatbot acceptance tests (full corpus and selected text only modes).
**Target Platform**: GitHub Pages (Docusaurus frontend), Container/Managed Platform (FastAPI backend).
**Project Type**: Monorepo (`/book`, `/rag-api`, `/infra`, `/scripts`) with a Docusaurus-based web application (frontend) and a FastAPI service (backend).
**Performance Goals**: `/rag-api/health` endpoint MUST return a 200 OK status within 50ms. RAG chatbot in both modes MUST correctly answer 95% of questions.
**Constraints**: API keys in environment variables, CORS properly configured, no secrets in repository, rate limit protection (optional).

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

*   **1) Mission**: The plan aligns with the mission to deliver a production-grade, Docusaurus-based textbook titled "Physical AI & Humanoid Robotics" deployed to GitHub Pages, featuring a RAG chatbot that answers questions from the book, including a "selected text only" mode.
*   **2) Non-Negotiables (Must-Haves)**:
    *   Docusaurus site builds and deploys on GitHub Pages (CI included): **PASS**. Phase 5 covers GitHub Actions deployment.
    *   Book content maps to syllabus: **PASS**. Phase 1 outlines content creation based on modules/weeks.
    *   RAG chatbot integrated into published site UI: **PASS**. Phase 4 covers frontend embedding.
    *   Backend stack for RAG (FastAPI + Neon + Qdrant): **PASS**. Phase 3 confirms FastAPI, storage in Phase 2.
    *   RAG uses OpenAI Agents/ChatKit SDKs: **PASS**. Explicitly mentioned in Phase 3.
    *   Supports full-book Q&A and “Answer from selected text only” mode: **PASS**. Phase 3 includes implementation for both modes.
    *   Security basics (env vars, no secrets, CORS): **PASS**. Phase 5 covers CORS and env vars; implicit in `.env.example`.
*   **3) Preferred Principles**:
    *   Content first: **PASS**. Phase 1 focuses on drafting core content.
    *   Stable IDs: **PASS**. Phase 2 explicitly mentions computing stable chunk IDs.
    *   Separation of concerns: **PASS**. Monorepo structure (book/rag-api/infra/scripts) promotes clear separation.
    *   Automation where safe: **PASS**. Phase 2 includes `scripts/index_docs.py`, Phase 5 includes GitHub Actions.
*   **4) Quality Bar**: The plan's objectives (chapter templates, diagrams, accurate RAG responses) align with the defined quality bar.
*   **5) Bonus Features**: The plan includes a "Bonus Tracks" phase to scaffold optional features, aligning with the constitution's guidance on these.
*   **6) Definition of Done**: The phases (Repo Bootstrap to Deployment) align with the "Definition of Done" covering repo contents, live GitHub Pages, and chatbot functionality.

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-rag/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command) - to be created
├── data-model.md        # Phase 1 output (/sp.plan command) - to be created
├── quickstart.md        # Phase 1 output (/sp.plan command) - to be created
├── contracts/           # Phase 1 output (/sp.plan command) - to be created
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
/
├── book/                # Docusaurus site
├── rag-api/             # FastAPI backend
├── infra/               # scripts for Qdrant/Neon setup + ingestion
├── scripts/             # doc chunking + embedding + upload
└── README.md            # Root README with local dev instructions
```

**Structure Decision**: The project will use a monorepo structure with dedicated directories for the Docusaurus frontend, FastAPI backend, infrastructure scripts, and general utility scripts, as defined in Section D of the specification.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No constitution check violations were identified in this plan.

## Plan — Build the Book + RAG + (Optional) Auth/Personalization/Urdu

## Phase 0 — Repo Bootstrap
1. Create mono-repo structure: `/book`, `/rag-api`, `/scripts`, `/infra`.
2. Add root tooling:
   - `.env.example` for both services
   - Makefile or npm scripts to run both locally
3. Initialize Spec-Kit Plus workflow and Claude Code subagents scaffolding.

## Phase 1 — Docusaurus Book MVP
1. Generate Docusaurus site in `/book`.
2. Create sidebar structure mapping exactly to Modules/Weeks.
3. Draft chapter templates:
   - objectives
   - core concepts
   - lab
   - checkpoint quiz
   - references
4. Write core content:
   - Modules 1–4
   - Capstone guide
   - Hardware tracks (On-Prem vs Ether Lab)
5. Add diagrams (Mermaid) for:
   - ROS 2 nodes/topics/services/actions
   - sim-to-real pipeline
   - VLA pipeline (Whisper → LLM planner → ROS actions)

## Phase 2 — Ingestion + Chunking Pipeline
1. Implement doc loader that reads Docusaurus markdown/MDX output:
   - parse headings to chunk sections
   - compute stable chunk IDs
   - store metadata to Neon
2. Implement embedding + upload:
   - embed chunks
   - upsert to Qdrant with payload: urls, headings, doc version
3. Add re-index command:
   - `scripts/index_docs.py`

## Phase 3 — RAG API
1. Build FastAPI service:
   - `/chat`, `/chat/selection`, `/ingest`, `/health`
2. Implement retrieval:
   - query embeddings
   - fetch top-k from Qdrant
   - assemble context with citations
3. Implement “selection-only” mode:
   - do NOT call Qdrant retrieval tool
   - answer only from provided selection text
   - add strict system instruction and guardrails
4. Add basic logging + error handling.

## Phase 4 — Frontend Chat Widget Embed
1. Add a Docusaurus plugin/component:
   - floating chat button
   - panel UI
2. Implement selection capture:
   - read `window.getSelection()`
   - pass to `/chat/selection`
3. Render citations as links to headings/anchors.

## Phase 5 — Deployment
1. GitHub Actions:
   - build and deploy Docusaurus to GitHub Pages
2. Deploy RAG API:
   - simplest: container or managed platform (document deployment steps)
   - configure CORS and env vars
3. Smoke test end-to-end from GitHub Pages.

## Phase 6 — Bonus Tracks (Parallel / If Time)
### Bonus A: Claude Code subagents + skills for content generation, diagramming, QA, and style linting.
### Bonus B: Auth via Better Auth, collect background, store profile in Neon.
### Bonus C: Personalization button (adapt chapter based on user profile, save/cache).
### Bonus D: Urdu Translation button (request translation, display side-by-side/toggled, cache translations).
