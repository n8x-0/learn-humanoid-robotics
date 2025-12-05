---
id: 3
title: Create Physical AI & Humanoid Robotics Textbook + RAG Chatbot Implementation Plan
stage: plan
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-rag
branch: 001-physical-ai-rag
user: user
command: /sp.plan
labels: ["planning", "implementation"]
links:
  spec: specs/001-physical-ai-rag/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-rag/plan.md
tests:
  - null
---

## Prompt

```text
# Plan — Build the Book + RAG + (Optional) Auth/Personalization/Urdu

## Phase 0 — Repo Bootstrap (Day 0)
1. Create mono-repo structure: `/book`, `/rag-api`, `/scripts`, `/infra`.
2. Add root tooling:
   - `.env.example` for both services
   - Makefile or npm scripts to run both locally
3. Initialize Spec-Kit Plus workflow and Claude Code subagents scaffolding.

## Phase 1 — Docusaurus Book MVP (Days 1–2)
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

## Phase 2 — Ingestion + Chunking Pipeline (Day 2–3)
1. Implement doc loader that reads Docusaurus markdown/MDX output:
   - parse headings to chunk sections
   - compute stable chunk IDs
   - store metadata to Neon
2. Implement embedding + upload:
   - embed chunks
   - upsert to Qdrant with payload: urls, headings, doc version
3. Add re-index command:
   - `scripts/index_docs.py`

## Phase 3 — RAG API (Days 3–4)
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

## Phase 4 — Frontend Chat Widget Embed (Day 4)
1. Add a Docusaurus plugin/component:
   - floating chat button
   - panel UI
2. Implement selection capture:
   - read `window.getSelection()`
   - pass to `/chat/selection`
3. Render citations as links to headings/anchors.

## Phase 5 — Deployment (Day 5)
1. GitHub Actions:
   - build and deploy Docusaurus to GitHub Pages
2. Deploy RAG API:
   - simplest: container or managed platform (document deployment steps)
   - configure CORS and env vars
3. Smoke test end-to-end from GitHub Pages.

## Phase 6 — Bonus Tracks (Parallel / If Time)
### Bonus A: Clau
```

## Response snapshot

The implementation plan for "Physical AI & Humanoid Robotics Textbook + RAG Chatbot" has been successfully generated and written to `specs/001-physical-ai-rag/plan.md`. The plan details the technical context, aligns with the project constitution, outlines the project structure, and breaks down the implementation into six distinct phases.

## Outcome

- ✅ Impact: A detailed implementation plan has been created, providing a roadmap for feature development.
- 🧪 Tests: No automated tests run for plan generation.
- 📁 Files: `specs/001-physical-ai-rag/plan.md` created.
- 🔁 Next prompts: User can now proceed with `/sp.tasks` to generate actionable tasks based on this plan.
- 🧠 Reflection: The planning process successfully integrated the user's high-level plan with project-specific details and constitutional checks, creating a comprehensive document.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Automate the update of dependent templates based on constitution changes.
