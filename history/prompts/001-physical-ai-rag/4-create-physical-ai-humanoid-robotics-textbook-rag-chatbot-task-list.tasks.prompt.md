---
id: 4
title: Create Physical AI & Humanoid Robotics Textbook + RAG Chatbot Task List
stage: tasks
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-rag
branch: 001-physical-ai-rag
user: user
command: /sp.tasks
labels: ["task-generation","implementation-planning"]
links:
  spec: specs/001-physical-ai-rag/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-rag/tasks.md
tests:
  - null
---

## Prompt

```text
# Tasks — Executable Backlog

## 0. Repo & Tooling
- [ ] Create mono-repo folders: `book/`, `rag-api/`, `scripts/`, `infra/`
- [ ] Add root README with dev commands
- [ ] Add `.env.example` for book + rag-api
- [ ] Add lint/format configs (prettier/eslint for book; ruff/black for rag-api)

## 1. Docusaurus Book
- [ ] Initialize Docusaurus in `/book`
- [ ] Configure navbar, footer, sidebars for modules/weeks
- [ ] Create chapter template MDX with required sections
- [ ] Write:
  - [ ] Foundations of Physical AI (Weeks 1–2)
  - [ ] ROS 2 Nervous System (Weeks 3–5)
  - [ ] Gazebo + Unity Digital Twin (Weeks 6–7)
  - [ ] NVIDIA Isaac Platform (Weeks 8–10)
  - [ ] Humanoid Dev (Weeks 11–12)
  - [ ] Conversational Robotics (Week 13)
  - [ ] Capstone project guide + rubric
  - [ ] Hardware ] return citations pointing to selection (or “selection” tag)
- [ ] Implement `/ingest` (dev/admin):
  - [ ] trigger indexing script or accept payload
- [ ] Add CORS config + rate limiting (basic)

## 5. Chat UI in Docusaurus
- [ ] Create React Chat widget component
- [ ] Add floating button + panel
- [ ] Add selection capturequirements + Ether Lab option
- [ ] Add Mermaid diagrams per chapter
- [ ] Add glossary + references section

## 2. Data Stores (Neon + Qdrant)
- [ ] Create Neon database + tables:
  - [ ] documents
  - [ ] chunks
  - [ ] users (optional)
  - [ ] chat_logs
  - [ ] personalization_cache (optional)
  - [ ] translations_cache (optional)
- [ ] Create Qdrant collection with payload schema

## 3. Indexing Pipeline
- [ ] Implement markdown/MDX parser to chunk by headings
- [ ] Generate stable chunk IDs and store metadata in Neon
- [ ] Embed chunks and upsert to Qdrant
- [ ] Add CLI:
  - `python scripts/index_docs.py --base-url <gh-pages-url>`

## 4. RAG API (FastAPI)
- [ ] Scaffold FastAPI project in `/rag-api`
- [ ] Implement `/health`
- [ ] Implement `/chat`:
  - [ ] embed query
  - [ ] retrieve from Qdrant
  - [ ] format context with citations
  - [ ] respond with answer + citation list
- [ ] Implement `/chat/selection`:
  - [ ] accept selection text + question
  - [ ] answer strictly from selection-only
  - [ re button
- [ ] Add mode toggle (full vs selection)
- [ ] Render citations as links
- [ ] Add config for API base URL

## 6. Deployment
- [ ] GitHub Actions for Docusaurus build + deploy to Pages
- [ ] Document RAG API deployment steps
- [ ] Set env vars in deployment target
- [ ] End-to-end smoke test checklist

## 7. Bonus: Reusable Intelligence (Claude Code)
- [ ] Define subagents:
  - [ ] ChapterWriter
  - [ ] LabDesigner
  - [ ] Diagrammer (Mermaid)
  - [ ] QA/LinkChecker
  - [ ] RAGTuner
- [ ] Define Skills:
  - [ ] “Create chapter from syllabus block”
  - [ ] “Generate lab steps + expected output”
  - [ ] “Convert concept to Mermaid diagram”
  - [ ] “Extract glossary terms”
  - [ ] “Create quiz questions”

## 8. Bonus: Auth + Personalization + Urdu
- [ ] Integrate Better Auth (signup/login)
- [ ] Signup questionnaire (software/hardware background)
- [ ] Store profile in Neon
- [ ] Add “Personalize chapter” button:
  - [ ] call backend
  - [ ] render adapted content
  - [ ] cache result
- [ ] Add “Translate to Urdu” button:
  - [ ] call backend
  - [ ] render Urdu content
  - [ ] cache result
```

## Response snapshot

The task list for "Physical AI & Humanoid Robotics Textbook + RAG Chatbot" has been successfully generated and written to `specs/001-physical-ai-rag/tasks.md`. The plan details the implementation steps organized by phases and user stories, with dependencies and parallelization opportunities outlined.

## Outcome

- ✅ Impact: A detailed task list has been created, providing an executable backlog for feature development.
- 🧪 Tests: No automated tests run for task generation.
- 📁 Files: `specs/001-physical-ai-rag/tasks.md` created.
- 🔁 Next prompts: User can now proceed with implementing the tasks, potentially using `/sp.implement` or other execution commands.
- 🧠 Reflection: The task generation process successfully translated the high-level plan and user stories into granular, actionable steps, adhering to the specified format.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Automate the conversion of task list items into TodoWrite calls during the implementation phase.
