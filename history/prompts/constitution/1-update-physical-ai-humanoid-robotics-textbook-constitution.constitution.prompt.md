---
id: 1
title: Update Physical AI & Humanoid Robotics Textbook Constitution
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-physical-ai-rag
user: user
command: /sp.constitution
labels: ["governance", "constitution"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

```text
# Spec-Kit Constitution — Physical AI & Humanoid Robotics Textbook

## 1) Mission
Deliver a production-grade, Docusaurus-based textbook titled **“Physical AI & Humanoid Robotics”** deployed to **GitHub Pages**, featuring:
- A complete course-aligned book structure with chapters, labs, and assessments.
- An embedded **RAG chatbot** that answers questions from the book and can answer using **only user-selected text**.
- A clean, reproducible dev setup enabling controlled vibe-coding via Spec-Kit Plus + Claude Code subagents/skills.

## 2) Non-Negotiables (Must-Haves)
- Docusaurus site builds and deploys on GitHub Pages (CI included).
- Book content maps to the provided syllabus (Modules + Week breakdown + hardware track).
- RAG chatbot integrated into the published site UI.
- Backend stack for RAG: **FastAPI + Neon Serverless Postgres + Qdrant Cloud Free Tier**.
- RAG uses **OpenAI Agents/ChatKit SDKs** and supports:
  - Full-book Q&A
  - “Answer from selected text only” mode
- Security basics: environment variables, no secrets in repo, CORS properly configured.

## 3) Preferred Principles
- **Content first**: ship a coherent learning journey before adding bells/whistles.
- **Stable IDs**: chapters, headings, and embeddings should remain stable across edits.
- **Separation of concerns**:
  - Docusaurus = content + UI
  - RAG backend = indexing + retrieval + chat
  - Data stores = Qdrant for vectors, Neon for metadata/user profiles/chat logs
- **Automation where safe**: scripts for ingestion/indexing and deployment.

## 4) Quality Bar
- Every chapter includes:
  - Learning objectives
  - Core concepts
  - Hands-on lab or exercise
  - Mini-quiz or checkpoint
  - References / further reading
- Code samples must run or be clearly labeled as pseudocode.
- Diagrams must be reproducible (Mermaid preferred).
- RAG answers must include citations (chapter/section anchors).

## 5) Bonus Features (If time permits)
- Reusable intelligence: Claude Code subagents + skills for content generation, diagramming, QA, and style linting.
- Signup/Signin using Better Auth, with onboarding questionnaire (software/hardware background).
- Personalization button per chapter (rewrite/adapt explanations based on user profile).
- Urdu translation button per chapter.

## 6) Definition of Done
- Repo contains:
  - Docusaurus site + content
  - RAG backend service + infra scripts
  - Deployment workflow(s)
  - README with local dev + deployment instructions
- Public GitHub Pages site is live.
- Chatbot answers:
  - Correctly uses the book corpus
  - Correctly restricts to selected text when requested
  - Provides citations
```

## Response snapshot

The constitution for 'Physical AI & Humanoid Robotics Textbook' has been successfully updated to version 1.0.0. The new constitution incorporates a detailed mission, non-negotiable requirements, preferred principles, quality bar, bonus features, and a definition of done. The governance section has been standardized. The version is 1.0.0, ratified and last amended on 2025-12-05.

## Outcome

- ✅ Impact: Project constitution updated with detailed principles and requirements.
- 🧪 Tests: No automated tests run for constitution update.
- 📁 Files: .specify/memory/constitution.md modified.
- 🔁 Next prompts: Explicit review and update of dependent templates (plan.md, spec.md, tasks.md) for full alignment.
- 🧠 Reflection: The process successfully integrated user-provided content into the constitution template, and initiated the consistency check on related templates.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Automate the update of dependent templates based on constitution changes.
