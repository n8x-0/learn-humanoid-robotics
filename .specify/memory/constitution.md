<!-- Sync Impact Report:
Version change: 0.0.0 (initial) → 1.0.0
Modified principles:
  - Mission (new)
  - Non-Negotiables (new)
  - Preferred Principles (new)
Added sections:
  - Quality Bar
  - Bonus Features
  - Definition of Done
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md (⚠ pending)
  - .specify/templates/spec-template.md (⚠ pending)
  - .specify/templates/tasks-template.md (⚠ pending)
  - .specify/templates/commands/*.md (⚠ pending)
Follow-up TODOs: None
-->
# Spec-Kit Constitution — Physical AI & Humanoid Robotics Textbook

## Core Principles

### 1) Mission
Deliver a production-grade, Docusaurus-based textbook titled “Physical AI & Humanoid Robotics” deployed to GitHub Pages, featuring:
- A complete course-aligned book structure with chapters, labs, and assessments.
- An embedded RAG chatbot that answers questions from the book and can answer using only user-selected text.
- A clean, reproducible dev setup enabling controlled vibe-coding via Spec-Kit Plus + Claude Code subagents/skills.

### 2) Non-Negotiables (Must-Haves)
- Docusaurus site builds and deploys on GitHub Pages (CI included).
- Book content maps to the provided syllabus (Modules + Week breakdown + hardware track).
- RAG chatbot integrated into the published site UI.
- Backend stack for RAG: FastAPI + Neon Serverless Postgres + Qdrant Cloud Free Tier.
- RAG uses OpenAI Agents/ChatKit SDKs and supports:
  - Full-book Q&A
  - “Answer from selected text only” mode
- Security basics: environment variables, no secrets in repo, CORS properly configured.

### 3) Preferred Principles
- Content first: ship a coherent learning journey before adding bells/whispers.
- Stable IDs: chapters, headings, and embeddings should remain stable across edits.
- Separation of concerns:
  - Docusaurus = content + UI
  - RAG backend = indexing + retrieval + chat
  - Data stores = Qdrant for vectors, Neon for metadata/user profiles/chat logs
- Automation where safe: scripts for ingestion/indexing and deployment.

### 4) Development Workflow
- All work will be done in a GitHub branch named `docs`.
- Pushes to `main` will only occur when explicitly asked to "go live".

## Quality Bar

- Every chapter includes:
  - Learning objectives
  - Core concepts
  - Hands-on lab or exercise
  - Mini-quiz or checkpoint
  - References / further reading
- Code samples must run or be clearly labeled as pseudocode.
- Diagrams must be reproducible (Mermaid preferred).
- RAG answers must include citations (chapter/section anchors).

## Bonus Features (If time permits)

- Reusable intelligence: Claude Code subagents + skills for content generation, diagramming, QA, and style linting.
- Signup/Signin using Better Auth, with onboarding questionnaire (software/hardware background).
- Personalization button per chapter (rewrite/adapt explanations based on user profile).
- Urdu translation button per chapter.

## Definition of Done

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

## Governance

This Constitution is the ultimate source of truth for project governance. Amendments require a formal proposal, review, and approval by project leadership. All code contributions and architectural decisions must align with these principles. Deviations require explicit, documented exceptions.

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
