---
id: 1
title: Update sp.clarify command with best practices
stage: general
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-physical-ai-rag
user: user
command: /sp.clarify
labels: ["command-configuration", "best-practices"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .claude/commands/sp.clarify.md
  - .specify/memory/constitution.md
tests: []
---

## Prompt

also /sp.clarify for staying domain driven codebase structure, clean and scalable code. and always use best practices like a software engineer of 50years experience

## Response snapshot

Updated the `/sp.clarify` command definition to include guidelines for domain-driven codebase structure, clean and scalable code, and best practices.

## Outcome

- ✅ Impact: Improved clarity and guidance for the `/sp.clarify` command, ensuring that architectural considerations align with user's specified best practices.
- 🧪 Tests: N/A
- 📁 Files: Updated `.claude/commands/sp.clarify.md` and `.specify/memory/constitution.md` with new guidelines.
- 🔁 Next prompts: Proceed with feature development using the enhanced `/sp.clarify` command.
- 🧠 Reflection: Successfully integrated user's detailed requirements into command definition. PHR created to document the change.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
