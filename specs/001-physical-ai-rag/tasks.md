---

description: "Task list for Physical AI & Humanoid Robotics Textbook + RAG Chatbot feature implementation"
---

# Tasks: Physical AI & Humanoid Robotics Textbook + RAG Chatbot

**Input**: Design documents from `/specs/001-physical-ai-rag/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume monorepo as defined in plan.md

## Phase 0: Repo & Tooling

**Purpose**: Project initialization and basic structure

- [ ] T001 [P] Create mono-repo folders: `book/`, `rag-api/`, `scripts/`, `infra/`
- [ ] T002 [P] Add root `README.md` with dev commands
- [ ] T003 [P] Add `.env.example` for book + rag-api
- [ ] T004 [P] Add lint/format configs (prettier/eslint for book; ruff/black for rag-api)

---

## Phase 1: Docusaurus Book (US1: Read Textbook Content) 🎯 MVP

**Goal**: Users can navigate and read the "Physical AI & Humanoid Robotics" textbook content, organized into defined sections and chapters.
**Independent Test**: The Docusaurus site builds successfully and all major sections/chapters are accessible and display content as intended.

### Implementation for Docusaurus Book

- [ ] T005 [P] [US1] Initialize Docusaurus in `/book`
- [ ] T006 [P] [US1] Configure navbar, footer, sidebars for modules/weeks in `book/docusaurus.config.js` and `book/sidebars.js`
- [ ] T007 [P] [US1] Create chapter template MDX with required sections in `book/docs/templates/chapter.mdx`
- [ ] T008 [US1] Write Foundations of Physical AI (Weeks 1–2) content in `book/docs/foundations/`
- [ ] T009 [US1] Write ROS 2 Nervous System (Weeks 3–5) content in `book/docs/ros2/`
- [ ] T010 [US1] Write Gazebo + Unity Digital Twin (Weeks 6–7) content in `book/docs/digital-twin/`
- [ ] T011 [US1] Write NVIDIA Isaac Platform (Weeks 8–10) content in `book/docs/isaac/`
- [ ] T012 [US1] Write Humanoid Dev (Weeks 11–12) content in `book/docs/humanoid-dev/`
- [ ] T013 [US1] Write Conversational Robotics (Week 13) content in `book/docs/conversational-robotics/`
- [ ] T014 [US1] Write Capstone project guide + rubric content in `book/docs/capstone/`
- [ ] T015 [US1] Write Hardware requirements + Ether Lab option content in `book/docs/appendix/hardware.mdx`
- [ ] T016 [P] [US1] Add Mermaid diagrams per chapter (e.g., in `book/docs/ros2/chapter1.mdx`)
- [ ] T017 [P] [US1] Add glossary + references section in `book/docs/appendix/glossary.mdx`

---

## Phase 2: Data Stores (Neon + Qdrant)

**Purpose**: Setup the necessary databases for RAG metadata and vector embeddings.

- [ ] T018 [P] Create Neon database + tables:
  - [ ] T019 [P] `documents` table
  - [ ] T020 [P] `chunks` table
  - [ ] T021 [P] `users` table (optional for bonus features)
  - [ ] T022 [P] `chat_logs` table
  - [ ] T023 [P] `personalization_cache` table (optional for bonus features)
  - [ ] T024 [P] `translations_cache` table (optional for bonus features)
- [ ] T025 [P] Create Qdrant collection with payload schema

---

## Phase 3: Indexing Pipeline

**Purpose**: Implement the document ingestion and embedding process.

- [ ] T026 Implement markdown/MDX parser to chunk by headings in `scripts/doc_parser.py`
- [ ] T027 Generate stable chunk IDs and store metadata in Neon via `scripts/doc_parser.py`
- [ ] T028 Embed chunks and upsert to Qdrant via `scripts/embedder.py`
- [ ] T029 Add CLI: `python scripts/index_docs.py --base-url <gh-pages-url>`

---

## Phase 4: RAG API (FastAPI) (US2: Ask the Book Chatbot, US3: Ask Selected Text Only Chatbot)

**Goal (US2)**: Users can ask questions to an embedded RAG chatbot, which provides answers based on the full textbook corpus with citations.
**Independent Test (US2)**: The RAG chatbot responds accurately to questions from the book, including citations, and the health endpoint returns OK.

**Goal (US3)**: Users can highlight specific text on a page and ask the chatbot a question, constraining the chatbot to only use the selected text as context.
**Independent Test (US3)**: The chatbot, in "selected text only" mode, answers questions solely from the provided selection and refuses to use outside knowledge.

### Implementation for RAG API

- [ ] T030 [P] Scaffold FastAPI project in `/rag-api`
- [ ] T031 [P] [US2] Implement `/health` endpoint in `rag-api/app/main.py`
- [ ] T032 [US2] Implement `/chat` endpoint:
  - [ ] T033 [US2] embed query
  - [ ] T034 [US2] retrieve from Qdrant
  - [ ] T035 [US2] format context with citations
  - [ ] T036 [US2] respond with answer + citation list
- [ ] T037 [US3] Implement `/chat/selection` endpoint:
  - [ ] T038 [US3] accept selection text + question
  - [ ] T039 [US3] answer strictly from selection-only
  - [ ] T040 [US3] return citations pointing to selection (or “selection” tag)
- [ ] T041 [P] [US2, US3] Implement `/ingest` (dev/admin) endpoint in `rag-api/app/main.py`:
  - [ ] T042 [P] [US2, US3] trigger indexing script or accept payload
- [ ] T043 [P] [US2, US3] Add CORS config + basic rate limiting in `rag-api/app/main.py`

---

## Phase 5: Chat UI in Docusaurus (US2: Ask the Book Chatbot, US3: Ask Selected Text Only Chatbot)

**Goal (US2)**: Users can ask questions to an embedded RAG chatbot, which provides answers based on the full textbook corpus with citations.
**Independent Test (US2)**: The RAG chatbot responds accurately to questions from the book, including citations, and the health endpoint returns OK.

**Goal (US3)**: Users can highlight specific text on a page and ask the chatbot a question, constraining the chatbot to only use the selected text as context.
**Independent Test (US3)**: The chatbot, in "selected text only" mode, answers questions solely from the provided selection and refuses to use outside knowledge.

### Implementation for Chat UI in Docusaurus

- [ ] T044 [P] [US2, US3] Create React Chat widget component in `book/src/components/ChatWidget/`
- [ ] T045 [P] [US2, US3] Add floating button + panel UI in `book/src/theme/Root.js`
- [ ] T046 [US3] Add selection capture button in `book/src/components/ChatWidget/`
- [ ] T047 [P] [US2, US3] Add mode toggle (full vs selection) in `book/src/components/ChatWidget/`
- [ ] T048 [P] [US2, US3] Render citations as links in `book/src/components/ChatWidget/`
- [ ] T049 [P] [US2, US3] Add config for API base URL in `book/docusaurus.config.js`

---

## Phase 6: Deployment

**Purpose**: Deploy the Docusaurus site and RAG API.

- [ ] T050 [P] GitHub Actions for Docusaurus build + deploy to Pages in `.github/workflows/deploy-book.yml`
- [ ] T051 Document RAG API deployment steps in `infra/README.md`
- [ ] T052 Set environment variables in deployment target for RAG API
- [ ] T053 End-to-end smoke test checklist

---

## Phase 7: Bonus: Reusable Intelligence (Claude Code)

**Purpose**: Integrate Claude Code subagents and skills.

- [ ] T054 Define subagents:
  - [ ] T055 `ChapterWriter`
  - [ ] T056 `LabDesigner`
  - [ ] T057 `Diagrammer` (Mermaid)
  - [ ] T058 `QA/LinkChecker`
  - [ ] T059 `RAGTuner`
- [ ] T060 Define Skills:
  - [ ] T061 “Create chapter from syllabus block”
  - [ ] T062 “Generate lab steps + expected output”
  - [ ] T063 “Convert concept to Mermaid diagram”
  - [ ] T064 “Extract glossary terms”
  - [ ] T065 “Create quiz questions”

---

## Phase 8: Bonus: Auth + Personalization + Urdu

**Purpose**: Implement optional features.

- [ ] T066 Integrate Better Auth (signup/login)
- [ ] T067 Signup questionnaire (software/hardware background)
- [ ] T068 Store profile in Neon
- [ ] T069 Add “Personalize chapter” button:
  - [ ] T070 call backend
  - [ ] T071 render adapted content
  - [ ] T072 cache result
- [ ] T073 Add “Translate to Urdu” button:
  - [ ] T074 call backend
  - [ ] T075 render Urdu content
  - [ ] T076 cache result

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 0)**: No dependencies - can start immediately
- **Docusaurus Book (Phase 1)**: Depends on Setup completion
- **Data Stores (Phase 2)**: Depends on Setup completion
- **Indexing Pipeline (Phase 3)**: Depends on Data Stores completion
- **RAG API (Phase 4)**: Depends on Data Stores completion
- **Chat UI in Docusaurus (Phase 5)**: Depends on Docusaurus Book and RAG API completion
- **Deployment (Phase 6)**: Depends on Docusaurus Book and RAG API completion
- **Bonus Features (Phase 7, 8)**: Can proceed in parallel with other phases, with dependencies as noted internally.

### Within Each User Story

- Tasks generally flow from data models -> services -> API endpoints -> UI integration.
- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- Many tasks within Docusaurus Book (Phase 1) can run in parallel once the initial setup is done.
- Data Stores (Phase 2) tasks can largely run in parallel.
- Within RAG API (Phase 4), individual endpoint implementations can be parallelized.
- Within Chat UI (Phase 5), component creation can be parallelized.
- Different user stories can be worked on in parallel by different team members, especially US1 vs. US2/US3 after foundational components are set up.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 0: Repo & Tooling
2. Complete Phase 1: Docusaurus Book
3. **STOP and VALIDATE**: Test User Story 1 independently (Docusaurus site builds and displays content)
4. Deploy/demo if ready

### Incremental Delivery

1. Complete Phase 0 (Setup)
2. Complete Phase 1 (Docusaurus Book) → Test independently → Deploy/Demo (MVP!)
3. Complete Phase 2 (Data Stores) + Phase 3 (Indexing)
4. Complete Phase 4 (RAG API) + Phase 5 (Chat UI) → Test US2/US3 independently → Deploy/Demo
5. Each major phase adds value without breaking previous functionality

### Parallel Team Strategy

With multiple developers:

1. Team completes Phase 0 (Repo & Tooling) together.
2. Once Phase 0 is done:
   - Developer A: Focus on Phase 1 (Docusaurus Book)
   - Developer B: Focus on Phase 2 (Data Stores) and Phase 3 (Indexing)
   - Developer C: Focus on Phase 4 (RAG API)
3. Once Phase 1, 2, 3, 4 are nearing completion, team can collaborate on Phase 5 (Chat UI) and Phase 6 (Deployment).
4. Bonus features (Phase 7, 8) can be tackled by any available developers in parallel.
5. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
