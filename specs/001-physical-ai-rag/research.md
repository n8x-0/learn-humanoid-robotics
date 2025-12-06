# Research for Backend API Testing Plan

## Decision: Testing Framework for FastAPI
**Rationale**: `pytest` is the de-facto standard for Python testing, with excellent integration for FastAPI via `pytest-fastapi` or directly using `TestClient` from `fastapi.testclient`.
**Alternatives considered**: `unittest` (less idiomatic for modern Python), `nose2`.

## Decision: Mocking Strategy for External Services (Qdrant, Neon Postgres)
**Rationale**: For unit and integration tests of the FastAPI application logic, external dependencies like Qdrant and Neon Postgres should be mocked to ensure tests are fast, isolated, and deterministic. Python's `unittest.mock` module (or `pytest-mock`) is suitable for this.
**Alternatives considered**: Using real services (too slow, expensive, and flaky for unit/integration tests); containerized services (more complex setup, still slower).

## Decision: Testing Qdrant Interactions
**Rationale**: While Qdrant itself should be mocked for FastAPI unit tests, dedicated integration tests for the Qdrant client/logic would involve either an in-memory Qdrant instance (if available and reliable for testing) or a lightweight Dockerized Qdrant instance. For initial planning, mocking is preferred for speed and simplicity in the FastAPI tests.
**Alternatives considered**: Full Qdrant Cloud environment (overkill for most tests).

## Decision: Testing Neon Postgres Interactions
**Rationale**: Similar to Qdrant, Neon Postgres interactions will be mocked for FastAPI unit/integration tests. For dedicated database logic testing, `pytest-asyncio` with a temporary in-memory SQLite database or a test-specific Postgres container (e.g., via `testcontainers`) would be appropriate. Again, mocking is prioritized for FastAPI application tests.
**Alternatives considered**: Direct connection to development Neon Postgres (risk of data corruption, slow).

## Decision: Testing OpenAI Agents SDK (Gemini Adapter)
**Rationale**: The custom Gemini adapter for the OpenAI Agents SDK should be thoroughly unit tested independently. For the FastAPI service, the interaction with this adapter and the underlying LLM call should be mocked. This ensures that the FastAPI tests focus on routing, data handling, and business logic, not the external LLM behavior.
**Alternatives considered**: Live LLM calls (expensive, slow, non-deterministic).

## Decision: Test Data Management
**Rationale**: For integration tests that involve data, define clear test data fixtures (e.g., using `pytest` fixtures) that can set up and tear down known states in mocked or temporary databases.
**Alternatives considered**: Manual data setup/teardown (error-prone, not reproducible).