# Smoke Test Checklist

## Docusaurus Book
- [ ] Access Docusaurus site via GitHub Pages URL.
- [ ] Navigate to several chapters and verify content display.
- [ ] Verify navbar and footer links are functional.
- [ ] Check sidebar navigation for all modules/weeks.
- [ ] Verify image and diagram rendering (e.g., Mermaid diagrams).
- [ ] Test search functionality.

## RAG Chatbot (Frontend)
- [ ] Open the chat widget on a docs page.
- [ ] Send a query in "Ask the book" mode and verify a relevant response.
- [ ] Verify citations are rendered as clickable links.
- [ ] Select a text snippet, switch to "Ask selected text" mode, and send a query.
- [ ] Verify the chatbot responds strictly from the selected text, with relevant citations (or "selection" tag).
- [ ] Verify error handling for API unavailability (e.g., if RAG API is down).

## RAG Backend (FastAPI)
- [ ] Access the `/health` endpoint (e.g., `http://localhost:8000/health`) and verify all services (database, Qdrant, Gemini) report "ok".
- [ ] Send a POST request to `/chat` with a query and verify a 200 OK response with an answer and citations.
- [ ] Send a POST request to `/chat/selection` with selected text and a query, and verify a 200 OK response with an answer and citations.
- [ ] Verify the `/ingest` endpoint returns the expected message (manual_ingestion_required).
- [ ] Verify CORS headers are correctly set when accessing from the Docusaurus frontend domain.
- [ ] Test rate limiting (if enabled) by sending many requests.

## End-to-End
- [ ] Deploy the Docusaurus site to GitHub Pages.
- [ ] Deploy the RAG API to its target environment.
- [ ] Verify the deployed Docusaurus site can successfully interact with the deployed RAG API.
