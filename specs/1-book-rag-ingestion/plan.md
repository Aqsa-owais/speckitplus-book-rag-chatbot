# Implementation Plan: Book RAG Content Ingestion Pipeline

**Branch**: `1-book-rag-ingestion` | **Date**: 2025-12-25 | **Spec**: [specs/1-book-rag-ingestion/spec.md](../specs/1-book-rag-ingestion/spec.md)
**Input**: Feature specification from `/specs/1-book-rag-ingestion/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a content ingestion pipeline that crawls deployed book URLs, extracts clean text, generates Cohere embeddings, and stores them in Qdrant Cloud with metadata. The pipeline will be built in Python using uv package manager with token-aware chunking and configurable parameters for repeatable execution.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: requests, beautifulsoup4, cohere, qdrant-client, python-dotenv, tokenizers
**Storage**: Qdrant Cloud (vector database)
**Testing**: pytest (for unit and integration tests)
**Target Platform**: Linux/Mac/Windows server environment
**Project Type**: Backend service/cli tool
**Performance Goals**: Process 100 pages within 30 minutes
**Constraints**: <200ms p95 for embedding generation, <1GB memory for typical book processing, offline-capable for local testing
**Scale/Scope**: Support books with up to 1000 pages, 10,000+ content chunks

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution principles:
- Technical explanations will be correct and verified against official documentation
- Code will be clear, readable, and well-commented
- Content will follow structured, chapter-based organization
- Implementation will align with project specification
- Markdown will be compatible with Docusaurus
- Real-world examples and practical applications will be emphasized

## Project Structure

### Documentation (this feature)

```text
specs/1-book-rag-ingestion/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── main.py              # Main entry point with orchestration logic
├── requirements.txt     # Project dependencies
├── .env.example         # Environment variables template
├── .gitignore           # Git ignore rules
├── pyproject.toml       # Project configuration for uv
├── src/
│   ├── __init__.py
│   ├── crawler.py       # URL collection and web crawling functionality
│   ├── extractor.py     # Text extraction and preprocessing functionality
│   ├── chunker.py       # Token-aware text chunking with overlap
│   ├── embedder.py      # Embedding generation using Cohere
│   └── storage.py       # Qdrant storage and retrieval functionality
├── config/
│   └── settings.py      # Configuration management
├── utils/
│   ├── __init__.py
│   ├── validators.py    # Input validation utilities
│   └── helpers.py       # General helper functions
└── tests/
    ├── __init__.py
    ├── test_crawler.py
    ├── test_extractor.py
    ├── test_chunker.py
    ├── test_embedder.py
    └── test_storage.py
```

**Structure Decision**: Backend service structure chosen to match the requirement for creating a `backend/` folder with a single entry file (`backend/main.py`) that orchestrates the entire ingestion pipeline. The modular design separates concerns into distinct modules following clean architecture principles.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple module files | Maintainability and separation of concerns | Single file approach would create unmaintainable monolith |
| External dependencies | Required for core functionality (web crawling, embeddings, vector storage) | Building from scratch would be impractical and error-prone |