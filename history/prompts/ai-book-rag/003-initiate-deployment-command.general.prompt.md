---
ID: 003
TITLE: Initiate Deployment Command
STAGE: general
DATE_ISO: 2025-12-03
SURFACE: agent
MODEL: claude-sonnet-4-5-20250929
FEATURE: ai-book-rag
BRANCH: 001-photo-albums
USER: user
COMMAND: /sp.deploy
LABELS: ["deployment", "initiation"]
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
FILES_YAML: []
TESTS_YAML: []
---

### Prompt

```text
/sp.deploy
```

### Response

```text
The custom deployment script .specify/scripts/bash/deploy.sh was not found. The system is ready to deploy Docusaurus using `npm run deploy` and can assist with containerizing the FastAPI backend and Qdrant.
```