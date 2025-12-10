# Docusaurus Book App Agents

This repository contains three reusable AI agents designed for a Docusaurus book app:

1. **SummarizerAgent**: Creates short summaries (3-5 lines) from input text/markdown
2. **CodeExampleRunnerAgent**: Executes JS/Python code snippets and returns results
3. **TranslatorAgent**: Translates text while preserving markdown formatting

## Architecture

Each agent is implemented as a separate FastAPI microservice with:
- Dedicated endpoints for specific functionality
- OpenAPI specifications
- Input validation via Pydantic models
- Error handling
- Structured for easy replacement with more sophisticated AI services (Claude/Qwen) later

## Setup and Installation

### Prerequisites
- Python 3.8+
- Node.js (for JavaScript code execution in Code Runner Agent)

### Installation Steps

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd agents
   ```

2. Install and run each agent separately:

   **Summarizer Agent (port 8001)**:
   ```bash
   cd summarizer
   pip install -r requirements.txt
   python main.py
   ```

   **Code Runner Agent (port 8002)**:
   ```bash
   cd ../code_runner
   pip install -r requirements.txt
   python main.py
   ```

   **Translator Agent (port 8003)**:
   ```bash
   cd ../translator
   pip install -r requirements.txt
   python main.py
   ```

### Alternative: Run all agents at once

You can also run all agents in separate terminal windows or use a process manager like `pm2` or a docker-compose file.

## API Endpoints

### SummarizerAgent
- `POST /summarize`: Generate a short summary from input text
- `GET /`: Health check

### CodeExampleRunnerAgent
- `POST /run_code`: Execute a code snippet
- `GET /`: Health check

### TranslatorAgent
- `POST /translate`: Translate text while preserving markdown
- `GET /`: Health check

## Frontend Example

The `example.html` file demonstrates how to call each agent from a frontend application:

1. Open `example.html` in a web browser
2. Make sure all agents are running on their respective ports
3. Use the interfaces to interact with each agent

## OpenAPI Specifications

Each agent includes an OpenAPI specification:
- Summarizer: `summarizer/openapi.json`
- Code Runner: `code_runner/openapi.json`
- Translator: `translator/openapi.json`

These specs can be used with API clients or for documentation generation.

## Security Considerations

- Code Runner Agent executes code in a sandboxed environment with timeouts
- All agents validate input using Pydantic models
- Production deployments should add additional security measures like authentication

## Extending the Agents

Each agent is structured to easily swap in more sophisticated AI services (like Claude or Qwen) by replacing the core processing functions while maintaining the same API contract.