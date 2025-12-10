# RAG Chatbot System for Docusaurus Book Website

This is a complete RAG (Retrieval Augmented Generation) chatbot system for your Docusaurus book website with the following features:

- Floating chat widget with streaming responses
- Right-click context menu for "Ask AI about this text"
- Integration with Qdrant Cloud for vector storage
- Neon Postgres for chat logs and feedback
- Support for both OpenAI and Qwen models

## Project Structure

```
backend/
├── rag_backend/
│   ├── main.py          # FastAPI server with RAG endpoints
│   ├── models.py        # Pydantic models
│   ├── database.py      # Database connection and models
│   ├── rag_agent.py     # RAG agent with Qdrant integration
│   └── ingestion.py     # Markdown ingestion script
src/components/ChatWidget/
├── ChatWidget.jsx       # React chat widget component
└── ChatWidget.css       # Styling for the chat widget
migrations/
└── 001_initial_schema.sql  # Database migration
.env.example             # Environment variables template
```

## Installation

1. Install Python dependencies:
```bash
pip install -r backend/requirements.txt
```

2. Install Node.js dependencies (if not already installed):
```bash
npm install
```

3. Set up environment variables:
```bash
cp .env.example .env
# Edit .env with your actual API keys and URLs
```

## Environment Variables

- `OPENAI_API_KEY`: Your OpenAI API key (or `QWEN_API_KEY` for Qwen)
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `DATABASE_URL`: Your Neon Postgres connection string
- `NEON_DATABASE_URL`: Your Neon Postgres connection string
- `DOCS_PATH`: Path to your documentation files (default: "docs")

## How to Run Locally

### 1. Start the Backend Server

```bash
cd backend
python -m uvicorn rag_backend.main:app --reload --port 8000
```

### 2. Ingest Documentation

```bash
cd backend
python rag_backend/ingestion.py
```

### 3. Start the Docusaurus Frontend

```bash
npm start
```

The chat widget will automatically appear on your Docusaurus pages.

## API Endpoints

- `POST /api/query` - Normal RAG query
- `POST /api/query-selected` - Query with selected text context
- `POST /api/add-document` - Add a document to the vector store
- `GET /api/health` - Health check

## Database Schema

The system uses Neon Postgres with the following tables:

- `chat_logs` - Stores conversation history
- `feedback` - Stores user feedback and ratings
- `source_clicks` - Tracks which sources users click

## Troubleshooting

### 404 Errors
- Make sure the backend server is running on port 8000
- Check that the API endpoints are correctly configured in the frontend
- Verify CORS settings in the backend

### CORS Issues
- In development, the backend allows all origins
- In production, update the CORS settings in `backend/rag_backend/main.py`

### Qdrant Auth Failures
- Verify that `QDRANT_URL` and `QDRANT_API_KEY` are correct
- Check that your Qdrant Cloud cluster is accessible
- Ensure the collection "book_chunks" exists

### FastAPI Timeout
- Increase timeout values in the Qdrant client configuration
- Check your network connection to Qdrant Cloud
- Verify that your API provider (OpenAI/Qwen) is accessible

### Docusaurus Plugin Not Loading
- Verify that the ChatWidget component is properly imported
- Check browser console for JavaScript errors
- Ensure all dependencies are installed

## Deployment

### Backend (FastAPI)
Deploy to any platform that supports Python (Heroku, Railway, etc.)

### Frontend (Docusaurus)
Deploy to Vercel, Netlify, or GitHub Pages as usual

Remember to update the API URL in the ChatWidget component to point to your deployed backend.

## Additional Features

- Context-aware responses using selected text
- Streaming responses for better UX
- Expert skill selection for specific tasks
- Source citation with click-to-highlight functionality
- Rate limiting and error handling
- Subagent system for specialized tasks