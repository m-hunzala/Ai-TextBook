# RAG Backend

A Retrieval Augmented Generation (RAG) backend built with FastAPI, integrating with Qdrant vector store and Google's Gemini embeddings.

## Features

- Query documents with semantic search
- Embed and upsert documents to vector store
- Generate answers using RAG methodology
- Better-Auth authentication system
- User profile collection during signup
- Support for both Google Gemini and OpenAI embedding providers
- Neon Serverless Postgres integration for analytics
- Track user queries, source clicks, and user interactions
- Analytics endpoint for admin insights
- Claude Code Subagents for specialized tasks
- Expert skill selector in chat UI

## Prerequisites

- Python 3.8+
- Qdrant Cloud account (free tier available)
- Google Gemini API key OR OpenAI API key
- Neon Serverless Postgres database (free tier available)

## Setup

### 1. Clone and navigate to backend directory

```bash
cd backend/rag_backend
```

### 2. Install dependencies

```bash
pip install -r requirements.txt
```

### 3. Database Setup

1. Create a free Neon Serverless Postgres database at [neon.tech](https://neon.tech)
2. Copy your connection string (DATABASE_URL)
3. Run the database migrations:

```bash
python migrate.py
```

This will create the necessary tables for storing user queries and source clicks.

### 4. Environment Configuration

Copy the example environment file and fill in your credentials:

```bash
cp .env.example .env
```

Then edit `.env` with your values:

```
# Qdrant Configuration
QDRANT_URL=your-qdrant-cluster-url
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=documents

# Embedding Provider (either Gemini or OpenAI)
GEMINI_API_KEY=your-gemini-api-key
# OR
OPENAI_API_KEY=your-openai-api-key

# Model Configuration for Answer Generation
GEMINI_MODEL_NAME=gemini-pro
GEMINI_TEMPERATURE=0.7
GEMINI_MAX_TOKENS=2048

# Claude Code API Configuration
ANTHROPIC_API_KEY=your-anthropic-api-key

# Database Configuration (Neon Serverless Postgres)
NEON_DATABASE_URL=your_neon_database_url_here

# API Key for Authentication
API_KEY=your-secret-api-key-here

# Document source directory (relative to the project root)
DOCS_PATH=../../docs
```

## Usage

### 1. Run the server

```bash
python main.py
```

Or with uvicorn:

```bash
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

Server will start on `http://localhost:8000`

### 2. Ingest documents

Before querying, you need to embed and upsert your documents:

```bash
curl -X POST "http://localhost:8000/embed-upsert" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json"
```

### 3. Query documents

```bash
curl -X POST "http://localhost:8000/query" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the main topics covered?",
    "top_k": 5,
    "context_ids": null
  }'
```

### 4. Get an answer with RAG

```bash
# Full RAG (retrieve from stored documents)
curl -X POST "http://localhost:8000/answer" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Summarize the key concepts",
    "selected_text": null
  }'

# With specific text
curl -X POST "http://localhost:8000/answer" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept",
    "selected_text": "The concept involves..."
  }'

# Get streaming response (for real-time responses)
curl -X POST "http://localhost:8000/answer-stream" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the main topics covered?",
    "selected_text": null
  }'

# Log a source click
curl -X POST "http://localhost:8000/log-source-click" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "query_id": 123,
    "source_url": "https://yoursite.com/docs/installation",
    "source_title": "Installation Guide",
    "user_id": "user123",
    "session_id": "session456"
  }'

# Get analytics data (for admin)
curl -X POST "http://localhost:8000/analytics" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json"

# List available subagents
curl -X GET "http://localhost:8000/subagents" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json"

# Call a subagent
curl -X POST "http://localhost:8000/subagent-call" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "subagent_name": "ros-helper",
    "query": "Create a ROS2 publisher node in Python"
  }'
```

### 6. Alternative authentication using API key header

Instead of using Bearer token, you can also use API key in header:

```bash
curl -X POST "http://localhost:8000/query" \
  -H "Authorization: YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the main topics covered?",
    "top_k": 3
  }'
```

## API Endpoints

- `POST /query` - Retrieve relevant passages based on query
- `POST /embed-upsert` - Process documents and store embeddings
- `POST /answer` - Generate answer using RAG methodology (returns trace)
- `POST /answer-stream` - Generate answer with streaming response
- `POST /log-source-click` - Log when user clicks on source link
- `POST /analytics` - Fetch top queries and sources for admin
- `GET /subagents` - List available Claude Code subagents
- `POST /subagent-call` - Call a specific subagent
- `GET /auth/user-info` - Get current user's basic information
- `GET /auth/user-profile` - Get current user's profile
- `POST /auth/user-profile` - Update user profile
- `POST /personalization/personalize` - Personalize chapter content based on user profile
- `GET /personalization/content/{chapter_url}` - Get personalized content
- `DELETE /personalization/content/{chapter_url}` - Clear personalized content
- `POST /translation/translate` - Translate chapter content to Urdu
- `GET /translation/content/{chapter_url}/{target_language}` - Get translated content
- `DELETE /translation/content/{chapter_url}/{target_language}` - Clear translation
- `GET /translation/languages/{chapter_url}` - Get available languages for chapter
- `GET /` - Health check

## Database Schema

The application uses Neon Serverless Postgres with the following tables:

### `user_queries`
- `id`: SERIAL PRIMARY KEY
- `query`: TEXT NOT NULL - The user's query
- `user_id`: VARCHAR(255) - Optional user identifier
- `session_id`: VARCHAR(255) - Session identifier
- `timestamp`: TIMESTAMP WITH TIME ZONE - When the query was made
- `response_time_ms`: INTEGER - Response time in milliseconds
- `success`: BOOLEAN - Whether the query was successful

### `clicked_sources`
- `id`: SERIAL PRIMARY KEY
- `query_id`: INTEGER REFERENCES user_queries(id) - Links to the original query
- `source_url`: TEXT NOT NULL - URL of the clicked source
- `source_title`: TEXT - Title of the source
- `user_id`: VARCHAR(255) - Optional user identifier
- `session_id`: VARCHAR(255) - Session identifier
- `timestamp`: TIMESTAMP WITH TIME ZONE - When the click occurred

## API Endpoints for Analytics

### POST /log-source-click
Logs when a user clicks on a source link in the chat widget.
```json
{
  "query_id": 123,
  "source_url": "https://example.com/docs/guide",
  "source_title": "Documentation Guide",
  "user_id": "user123",
  "session_id": "session456"
}
```

### POST /analytics
Fetches analytics data for admin dashboard.
Response:
```json
{
  "top_queries": [
    {"query": "How to install", "count": 45, "avg_response_time": 230.5}
  ],
  "top_sources": [
    {"source_url": "https://example.com/docs/install", "source_title": "Installation Guide", "click_count": 23}
  ],
  "total_queries": 1500,
  "total_clicks": 890
}
```

## Architecture

The system consists of:

1. **Document Extraction**: Extracts Markdown documents from Docusaurus sites
2. **Text Chunking**: Splits documents into overlapping chunks of ~500 tokens
3. **Embedding Provider**: Generates embeddings using Google Gemini or OpenAI
4. **Vector Store**: Stores embeddings in Qdrant for efficient similarity search
5. **RAG System**: Combines retrieval and generation to answer queries
6. **Answer Generation**: Uses Google Gemini with context enforcement and code extraction
7. **Database Integration**: Stores user queries and source clicks in Neon Postgres
8. **Analytics**: Provides insights through admin endpoint
9. **Streaming Support**: Optional streaming responses for improved UX
10. **Authentication**: Secures endpoints with API key

## Development

To run in development mode with auto-reload:

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

## Troubleshooting

- Ensure all environment variables are properly set
- Check that Qdrant cluster is accessible
- Verify API keys are valid and have proper permissions
- Confirm document paths exist and are readable