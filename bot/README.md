# RAG Chatbot Microservice

This is a Retrieval-Augmented Generation (RAG) chatbot microservice that allows users to ask questions about documents (PDF, Markdown, URLs) and get grounded answers with citations.

## Features

- **Document Ingestion**: Accept PDF, Markdown, and URL inputs
- **Vector Storage**: Uses Qdrant for efficient similarity search
- **Session Management**: Maintains conversation history
- **Highlight Search**: Query within specific text selections
- **Frontend Widget**: Embeddable chat interface for Docusaurus sites
- **Security**: API key authentication and rate limiting

## Architecture

- **FastAPI**: Web framework and API
- **Qdrant**: Vector database for document embeddings
- **OpenAI**: For text embeddings and generation (with option for local models)
- **PostgreSQL**: For metadata and user highlights (Neon Serverless in production)
- **Redis**: For rate limiting
- **Frontend Widget**: JavaScript widget for Docusaurus integration

## Setup

### Prerequisites

- Docker and Docker Compose
- OpenAI API key (or local embedding model option)
- Qdrant Cloud account (for production) or local instance

### Environment Variables

Create a `.env` file with the following variables:

```bash
# API Keys
OPENAI_API_KEY=your_openai_api_key
QDRANT_API_KEY=your_qdrant_api_key
NEON_DB_URL=your_neon_db_connection_string
EXPECTED_API_KEY=your_expected_api_key

# URLs
QDRANT_URL=your_qdrant_cloud_url  # For production, or http://qdrant:6333 for local dev
NEON_DB_URL=your_neon_db_url

# Configuration
EMBEDDING_MODEL=text-embedding-3-small
USE_LOCAL_EMBEDDINGS=false  # Set to true to use local embedding model
OPENAI_MODEL=gpt-3.5-turbo  # or gpt-4
```

### Local Development with Docker Compose

1. Make sure you have Docker and Docker Compose installed
2. Create your `.env` file as described above
3. Run the services:

```bash
# For local development with local Qdrant
docker-compose up -d

# For production with Qdrant Cloud, set QDRANT_URL in .env to your cloud instance
# and remove the qdrant service from docker-compose.yml
```

The API will be available at `http://localhost:8000`

### Production Deployment with Qdrant Cloud and Neon

1. Create a Qdrant Cloud account at [qdrant.tech](https://qdrant.tech/)
2. Create a Neon Serverless PostgreSQL database
3. Update your `.env` file with:
   - QDRANT_URL: Your Qdrant Cloud instance URL
   - QDRANT_API_KEY: Your Qdrant API key
   - NEON_DB_URL: Your Neon database connection string
4. Deploy using Docker or your preferred cloud platform

## API Endpoints

### POST /ingest
Ingest documents (PDF, Markdown, URLs) into the system.

Request body:
```json
{
  "url": "https://example.com/document.pdf",
  "file_url": "https://example.com/file.pdf",
  "source_type": "pdf"
}
```

### POST /query
Query the knowledge base.

Request body:
```json
{
  "query": "Your question here",
  "highlight_text": "Optional text to search within",
  "session_id": "Optional session ID"
}
```

### POST /chat
Chat with conversation memory.

Request body:
```json
{
  "message": "Your message here",
  "session_id": "Optional session ID",
  "highlight_text": "Optional text to search within"
}
```

## Frontend Widget Integration

To add the chatbot to your Docusaurus site:

1. Add the JavaScript widget to your Docusaurus site:
   ```html
   <script src="path/to/chatbot.js"></script>
   ```

2. Optionally configure the widget:
   ```javascript
   window.RAG_CHATBOT_CONFIG = {
     apiEndpoint: 'http://localhost:8000',
     apiKey: 'your_api_key'
   };
   ```

The widget provides:
- Floating chat icon
- On-page "Ask about this selection" button
- Conversation history
- Source citations

## Security

- API key authentication required for all endpoints
- Rate limiting (30 requests per minute per IP for chat/query, 10 for ingestion)
- Input validation and sanitization
- Proper CORS configuration (configure origins in production)

## Development

To run locally without Docker:

1. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set environment variables (see .env.example)

3. Run the application:
   ```bash
   uvicorn fastapi_app:app --reload
   ```

## Configuration Options

- `USE_LOCAL_EMBEDDINGS`: Set to "true" to use local sentence transformer models instead of OpenAI
- `EMBEDDING_MODEL`: OpenAI embedding model to use
- `OPENAI_MODEL`: OpenAI model for text generation
- `EXPECTED_API_KEY`: Set the expected API key for authentication
- Various other parameters can be configured in the environment

## Troubleshooting

- Make sure all required environment variables are set
- Check that your Qdrant instance is accessible and the API key is correct
- Verify that your OpenAI API key is valid and has sufficient quota
- Check the container logs with `docker-compose logs` if using Docker

## Scaling Considerations

- Use Qdrant Cloud for production workloads
- Implement connection pooling for PostgreSQL
- Consider caching for frequently requested content
- Monitor and adjust rate limits based on your needs
- Use a CDN for the frontend widget in production