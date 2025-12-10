# Book RAG Chatbot

A Retrieval-Augmented Generation (RAG) chatbot that answers questions about published books using vector search and OpenAI's language models.

## Features

- **Book Ingestion**: Upload and process book content for semantic search
- **Semantic Search**: Query book content using natural language
- **Selected Text Queries**: Ask questions about specific text selections
- **Web Interface**: Clean and intuitive chat interface
- **Analytics**: Query logging in PostgreSQL
- **Scalable Architecture**: Uses Qdrant for vector storage and Neon Postgres for metadata

## Architecture

- **Backend**: FastAPI with async support
- **Vector Database**: Qdrant Cloud for semantic search
- **Relational Database**: Neon Postgres for metadata and analytics
- **Embeddings & LLM**: OpenAI (Embeddings and Chat Completions)
- **Frontend**: HTML/CSS/JS with chat widget and text selection

## Prerequisites

- Python 3.8+
- OpenAI API key
- Qdrant Cloud account and API key
- Neon Postgres account and connection string

## Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd rag-chatbot
```

### 2. Install Dependencies

```bash
pip install -r requirements.txt
```

### 3. Environment Variables

Create a `.env` file based on `.env.example`:

```bash
cp .env.example .env
```

Update the `.env` file with your actual credentials:

```env
# OpenAI Settings
OPENAI_API_KEY=your_openai_api_key_here

# Qdrant Cloud Settings
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_chunks

# Neon Postgres Settings
DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require

# Application Settings
DEBUG=False
```

### 4. Run the Application

```bash
# Using uvicorn directly
uvicorn app.main:app --reload

# Or using the main script
python -m app.main
```

The application will start on `http://localhost:8000`.

### 5. Docker Setup (Alternative)

Build and run with Docker:

```bash
# Build the image
docker build -t rag-chatbot .

# Run the container
docker run -p 8000:8000 --env-file .env rag-chatbot
```

## API Endpoints

### Health Check
```
GET /health
```
Check if the API is running.

### Ingest Book Content
```
POST /api/v1/ingest
```
Ingest book content into the vector database.

Request body:
```json
{
  "title": "Book Title",
  "author": "Author Name",
  "text": "Full text content of the book...",
  "content_hash": "optional hash of content"
}
```

### Query Book Content
```
POST /api/v1/query
```
Query the book content using semantic search.

Request body:
```json
{
  "query": "Your question about the book",
  "top_k": 5
}
```

### Query Selected Text
```
POST /api/v1/query_selected
```
Query using only the user-provided selected text.

Request body:
```json
{
  "query": "Your question",
  "selected_text": "The text you want to ask about"
}
```

## Example Usage

### 1. Ingest a Book

Using curl:
```bash
curl -X POST http://localhost:8000/api/v1/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "title": "Example Book",
    "author": "John Doe",
    "text": "This is the full content of the book..."
  }'
```

### 2. Query the Book

Using curl:
```bash
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is the main theme of this book?",
    "top_k": 3
  }'
```

### 3. Query Selected Text

Using curl:
```bash
curl -X POST http://localhost:8000/api/v1/query_selected \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept",
    "selected_text": "The specific text content you want to ask about..."
  }'
```

## Frontend Interface

The application provides a web interface at `http://localhost:8000` with:

- Chat interface for asking questions
- Book ingestion form
- "Ask about selected text" feature
- Real-time response display

## Configuration

### Environment Variables

- `OPENAI_API_KEY`: Your OpenAI API key (required)
- `QDRANT_URL`: URL to your Qdrant Cloud instance (required)
- `QDRANT_API_KEY`: Your Qdrant API key (required)
- `QDRANT_COLLECTION`: Name of the collection to store book chunks (default: book_chunks)
- `NEON_DATABASE_URL`: Neon Postgres connection string (required)
- `DATABASE_URL` (fallback): Alternative Postgres connection string if NEON_DATABASE_URL is not set
- `DEBUG`: Enable debug mode (default: False)
- `EMBED_MODEL`: Embedding model to use (default: text-embedding-ada-002)
- `CHUNK_SIZE`: Size of text chunks (default: 800)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 100)

### Setting up Qdrant Cloud

1. **Sign up for Qdrant Cloud**:
   - Go to [qdrant.tech](https://qdrant.tech/) and sign up for an account
   - Create a new cloud project
   - Select your preferred region and plan

2. **Get your Qdrant credentials**:
   - After creating your project, you'll see your cluster details
   - Note the "Cluster URL" - this is your `QDRANT_URL`
   - Find your "API Key" in the project settings - this is your `QDRANT_API_KEY`

3. **Required Qdrant Collection Configuration**:
   - Collection name: `book_chunks` (or your custom name via `QDRANT_COLLECTION` env var)
   - Vector configuration:
     - Size: 1536 (for OpenAI's text-embedding-ada-002 model)
     - Distance: Cosine
   - Payload schema: Must support fields for book_id, chunk_id, text, page, chapter, etc.

4. **Creating Collection via HTTP API (example)**:
   ```bash
   curl -X PUT 'https://your-cluster-url.qdrant.tech/collections/book_chunks' \
     -H 'Content-Type: application/json' \
     -H 'api-key: YOUR_QDRANT_API_KEY' \
     -d '{
       "vectors": {
         "size": 1536,
         "distance": "Cosine"
       }
     }'
   ```

### Setting up Neon Serverless

1. **Sign up for Neon**:
   - Go to [neon.tech](https://neon.tech/) and create an account
   - Create a new project

2. **Get your DATABASE_URL**:
   - In your Neon project dashboard, go to the "Connection Details" section
   - Select your preferred connection method (e.g., "Command Line")
   - Copy the full connection string - this is your `NEON_DATABASE_URL`
   - Format: `postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require`

3. **Required Postgres Tables**:
   The application will automatically create the required tables:
   - `book_chunks` table for storing chunk metadata
   - `books` table for storing book metadata
   - `queries` table for storing query logs

### Text Processing

The ingestion process automatically:
- Splits text into configurable chunk sizes (default: 800 characters)
- Generates embeddings for each chunk using your specified OpenAI model
- Stores chunks in Qdrant vector database with proper metadata
- Records metadata in Neon Postgres database

## Security & Deployment

### Security Best Practices

1. **Environment Variables for Secrets**:
   - Store all sensitive information (API keys, database URLs) in environment variables
   - Never commit `.env` files to version control
   - Use `.env.example` for documentation only
   - Consider using a secrets manager in production environments

2. **Rate Limiting**:
   - The `/query` and `/query_selected` endpoints include built-in rate limiting to prevent abuse
   - Rate limiter in `ingest.py` prevents exceeding OpenAI API limits (3 requests/sec by default)
   - For additional protection, consider implementing rate limiting at the network level (e.g., with nginx)

3. **Authentication**:
   - Add API key authentication by requiring an `X-API-Key` header
   - Example middleware implementation:

   ```python
   from fastapi import HTTPException, Header
   import os

   API_KEY = os.getenv("API_KEY")

   def verify_api_key(x_api_key: str = Header(None)):
       if API_KEY and x_api_key != API_KEY:
           raise HTTPException(status_code=401, detail="Invalid API Key")
   ```
   - Add the API key check to your endpoints by including it as a dependency

4. **Input Validation**:
   - All endpoints validate input using Pydantic models
   - Text inputs are sanitized and length-limited
   - File paths in ingestion are validated to prevent path traversal

### Docker & Containerization

#### Dockerfile
```dockerfile
# Use Python 3.11 slim image
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements file
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Create non-root user for security
RUN useradd --create-home --shell /bin/bash app \
    && chown -R app:app /app
USER app

# Expose port
EXPOSE 8000

# Run the application
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

#### Docker Compose for Development
```yaml
version: '3.8'
services:
  app:
    build: .
    ports:
      - "8000:8000"
    environment:
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - QDRANT_URL=${QDRANT_URL}
      - QDRANT_API_KEY=${QDRANT_API_KEY}
      - NEON_DATABASE_URL=${NEON_DATABASE_URL}
      - QDRANT_COLLECTION=book_chunks
      - DEBUG=true
    env_file:
      - .env
    depends_on:
      - qdrant
    restart: unless-stopped

  qdrant:
    image: qdrant/qdrant:latest
    ports:
      - "6333:6333"
      - "6334:6334"
    volumes:
      - qdrant_data:/qdrant/storage
    environment:
      - QDRANT_API_KEY=${QDRANT_API_KEY}
    restart: unless-stopped

volumes:
  qdrant_data:
```

### Production Deployment Recommendations

1. **Hosting Platforms**:
   - **Vercel**: For serverless deployments of FastAPI with short-lived requests
   - **Render**: For container-based deployments with managed scaling
   - **DigitalOcean**: For flexible VPS or App Platform deployments
   - **AWS/Azure/GCP**: For full control and custom configurations

2. **Infrastructure**:
   - Use Qdrant Cloud for vector database (production-grade, scalable)
   - Use Neon Serverless for Postgres (auto-scaling, serverless)
   - Use a CDN for static assets (if serving frontend from the API)

3. **Monitoring & Logging**:
   - Implement structured logging with appropriate log levels
   - Monitor API usage and performance metrics
   - Set up alerts for failed requests or high latency

### CI/CD Security

1. **Encrypt Environment Variables**:
   - **GitHub Actions**: Use repository secrets (`Settings > Secrets and variables > Actions`)
   - **GitLab CI**: Use encrypted variables (`Settings > CI/CD > Variables`)
   - **CircleCI**: Use context variables (`Settings > Contexts`)
   - **Vercel**: Use environment variables in project settings

2. **Secret Management**:
   ```bash
   # Example for GitHub Actions secrets setup:
   # In your GitHub repo: Settings > Secrets and variables > Actions
   # Add these secrets:
   # OPENAI_API_KEY
   # QDRANT_API_KEY
   # QDRANT_URL
   # NEON_DATABASE_URL
   ```

3. **Build Security**:
   - Scan dependencies for vulnerabilities (using tools like `pip-audit`)
   - Run security tests as part of CI pipeline
   - Use multi-stage builds to minimize attack surface

### Additional Security & Best Practices

1. **Network Security**:
   - Use HTTPS/TLS in production
   - Implement CORS with specific origins (not wildcard in production)
   - Use a Web Application Firewall (WAF) if needed

2. **API Security**:
   - Implement request size limits to prevent abuse
   - Add authentication/authorization for sensitive operations
   - Use proper error messages that don't leak system information

3. **Data Security**:
   - Encrypt sensitive data in transit (all external APIs use HTTPS)
   - Consider encrypting sensitive data at rest if required
   - Implement proper data retention policies

4. **Privacy Considerations**:
   - If your book contains paid content, ensure Qdrant collection access is private
   - Never embed API keys in client-side code
   - Use environment variables for all sensitive credentials
   - Consider implementing API key authentication for production use

5. **Selected-Text Only Best Practices**:
   - Use a strict system instruction: "You are a helpful assistant. ONLY use the text inside the CONTEXT markers to answer. If the answer is not present, reply exactly: 'I don't know based on the provided text.'"
   - Set temperature to 0.0 to prevent hallucination beyond the provided selected text
   - These measures ensure the model refuses to generate content outside the provided context

6. **Cost Optimization for Embeddings**:
   - Pre-embed all chunks once at ingest time and reuse them
   - This prevents recomputing embeddings for the same content multiple times
   - Embedding costs can be significant, so reusing precomputed embeddings saves costs

7. **Citations**:
   - The /query endpoint includes chunk_id and short excerpt to let the frontend show "source" buttons that jump to book location
   - This provides transparency about where the information came from in the original book

## Running the Application

### Local Development

1. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

2. **Set up environment variables**:
   ```bash
   cp .env.example .env
   # Edit .env with your actual credentials
   ```

3. **Run the application**:
   ```bash
   # Option 1: Using uvicorn directly
   uvicorn app.main:app --reload

   # Option 2: Using the run script
   python run.py --reload

   # Option 3: Using Python module
   python -m app.main
   ```

4. **Run with Docker** (if preferred):
   ```bash
   # Build and run with Docker
   docker build -t rag-chatbot .
   docker run -p 8000:8000 --env-file .env rag-chatbot
   ```

5. **Run tests**:
   ```bash
   # Run the query_selected tests
   python -m pytest tests/test_query_selected.py -v

   # Or run the test directly
   python tests/test_query_selected.py
   ```

The application will start on `http://localhost:8000` with the API endpoints available.

### Using the Ingestion Script

To ingest a book:
```bash
python -m app.ingest --file path/to/book.pdf --book-id "unique_book_id"
```

## Troubleshooting

### Common Issues

- **OpenAI API Error**: Verify your API key is correct and has sufficient quota
- **Qdrant Connection Error**: Check your Qdrant URL and API key
- **PostgreSQL Connection Error**: Verify your connection string and credentials

### Enable Debug Mode

Set `DEBUG=True` in your environment to get more detailed error messages.

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a pull request

## License

[MIT License](LICENSE)