# Neon Serverless Postgres Integration Guide

This document explains how to integrate Neon Serverless Postgres with the RAG backend for analytics and user interaction tracking.

## Overview

The backend now includes:
- A Neon Serverless Postgres database to store user interactions
- Automatic logging of user queries
- Logging of when users click on source links
- An analytics endpoint for admin insights

## Database Schema

### `user_queries` table
Stores information about user queries:
- `id`: SERIAL PRIMARY KEY - Unique identifier for each query
- `query`: TEXT NOT NULL - The actual query text
- `user_id`: VARCHAR(255) - Optional user identifier (for logged in users)
- `session_id`: VARCHAR(255) - Session identifier (for anonymous users)
- `timestamp`: TIMESTAMP WITH TIME ZONE - When the query was made
- `response_time_ms`: INTEGER - How long the query took to process
- `success`: BOOLEAN - Whether the query was processed successfully

### `clicked_sources` table
Stores information about source link clicks:
- `id`: SERIAL PRIMARY KEY - Unique identifier for each click
- `query_id`: INTEGER REFERENCES user_queries(id) - Links to the original query
- `source_url`: TEXT NOT NULL - The URL of the clicked source
- `source_title`: TEXT - Title of the source document
- `user_id`: VARCHAR(255) - Optional user identifier
- `session_id`: VARCHAR(255) - Session identifier
- `timestamp`: TIMESTAMP WITH TIME ZONE - When the click occurred

## Setup Instructions

### 1. Create Neon Database
1. Go to [https://neon.tech](https://neon.tech) and create a free account
2. Create a new project
3. Get your connection string (DATABASE_URL) from the project dashboard

### 2. Configure Environment
Add the following to your `.env` file:
```
NEON_DATABASE_URL=your_neon_database_connection_string_here
```

### 3. Run Migrations
Run the database migration script to create tables:
```bash
python migrate.py
```

## API Endpoints

### POST /log-source-click
Endpoint to log when a user clicks on a source link.

Request body:
```json
{
  "query_id": 123,
  "source_url": "https://example.com/docs/guide",
  "source_title": "Documentation Guide",
  "user_id": "user123",
  "session_id": "session456"
}
```

Response:
```json
{
  "status": "success"
}
```

### POST /analytics
Endpoint to fetch analytics data for admin dashboard.

Response:
```json
{
  "top_queries": [
    {
      "query": "How to install?",
      "count": 45,
      "avg_response_time": 230.5
    }
  ],
  "top_sources": [
    {
      "source_url": "https://example.com/docs/install",
      "source_title": "Installation Guide",
      "click_count": 23
    }
  ],
  "total_queries": 1500,
  "total_clicks": 890
}
```

## Integration with Chat Widget

The chat widget can be enhanced to log source clicks by calling the `/log-source-click` endpoint when a user clicks on a source link:

1. When the RAG system generates an answer, it returns a `query_id` in the trace
2. When the user clicks on a source link in the frontend, send a POST request to `/log-source-click` with the relevant information
3. The backend will store this interaction in the database

Example from the frontend:
```javascript
// Assuming you have the query ID from the backend response
const logSourceClick = async (queryId, sourceUrl, sourceTitle) => {
  try {
    const response = await fetch('/log-source-click', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${API_KEY}`
      },
      body: JSON.stringify({
        query_id: queryId,
        source_url: sourceUrl,
        source_title: sourceTitle
      })
    });
    return response.json();
  } catch (error) {
    console.error('Error logging source click:', error);
  }
};
```

## Using Analytics Data

The analytics endpoint provides valuable insights:
- **Top Queries**: See which questions users ask most frequently
- **Top Sources**: Identify which documentation pages are most helpful
- **Response Times**: Monitor performance of your RAG system
- **User Engagement**: Track how often users click on source links

This data can be used to:
- Improve documentation based on frequently asked questions
- Optimize the RAG system performance
- Identify gaps in documentation
- Understand user behavior patterns