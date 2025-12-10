-- Initial schema for RAG chatbot system

-- Chat logs table
CREATE TABLE IF NOT EXISTS chat_logs (
    id SERIAL PRIMARY KEY,
    session_id VARCHAR(255) NOT NULL,
    user_id VARCHAR(255),
    query TEXT NOT NULL,
    response TEXT NOT NULL,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Feedback table
CREATE TABLE IF NOT EXISTS feedback (
    id SERIAL PRIMARY KEY,
    session_id VARCHAR(255) NOT NULL,
    user_id VARCHAR(255),
    query TEXT NOT NULL,
    response TEXT NOT NULL,
    rating INTEGER CHECK (rating >= 1 AND rating <= 5),
    comment TEXT,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Source clicks table
CREATE TABLE IF NOT EXISTS source_clicks (
    id SERIAL PRIMARY KEY,
    query_id INTEGER,
    source_url TEXT NOT NULL,
    source_title VARCHAR(500),
    user_id VARCHAR(255),
    session_id VARCHAR(255),
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create indexes for better performance
CREATE INDEX IF NOT EXISTS idx_chat_logs_session_id ON chat_logs(session_id);
CREATE INDEX IF NOT EXISTS idx_chat_logs_user_id ON chat_logs(user_id);
CREATE INDEX IF NOT EXISTS idx_chat_logs_timestamp ON chat_logs(timestamp);

CREATE INDEX IF NOT EXISTS idx_feedback_session_id ON feedback(session_id);
CREATE INDEX IF NOT EXISTS idx_feedback_user_id ON feedback(user_id);
CREATE INDEX IF NOT EXISTS idx_feedback_timestamp ON feedback(timestamp);

CREATE INDEX IF NOT EXISTS idx_source_clicks_query_id ON source_clicks(query_id);
CREATE INDEX IF NOT EXISTS idx_source_clicks_user_id ON source_clicks(user_id);
CREATE INDEX IF NOT EXISTS idx_source_clicks_timestamp ON source_clicks(timestamp);