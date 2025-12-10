-- Schema for Neon Serverless Postgres
-- Better-Auth users table (following Better-Auth schema requirements)
CREATE TABLE IF NOT EXISTS users (
    id SERIAL PRIMARY KEY,
    email VARCHAR(255) UNIQUE NOT NULL,
    email_verified BOOLEAN DEFAULT FALSE,
    name VARCHAR(255),
    image TEXT,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Better-Auth accounts table (for OAuth providers)
CREATE TABLE IF NOT EXISTS accounts (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
    provider_id VARCHAR(255) NOT NULL,
    provider_user_id VARCHAR(255) NOT NULL,
    access_token TEXT,
    refresh_token TEXT,
    id_token TEXT,
    expires_at TIMESTAMP WITH TIME ZONE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(user_id, provider_id, provider_user_id)
);

-- Better-Auth sessions table
CREATE TABLE IF NOT EXISTS sessions (
    id VARCHAR(255) PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Better-Auth verification tokens table
CREATE TABLE IF NOT EXISTS verification_tokens (
    id VARCHAR(255) PRIMARY KEY,
    identifier VARCHAR(255) NOT NULL,
    value VARCHAR(255) NOT NULL,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(identifier, value)
);

-- User profiles table for storing additional information
CREATE TABLE IF NOT EXISTS user_profiles (
    user_id INTEGER PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    primary_os VARCHAR(20), -- Linux/Win/Mac
    gpu_model VARCHAR(50),  -- RTX model or none
    experience_level VARCHAR(20), -- Beginner/Intermediate/Advanced
    preferred_hardware VARCHAR(20), -- Jetson/Cloud
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Table for storing user queries
CREATE TABLE IF NOT EXISTS user_queries (
    id SERIAL PRIMARY KEY,
    query TEXT NOT NULL,
    user_id VARCHAR(255),
    session_id VARCHAR(255),
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    response_time_ms INTEGER,
    success BOOLEAN DEFAULT TRUE
);

-- Table for storing clicked sources
CREATE TABLE IF NOT EXISTS clicked_sources (
    id SERIAL PRIMARY KEY,
    query_id INTEGER REFERENCES user_queries(id) ON DELETE CASCADE,
    source_url TEXT NOT NULL,
    source_title TEXT,
    user_id VARCHAR(255),
    session_id VARCHAR(255),
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Create indexes for better performance
CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);
CREATE INDEX IF NOT EXISTS idx_sessions_user_id ON sessions(user_id);
CREATE INDEX IF NOT EXISTS idx_sessions_expires_at ON sessions(expires_at);
CREATE INDEX IF NOT EXISTS idx_verification_tokens_identifier ON verification_tokens(identifier);
CREATE INDEX IF NOT EXISTS idx_verification_tokens_value ON verification_tokens(value);
CREATE INDEX IF NOT EXISTS idx_user_queries_timestamp ON user_queries(timestamp);
CREATE INDEX IF NOT EXISTS idx_user_queries_user_id ON user_queries(user_id);
CREATE INDEX IF NOT EXISTS idx_clicked_sources_query_id ON clicked_sources(query_id);
CREATE INDEX IF NOT EXISTS idx_clicked_sources_timestamp ON clicked_sources(timestamp);

-- Table for storing personalized content versions
CREATE TABLE IF NOT EXISTS personalized_content (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
    chapter_url VARCHAR(500) NOT NULL,  -- URL of the original chapter
    original_content_hash VARCHAR(255), -- Hash to detect content changes
    personalized_content TEXT,  -- The personalized version
    personalization_settings JSONB, -- Store personalization preferences used
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(user_id, chapter_url)  -- Each user gets one personalized version per chapter
);

-- Index for efficient lookup by user and chapter
CREATE INDEX IF NOT EXISTS idx_personalized_content_user_chapter ON personalized_content(user_id, chapter_url);
CREATE INDEX IF NOT EXISTS idx_personalized_content_created_at ON personalized_content(created_at);

-- Table for storing translated content versions
CREATE TABLE IF NOT EXISTS translated_content (
    id SERIAL PRIMARY KEY,
    chapter_url VARCHAR(500) NOT NULL,  -- URL of the original chapter
    target_language VARCHAR(10) NOT NULL DEFAULT 'ur',  -- Language code (ur for Urdu)
    original_content_hash VARCHAR(255), -- Hash to detect content changes
    translated_content TEXT,  -- The translated version
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(chapter_url, target_language)  -- One translation per chapter per language
);

-- Index for efficient lookup by chapter and language
CREATE INDEX IF NOT EXISTS idx_translated_content_chapter_lang ON translated_content(chapter_url, target_language);
CREATE INDEX IF NOT EXISTS idx_translated_content_created_at ON translated_content(created_at);