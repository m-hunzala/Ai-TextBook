-- Migration to create chapter_translations table

-- Create the table
CREATE TABLE IF NOT EXISTS chapter_translations (
    id SERIAL PRIMARY KEY,
    chapter_id VARCHAR(255) NOT NULL,
    language VARCHAR(10) NOT NULL,
    original_content TEXT NOT NULL,
    translated_content TEXT NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    
    -- Create unique constraint to prevent duplicate translations
    CONSTRAINT unique_chapter_language UNIQUE (chapter_id, language)
);

-- Create indexes for better performance
CREATE INDEX IF NOT EXISTS idx_chapter_translations_chapter_id ON chapter_translations(chapter_id);
CREATE INDEX IF NOT EXISTS idx_chapter_translations_language ON chapter_translations(language);
CREATE INDEX IF NOT EXISTS idx_chapter_translations_created_at ON chapter_translations(created_at);

-- Create a trigger to update the 'updated_at' timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER update_chapter_translations_updated_at 
    BEFORE UPDATE ON chapter_translations 
    FOR EACH ROW 
    EXECUTE FUNCTION update_updated_at_column();

-- Optional: Create a partial index for frequently accessed translations
-- This example assumes you'll be querying mostly for Urdu translations
-- CREATE INDEX IF NOT EXISTS idx_chapter_translations_urdu ON chapter_translations(chapter_id) WHERE language = 'ur';