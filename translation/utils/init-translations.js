// translation/utils/init-translations.js

import { neon } from '@neondatabase/serverless';

const sql = neon(process.env.NEON_DB_URL || '');

/**
 * Initialize the chapter_translations table if it doesn't exist
 */
export async function initializeTranslationTable() {
  try {
    // The table should already exist from the migration, but just in case
    await sql(`
      CREATE TABLE IF NOT EXISTS chapter_translations (
        id SERIAL PRIMARY KEY,
        chapter_id VARCHAR(255) NOT NULL,
        language VARCHAR(10) NOT NULL,
        original_content TEXT NOT NULL,
        translated_content TEXT NOT NULL,
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        
        CONSTRAINT unique_chapter_language UNIQUE (chapter_id, language)
      )
    `);
    
    // Create indexes
    await sql(`CREATE INDEX IF NOT EXISTS idx_chapter_translations_chapter_id ON chapter_translations(chapter_id)`);
    await sql(`CREATE INDEX IF NOT EXISTS idx_chapter_translations_language ON chapter_translations(language)`);
    
    console.log("Translation table initialized successfully");
  } catch (error) {
    console.error("Error initializing translation table:", error);
    throw error;
  }
}

/**
 * Check if a translation exists in cache
 */
export async function checkTranslationCache(chapterId, language) {
  try {
    const result = await sql(
      'SELECT translated_content FROM chapter_translations WHERE chapter_id = $1 AND language = $2',
      [chapterId, language]
    );
    
    return result.length > 0 ? result[0].translated_content : null;
  } catch (error) {
    console.error("Error checking translation cache:", error);
    return null;
  }
}

/**
 * Save a translation to cache
 */
export async function saveTranslationToCache(chapterId, language, originalContent, translatedContent) {
  try {
    await sql(
      `INSERT INTO chapter_translations (chapter_id, language, original_content, translated_content)
       VALUES ($1, $2, $3, $4)
       ON CONFLICT (chapter_id, language)
       DO UPDATE SET
         original_content = EXCLUDED.original_content,
         translated_content = EXCLUDED.translated_content,
         updated_at = CURRENT_TIMESTAMP`,
      [chapterId, language, originalContent, translatedContent]
    );
    
    console.log("Translation saved to cache");
  } catch (error) {
    console.error("Error saving translation to cache:", error);
    throw error;
  }
}