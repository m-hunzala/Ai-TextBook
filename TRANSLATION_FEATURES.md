# Urdu Translation Feature

## Overview

The translation feature allows users to translate chapters to Urdu while preserving code blocks and technical content. The system provides toggle functionality to switch between English, Urdu, and side-by-side views.

## How It Works

### 1. Translation Process
- User clicks "Translate to Urdu" button at the top of each chapter
- System sends original content to LLM for translation
- Code blocks, configuration examples, and technical identifiers are preserved
- Translated content is cached in Neon Postgres to avoid repeated API costs
- Results are displayed with toggle to switch between views

### 2. View Modes
- **English View**: Original English content
- **Urdu View**: Translated Urdu content with RTL rendering
- **Side-by-Side View**: Both languages displayed side-by-side for comparison

### 3. Code Block Preservation
- All code blocks (```...``` and `...`) are identified and extracted before translation
- Original code blocks are preserved exactly as they are
- Code blocks are properly restored in translated content
- Code blocks maintain LTR direction even in RTL Urdu text

## Technical Implementation

### Database Schema
```sql
-- Table for storing translated content versions
CREATE TABLE translated_content (
    id SERIAL PRIMARY KEY,
    chapter_url VARCHAR(500) NOT NULL,  -- URL of the original chapter
    target_language VARCHAR(10) NOT NULL DEFAULT 'ur',  -- Language code (ur for Urdu)
    original_content_hash VARCHAR(255), -- Hash to detect content changes
    translated_content TEXT,  -- The translated version
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(chapter_url, target_language)  -- One translation per chapter per language
);
```

### Backend API Endpoints

#### `POST /translation/translate`
Translates content to the target language (Urdu).

Request body:
```json
{
  "chapter_url": "https://example.com/docs/chapter1",
  "original_content": "Original markdown content...",
  "target_language": "ur",
  "force_regenerate": false
}
```

Response:
```json
{
  "status": "success",
  "message": "Content translated successfully",
  "translated_content": "Translated Urdu content...",
  "cache_hit": false
}
```

#### `GET /translation/content/{chapter_url}/{target_language}`
Retrieves both original and translated content.

#### `GET /translation/languages/{chapter_url}`
Returns all available languages for a specific chapter.

#### `DELETE /translation/content/{chapter_url}/{target_language}`
Clears cached translation.

### Frontend Component
The `TranslationToggle` component provides:
- "Translate to Urdu" button
- Toggle between English/Urdu/side-by-side views
- Clear translation option
- Visual indicators for language

## Translation Quality

### Text Translation
- Professional-grade translation of narrative content
- Technical terminology appropriately translated
- Cultural context preserved where relevant
- Proper Urdu typography and rendering

### Code Preservation
- All code blocks remain in original English
- Syntax highlighting maintained
- Technical identifiers unchanged
- Configuration examples preserved exactly

### RTL Support
- Urdu text renders right-to-left properly
- Code blocks maintain left-to-right direction
- Mixed content (Urdu with English code) handled correctly

## Caching Strategy

### Content Hashing
- Uses SHA256 hashing to detect content changes
- Prevents unnecessary translation API calls
- Automatically regenerates when original content changes

### Database Caching
- Efficient storage of translated content
- Indexed for fast retrieval by chapter and language
- Automatic cleanup mechanisms

## User Experience

### Initial State
- "Translate to Urdu" button visible at top of each chapter
- English content displayed by default

### After Translation
- Toggle buttons to switch between views
- Clear translation option
- Visual language indicators

### Language Indicators
- "English" or "اردو" badges show current language
- Clear visual distinction between languages
- Proper RTL/LTR indicators

## Implementation Considerations

### Performance
- Translated content cached to minimize API costs
- Content hashing prevents unnecessary regeneration
- Efficient database queries with proper indexing
- Client-side view switching without server calls

### Accuracy
- LLM instructed to preserve technical accuracy
- Code blocks completely preserved
- Technical terms handled appropriately
- Human-readable translations prioritized

### Fallback Strategy
- If translation fails, original content is preserved
- Clear error handling for API failures
- Graceful degradation when translation unavailable

## API Usage

### Translating Content
```javascript
fetch('/translation/translate', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    chapter_url: window.location.href,
    original_content: originalMarkdownContent,
    target_language: 'ur',
    force_regenerate: false
  })
});
```

### Getting Available Languages
```javascript
fetch('/translation/languages/' + encodeURIComponent(chapterUrl))
  .then(response => response.json())
  .then(data => console.log(data.languages));
```

### Clearing Translation
```javascript
fetch('/translation/content/' + encodeURIComponent(chapterUrl) + '/ur', {
  method: 'DELETE'
});
```

## RTL Support

The interface properly handles right-to-left text rendering:
- Urdu text flows from right to left
- Code blocks maintain left-to-right direction
- Mixed content (Urdu with English code) renders correctly
- Proper text alignment for each language

## Security and Privacy

- No user-specific information required for translation
- Chapter content temporarily stored only for translation
- Translated content cached per chapter, not per user
- No sensitive information transmitted to translation API

This feature enables broader access to documentation by providing Urdu translations while maintaining technical accuracy and preserving important code examples.