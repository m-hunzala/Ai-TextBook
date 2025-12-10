# Content Personalization Feature

## Overview

The personalization feature allows logged-in users to get chapters customized based on their profile information. The system tailors content complexity, hardware-specific instructions, and includes relevant examples based on user preferences.

## How It Works

### 1. User Profile Collection
During signup, users provide information that guides personalization:
- **Primary OS** (Linux/Windows/Mac): Provides OS-specific command examples
- **GPU Model**: Includes optimization tips for specific hardware
- **Experience Level** (Beginner/Intermediate/Advanced): Adjusts content complexity
- **Preferred Hardware** (Jetson/Cloud): Tailors examples and instructions

### 2. Content Transformation Process
The personalization flow:
1. User clicks "Personalize this chapter" button
2. System retrieves user profile and original content
3. LLM (Google Gemini) rewrites content based on user preferences
4. Personalized content is cached in Neon Postgres
5. Results are displayed with toggle to view original content

### 3. Content Adjustments by Experience Level

- **Beginner**: Simplified explanations, step-by-step instructions, technical term definitions
- **Intermediate**: Moderate detail with some advanced concepts
- **Advanced**: Technical explanations, advanced optimizations, optional challenges/exercises

## Technical Implementation

### Database Schema
```sql
-- Table for storing personalized content versions
CREATE TABLE personalized_content (
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
```

### Backend API Endpoints

#### `POST /personalization/personalize`
Personalizes content based on user profile.

Request body:
```json
{
  "chapter_url": "https://example.com/docs/chapter1",
  "original_content": "Original markdown content...",
  "force_regenerate": false
}
```

Response:
```json
{
  "status": "success",
  "message": "Content personalized successfully",
  "personalized_content": "Personalized content...",
  "cache_hit": false
}
```

#### `GET /personalization/content/{chapter_url}`
Retrieves both original and personalized content.

#### `DELETE /personalization/content/{chapter_url}`
Clears cached personalized content.

### Frontend Component
The `PersonalizationToggle` component provides:
- "Personalize this chapter" button
- Toggle between original and personalized content
- Regenerate and clear options
- Visual indicators for content type

## Usage Examples

### For Beginners
- More detailed explanations
- Step-by-step breakdowns
- Definition of technical terms
- Basic code examples

### For Advanced Users
- Technical depth and optimizations
- Performance considerations
- Advanced configuration options
- Optional exercises and challenges

### Hardware-Specific Examples
- Jetson users get embedded-focused examples
- Cloud users get scalability-focused examples
- GPU-specific optimization tips
- OS-specific command examples

## Implementation Considerations

### Performance
- Personalized content is cached to avoid repeated LLM calls
- Content hashing detects when regeneration is needed
- Efficient database queries with proper indexing

### Security
- All endpoints require Better-Auth token authentication
- Content is personalized per-user only
- User data is properly isolated

### Fallback Strategy
- If LLM fails, original content is returned
- Clear error handling for API failures
- Graceful degradation when features are unavailable

## User Experience

### Initial State
- "Personalize this chapter" button visible to logged-in users
- Original content displayed by default

### After Personalization
- Toggle button to switch between original/personalized views
- "Regenerate" option to update personalization
- "Clear" option to remove personalized version

### Content Indicators
- Visual badges showing "Original Content" or "Personalized for you"
- Clear visual distinction between content versions

## Setup Requirements

### Backend
- Better-Auth configured with user profiles
- Gemini API key for content transformation
- Neon Postgres for content caching

### Frontend
- Authentication token storage
- Chapter URL identification
- Content rendering capabilities

## API Usage

```javascript
// Personalize content
fetch('/personalization/personalize', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': `Bearer ${jwtToken}`
  },
  body: JSON.stringify({
    chapter_url: window.location.href,
    original_content: originalMarkdownContent,
    force_regenerate: false
  })
});

// Toggle to view original content
fetch('/personalization/content/' + encodeURIComponent(chapterUrl), {
  headers: {
    'Authorization': `Bearer ${jwtToken}`
  }
});
```

## Caching Strategy

- Personalized content cached per user and chapter
- Content hash prevents unnecessary regeneration
- Cache can be cleared when user preferences change
- Efficient cleanup of outdated personalizations

This feature enhances the learning experience by providing content that matches each user's background, preferred hardware, and technical level.