# Better-Auth Integration for Docusaurus

This implementation adds Better-Auth authentication to your Docusaurus site with custom registration fields and user metadata management.

## Features

- **Signup, Signin, Signout**: Complete authentication flow
- **Custom Registration Fields**:
  - Software background (text input)
  - Hardware available (checkboxes: RTX_40x, RTX_30x, Jetson_Orin, None)
  - Experience level (select: Beginner, Intermediate, Advanced)
- **Metadata Storage**: User metadata stored in PostgreSQL with Drizzle ORM
- **Secure Endpoints**: Protected API routes for user profile management
- **Frontend Integration**: React components for authentication UI

## Architecture

- **Backend**: Better-Auth with PostgreSQL adapter and Drizzle ORM
- **Frontend**: React components for auth UI and metadata utilities
- **Database**: PostgreSQL with custom user metadata table

## Setup

### Prerequisites

- Node.js 18+
- PostgreSQL database

### Installation

1. Install dependencies:
```bash
npm install better-auth @better-auth/postgres-adapter drizzle-orm pg @types/pg
```

2. Set up environment variables:
```bash
# .env
DATABASE_URL=postgresql://user:password@localhost:5432/mydb
SECRET_KEY=your-super-secret-key
```

### Database Setup

1. Run the following SQL to create the metadata table:

```sql
CREATE TABLE user_metadata (
  id SERIAL PRIMARY KEY,
  user_id TEXT UNIQUE NOT NULL,
  software_background TEXT,
  hardware_available JSON,
  experience_level TEXT,
  created_at TIMESTAMP DEFAULT NOW() NOT NULL,
  updated_at TIMESTAMP DEFAULT NOW() NOT NULL
);

CREATE INDEX idx_user_metadata_user_id ON user_metadata(user_id);
```

2. Alternatively, if using Drizzle Kit for migrations:

```bash
npx drizzle-kit generate:pg # Generate migration
npx drizzle-kit push:pg      # Apply migration
```

### Backend Configuration

1. Start the authentication server (or integrate with your existing backend):
```bash
# Example server setup
import express from 'express';
import { auth } from './auth/backend/auth';
import { 
  getCurrentUserProfile, 
  updateUserProfile, 
  deleteUserProfile, 
  signUp, 
  signIn, 
  signOut 
} from './auth/backend/api';
import { requireAuth } from './auth/backend/middleware';

const app = express();
app.use(express.json());

// Authentication endpoints
app.post('/api/auth/signup', signUp);
app.post('/api/auth/signin', signIn);
app.post('/api/auth/signout', signOut);

// Protected endpoints
app.get('/api/auth/profile', requireAuth, getCurrentUserProfile);
app.put('/api/auth/profile', requireAuth, updateUserProfile);
app.delete('/api/auth/profile', requireAuth, deleteUserProfile);

app.listen(3000, () => {
  console.log('Auth server running on port 3000');
});
```

### Frontend Integration

1. Add the AuthComponent to your Docusaurus pages:
```jsx
// Example Docusaurus page with auth
import React from 'react';
import AuthComponent from '../auth/frontend/AuthComponent';

function MyPage() {
  return (
    <div>
      <h1>My Protected Page</h1>
      <AuthComponent />
    </div>
  );
}

export default MyPage;
```

2. Use the metadata utilities to fetch and display user-specific content:
```jsx
import { useUserMetadata } from '../auth/frontend/metadataUtils';

function PersonalizedContent() {
  const { userProfile, loading, error } = useUserMetadata();

  if (loading) return <div>Loading...</div>;
  if (error) return <div>Error: {error}</div>;

  return (
    <div>
      <h2>Content based on your profile:</h2>
      {userProfile?.user.experienceLevel && (
        <p>Recommended for {userProfile.user.experienceLevel} users</p>
      )}
    </div>
  );
}
```

## API Endpoints

- `POST /api/auth/signup` - User registration with custom fields
- `POST /api/auth/signin` - User login
- `POST /api/auth/signout` - User logout
- `GET /api/auth/profile` - Get current user profile (protected)
- `PUT /api/auth/profile` - Update user profile (protected)
- `DELETE /api/auth/profile` - Delete user profile (protected)

## Data Schema

The user metadata is stored in the `user_metadata` table:

```typescript
interface UserMetadata {
  id: number;            // Auto-generated ID
  userId: string;        // References Better-Auth user ID
  softwareBackground?: string;    // User's software background
  hardwareAvailable?: string[];   // Array of selected hardware
  experienceLevel?: string;       // "beginner" | "intermediate" | "advanced"
  createdAt: Date;       // Creation timestamp
  updatedAt: Date;       // Update timestamp
}
```

## Security

- All profile endpoints are protected with authentication middleware
- Passwords are securely hashed by Better-Auth
- Session management handled by Better-Auth
- Use HTTPS in production

## Development

To run the auth server locally:

1. Set up your PostgreSQL database
2. Update the DATABASE_URL in your environment
3. Run the Drizzle migrations (or create tables manually)
4. Start the auth server with `ts-node` or build and run

## Integration with Docusaurus

To integrate with your Docusaurus site:

1. Add the auth API routes to your Docusaurus backend server
2. Create React components using the provided utilities
3. Use the user metadata to customize content and experience

This implementation provides a clean separation between authentication (handled by Better-Auth) and user metadata (handled by custom tables), making it easy to extend with additional fields or functionality.

## Chapter Personalization

The system provides dynamic content transformation based on user profile data:

### Transformation Rules

1. **Experience Level**:
   - Beginner: Simplified text, fewer advanced steps, hidden complex sections
   - Intermediate: Moderate detail, standard content
   - Advanced: All content including advanced sections

2. **Hardware Available**:
   - RTX users: Show "Advanced Isaac Sim steps" section
   - Jetson users: Include "Jetson deployment" subsection
   - Other hardware: Standard content

3. **Content Markers**:
   - `<!-- BEGINNER_HIDE_START -->` / `<!-- BEGINNER_HIDE_END -->`: Hide from beginners
   - `<!-- COMPLEX_EXPLANATION_START -->` / `<!-- COMPLEX_EXPLANATION_END -->`: Complex content
   - `<!-- ISAAC_SIM_ADVANCED_START -->` / `<!-- ISAAC_SIM_ADVANCED_END -->`: Isaac Sim advanced steps
   - `<!-- JETSON_DEPLOYMENT_START -->` / `<!-- JETSON_DEPLOYMENT_END -->`: Jetson deployment section

### API Endpoints

- `POST /api/chapters/:id/personalize` - Transform chapter content based on user profile (protected)

### Frontend Implementation

1. Use the PersonalizedChapter component:
```jsx
import PersonalizedChapter from '../auth/frontend/PersonalizedChapter';

function MyChapterPage() {
  const chapterMarkdown = `# My Chapter Content...`;

  return (
    <PersonalizedChapter
      chapterId="my-chapter"
      originalMarkdown={chapterMarkdown}
    />
  );
}
```

2. Or use the client-side function:
```jsx
import { replaceChapterContent } from '../auth/frontend/clientPersonalize';

// Replace content in an element
await replaceChapterContent('chapter-1', 'chapter-content');
```

## Urdu Translation Feature

The system includes Urdu translation capabilities with markdown preservation:

### Translation API Endpoint

- `POST /api/translate` - Translates chapter content to Urdu while preserving markdown structure

### Features

1. **Markdown Preservation**:
   - Fenced code blocks (```code```) are preserved as-is
   - Inline code (`code`) remains unchanged
   - Links, images, headers, and formatting are maintained
   - Only text content is translated

2. **Caching System**:
   - Translations are cached to file system to improve performance
   - Cache expires after 24 hours by default
   - Reduces API calls to translation service

3. **Fallback Handling**:
   - If translation fails, original content is preserved
   - Error messages provided without breaking user experience

### Client-Side Usage

1. Add a translation button to your Docusaurus pages:
```html
<button onclick="toggleTranslation('chapter-1')" class="translate-btn">.Translate to Urdu</button>
```

2. Use the translation functions:
```javascript
import { translateChapter, toggleTranslation } from '../auth/frontend/translateChapter';

// Direct translation
await translateChapter('chapter-1', {
  contentSelector: '.doc-content',
  buttonSelector: '.translate-btn',
  targetLang: 'ur'
});

// Toggle between original and translated
await toggleTranslation('chapter-1');
```

### Translation Process

1. Extract non-translatable elements (code blocks, links, etc.)
2. Send only translatable text to the translation service
3. Receive translated text back
4. Restore original non-translatable elements in correct positions
5. Cache the result for future requests

### Configuration

Set environment variables for translation service:
```bash
# Use our internal TranslatorAgent (recommended)
TRANSLATION_PROVIDER=agent
AGENT_TRANSLATOR_URL=http://localhost:8003

# OR use an external API (not implemented in this example)
TRANSLATION_PROVIDER=api
TRANSLATION_API_KEY=your_api_key_here
```

## Future Enhancements

- Social authentication providers
- Password reset functionality
- Email verification
- Session management improvements
- Rate limiting for auth endpoints
- More sophisticated content personalization rules
- Additional language support beyond Urdu
- Database-based caching for better scalability