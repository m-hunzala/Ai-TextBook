---
sidebar_position: 12
---

# Integrating Personalization in Chapters

## Overview
This guide explains how to integrate the personalization feature into your Docusaurus documentation chapters.

## Adding the Personalize Button

### Step 1: Import the Component

```jsx
import PersonalizeChapterButton from '@site/src/components/PersonalizeChapterButton';
```

### Step 2: Add the Button to Your Chapter

Add the PersonalizeChapterButton component at the beginning of your chapter content:

```jsx
import React, { useState } from 'react';
import PersonalizeChapterButton from '@site/src/components/PersonalizeChapterButton';

function ChapterWithPersonalization() {
  const [content, setContent] = useState(`# Your Chapter Content

This is the original chapter content that can be personalized based on user profile...`);

  return (
    <div>
      <PersonalizeChapterButton 
        chapterContent={content}
        onPersonalize={setContent} 
      />
      <div dangerouslySetInnerHTML={{ __html: content }} />
    </div>
  );
}

// For use in regular markdown files, just include this at the top of your .md file:
// <PersonalizeChapterButton />
```

## How Personalization Works

When a user clicks the "Personalize this chapter" button:

1. The system checks if the user is logged in
2. Fetches the user's profile from Neon Postgres
3. Applies personalization rules based on:
   - GPU VRAM (shows cloud alternatives for low VRAM GPUs)
   - Operating System (local vs cloud instructions)
   - Experience Level (content complexity)
   - Hardware Owned (relevant examples and checklists)
   - Main Domain (domain-specific examples)

## Example Personalization Rules

### GPU-Aware Code Replacement
```javascript
// Before personalization (high-end GPU)
const model = loadLargeModel();
model.to('cuda:0');

// After personalization (low VRAM GPU)
const model = loadOptimizedModel();
// Automatically configured for available VRAM
```

### Experience Level Adjustment
```markdown
### Advanced Technique (shown to Advanced users)
This technique uses complex algorithms...

### Basic Approach (shown to Beginner users)
This is a simplified approach...
```

## Security Considerations

- Profile data is only accessed when users are authenticated
- User data is not exposed to other users
- All API requests are properly authenticated

## Implementation Notes

For Docusaurus integration, you may need to implement the personalization on the server-side during build time or use a client-side approach for dynamic personalization.

The example implementation shows a client-side approach, but for production use, consider pre-processing content on the backend to optimize performance.