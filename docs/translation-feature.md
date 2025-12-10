---
sidebar_position: 13
---

# Chapter Translation Feature

## Overview
Our platform provides on-demand translation of chapter content to Urdu, preserving technical terms and code while translating exposition and explanations. The feature includes caching to optimize costs and performance.

## Features

### On-Demand Translation
- Translate any chapter to Urdu with a single click
- Preserves all code blocks, URDF files, and technical identifiers
- Translates exposition, tasks, and rubrics into formal Urdu
- Caching layer to avoid repeated translation costs

### Technical Preservation
- Code blocks (```python, ```bash, etc.) remain in original language
- Variable names, function names, and technical identifiers preserved
- URDF and configuration files remain unchanged
- External links and references maintained

### Caching & Performance
- Translations cached in Neon Postgres database
- Subsequent requests served from cache
- Automatic cache update when content changes

### Fallback Mechanism
- If LLM translation fails, shows original content with error message
- Side-by-side view of original and partial translations when possible

## Implementation

### API Endpoint
The `/api/translate` endpoint accepts:
- `chapter_id`: Unique identifier for the chapter
- `content`: Original chapter content in markdown
- `language`: Target language (currently 'ur' for Urdu)

### React Component
The `TranslatorButton` component provides:
- Simple "Translate to Urdu" button
- Translation status indicators
- Error handling and fallback display

## Using in Chapters

To add translation to your chapters, include the TranslatorButton component:

```jsx
import TranslatorButton from '@site/src/components/TranslatorButton';

<TranslatorButton 
  chapterId="week-1-introduction" 
  chapterContent={content}
  onTranslate={setContent} 
  language="ur"
/>
```

## How Translation Works

1. User clicks "Translate to Urdu" button
2. System checks cache for existing translation
3. If cached, returns immediately; otherwise:
4. Sends content to LLM with instructions to preserve:
   - All code blocks
   - Technical terms and identifiers
   - URDF and configuration files
5. Translates only the exposition, explanations, and instructions
6. Caches the result for future use
7. Updates the page with translated content

## Technical Terms Preserved

The translation system maintains the following technical elements:

### Programming Terms
- Language keywords (`def`, `class`, `import`, etc.)
- Common libraries (`numpy`, `pandas`, `opencv`, etc.)
- ROS/ROS2 specific terms (`rospy`, `rclpy`, `catkin`, etc.)

### Hardware & Robotics
- Sensor types (`LIDAR`, `IMU`, `camera`, etc.)
- Robot components (`actuator`, `controller`, etc.)
- Technical specifications (`RTX 4070`, `Jetson`, etc.)

### Formats & Protocols
- Code formats (`JSON`, `XML`, `YAML`, etc.)
- Protocols (`HTTP`, `TCP/IP`, etc.)
- File formats (`URDF`, `STL`, `DAE`, etc.)

## Performance Considerations

- Initial translation may take 5-15 seconds depending on chapter length
- Subsequent views load instantly from cache
- Translation quality improves with longer, more context-rich content
- Costs optimized through caching to avoid redundant API calls

## Troubleshooting

### Translation Takes Too Long
- Large chapters with complex code may take longer to process
- Check for proper code block formatting

### Code Blocks Not Preserved
- Ensure code blocks use proper markdown formatting (```lang)
- Verify technical terms are not being translated

### Cache Issues
- Cached translations remain until the system updates
- Contact support if you need to refresh a translation

## Security

- All content is processed securely through the API
- No personal information is included in translation requests
- Translation cache is secured with the same database protections as other content