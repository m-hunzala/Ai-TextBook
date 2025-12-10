---
sidebar_position: 9
---

# Chat Widget Integration

## Overview
The RAG Chatbot includes a React component that can be integrated into your Docusaurus site. This widget provides a floating chat button and enables users to ask questions about selected text on the page.

## Installation

### 1. Add the component to your Docusaurus site

The ChatWidget component is already included in the `src/components/ChatWidget` directory. You can import and use it in your Docusaurus pages or layout.

### 2. Import the component

```jsx
import ChatWidget from '@site/src/components/ChatWidget';
```

### 3. Add the widget to your layout

You can add the widget to your site's layout by modifying the `src/pages/Layout.js` file or by adding it to individual pages.

## Usage Examples

### Basic Usage

```jsx
import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function MyPage() {
  return (
    <div>
      <h1>My Documentation Page</h1>
      <p>Some content that users might want to ask questions about.</p>
      
      {/* Add the chat widget */}
      <ChatWidget 
        apiEndpoint="https://your-rag-api.com" 
        apiKey="your-api-key"
      />
    </div>
  );
}
```

### Layout Integration

To add the chat widget to all pages, modify your main layout file (usually in `src/theme/Layout/index.js`):

```jsx
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
        <ChatWidget 
          apiEndpoint={process.env.REACT_APP_CHATBOT_API_ENDPOINT || 'http://localhost:8000'}
          apiKey={process.env.REACT_APP_CHATBOT_API_KEY || ''}
        />
      </OriginalLayout>
    </>
  );
}
```

## Configuration Options

The ChatWidget component accepts the following props:

| Prop | Type | Default | Description |
|------|------|---------|-------------|
| `apiEndpoint` | String | `'http://localhost:8000'` | Base URL of your RAG chatbot API |
| `apiKey` | String | `''` | API key for authentication |

## Features

### Floating Chat Button
- Appears in the bottom-right corner of the page
- Toggles the chat interface when clicked

### Text Selection
- When users select text on the page, a "Ask about this" button appears
- Clicking the button sends the selected text to the AI for context-specific answers
- The selected text is provided as the `highlight_text` parameter to the API

### Source Citations
- Responses include source snippets from the documentation
- Links to the original source pages
- Confidence scores for retrieved information

### Session Management
- Maintains conversation history within the same page session
- Clear interface for asking follow-up questions

## API Integration

The widget uses the `/query` endpoint of your RAG chatbot API. When a user makes a selection, it sends:

```json
{
  "query": "User's question or generated query about selection",
  "highlight_text": "The selected text",
  "document_url": "Current page URL"
}
```

## Styling

The component includes its own CSS file with responsive styling. You can customize the appearance by modifying the CSS file at `src/components/ChatWidget/ChatWidget.css`.

## Security

- API key is passed in the request headers
- All communication happens over HTTPS in production
- The widget respects your API's rate limiting

## Troubleshooting

### Widget Not Appearing
- Check that you've properly imported the component
- Verify the API endpoint is accessible
- Check browser console for any JavaScript errors

### Text Selection Not Working
- Ensure the page content is selectable (not disabled by CSS)
- Check that no other scripts are interfering with selection events

### API Communication Issues
- Verify the API endpoint is correct
- Check that the API key is valid
- Ensure CORS is properly configured on your API server