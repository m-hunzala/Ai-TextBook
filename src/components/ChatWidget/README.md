# RAG Chat Widget for Docusaurus

A professional, lightweight chat widget integrated with the RAG (Retrieval-Augmented Generation) backend that allows users to ask questions about the documentation. Features a modern UI that matches the Physical AI & Humanoid Robotics book theme with dark/light mode support and responsive design.

## Features

- **Modern Chat Interface**: Clean, professional design with rounded cards, subtle shadows, and smooth animations
- **Text Selection Integration**: Users can select text on the page and ask questions about the selected content
- **Subagent Support**: Integration with expert AI subagents for specialized queries
- **Source Citations**: Responses include clickable source links that highlight relevant passages in the documentation
- **Dark/Light Theme**: Automatically adapts to the Docusaurus theme (dark/light mode)
- **Responsive Design**: Works well on both desktop and mobile devices
- **Chat Panel with Message History**: Organized conversation view with distinct user/bot messages
- **Typing Indicators and Error Handling**: Visual feedback during API calls
- **Rate Limiting Support**: Handles rate limit responses gracefully

## Installation

1. Make sure you have the component files in your Docusaurus project:
   ```
   src/components/ChatWidget/
   ├── ChatWidget.jsx
   ├── ChatWidget.css
   └── README.md
   ```

2. Create the components directory if it doesn't exist:
   ```bash
   mkdir -p src/components/ChatWidget
   ```

3. Copy the component files to the directory.

## Usage in Docusaurus

### Option 1: Add to Layout (appears on all pages)

To add the chat widget to all pages, modify your `src/theme/Layout/index.js` or create a wrapper:

```jsx
import React from 'react';
import Layout from '@theme/Layout';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <ChatWidget 
        apiUrl={process.env.REACT_APP_API_URL || 'http://localhost:8000'} 
        apiKey={process.env.REACT_APP_API_KEY || 'your-api-key-here'}
      />
    </>
  );
}
```

### Option 2: Add to specific components

You can add the chat widget to specific pages by importing it directly:

```jsx
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

function MyPage() {
  return (
    <div>
      {/* Your page content */}
      <ChatWidget 
        apiUrl="https://your-api-endpoint.com"
        apiKey="your-api-key"
      />
    </div>
  );
}
```

### Option 3: Using Docusaurus MDX support

You can also add it to individual MDX documents:

```mdx
---
title: My Document
---

import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

# My Document

Here's some content...

<ChatWidget 
  apiUrl="https://your-api-endpoint.com"
  apiKey="your-api-key"
/>
```

## Environment Variables

Add these environment variables to your `.env` file:

```env
REACT_APP_API_URL=https://your-rag-backend.com
REACT_APP_API_KEY=your-api-key-here
```

## GitHub Pages Deployment

For GitHub Pages deployment, set the `REACT_APP_API_URL` environment variable during build:

```bash
REACT_APP_API_URL=https://your-backend-api.com npm run build
```

This ensures the chat widget connects to your backend API when deployed to GitHub Pages.

## Configuration

The component accepts these props:

- `apiUrl` (string): The base URL of your RAG backend API (default: 'http://localhost:8000')
- `apiKey` (string): The API key for authentication (required)

## How to Use

1. Select text on any documentation page
2. Click the "Answer from selected text" button in the chat widget
3. Type your question about the selected text
4. Press send to get an AI-generated response

If you don't want to use selected text, simply type your question without selecting text first.

## Styling

The widget comes with default styling in `ChatWidget.css`. You can customize the appearance by modifying this file or by adding additional CSS classes.

## Browser Support

The widget uses modern JavaScript features and should work in all modern browsers. It uses the Selection API which is supported in all modern browsers.