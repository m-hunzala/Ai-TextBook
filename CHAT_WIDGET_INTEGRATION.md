# Docusaurus Chat Widget Integration Guide

This guide explains how to integrate the chat widget into your Docusaurus site.

## Prerequisites

- Docusaurus v2 or v3 project
- RAG backend running and accessible
- API key for the RAG backend

## File Structure

First, ensure you have the following files in your Docusaurus project:

```
my-docusaurus-site/
├── src/
│   ├── components/
│   │   └── ChatWidget/
│   │       ├── ChatWidget.jsx
│   │       ├── ChatWidget.css
│   │       └── README.md
│   └── theme/
│       └── ChatWidgetWrapper.jsx
└── .env
```

## Step 1: Environment Configuration

Add the following to your `.env` file (in the root of your Docusaurus project):

```env
# RAG Backend Configuration
REACT_APP_API_URL=https://your-rag-backend.com
REACT_APP_API_KEY=your-api-key-here
```

Replace with your actual API endpoint and API key.

## Step 2: Add to Layout (Recommended)

The most common approach is to add the chat widget to all pages by modifying the Layout component.

### For Docusaurus v3:

Create or modify `src/pages/LayoutWrapper.jsx`:

```jsx
import React from 'react';
import Layout from '@theme/Layout';
import ChatWidgetWrapper from '@site/src/theme/ChatWidgetWrapper';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <ChatWidgetWrapper />
    </>
  );
}
```

Then in your `docusaurus.config.js`, update the presets to use your custom layout:

```js
module.exports = {
  // ... other config
  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // ... other docs config
        },
        blog: false, // optional
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],
  themes: [
    // Add a custom theme to wrap all pages
    () => {
      const ErrorBoundary = require('@docusaurus/ErrorBoundary').default;
      const LayoutWrapper = require('./src/pages/LayoutWrapper').default;
      return {
        name: 'custom-theme',
        getThemePath() {
          return require.resolve('./src/pages/LayoutWrapper');
        },
        render({props}) {
          return (
            <ErrorBoundary
              fallback={(errorInfo) => (
                <LayoutWrapper>
                  <div>Something went wrong: {errorInfo?.error?.message}</div>
                </LayoutWrapper>
              )}>
              <LayoutWrapper />
            </ErrorBoundary>
          );
        },
      };
    },
  ],
};
```

### Alternative: For Docusaurus v2 or simpler integration

If you prefer a simpler approach for Docusaurus v2, you can modify the main Layout theme component:

1. Create `src/theme/Layout/index.js`:
```jsx
import React from 'react';
import Theme from '@theme-original/Layout';
import ChatWidgetWrapper from '@site/src/theme/ChatWidgetWrapper';

export default function Layout(props) {
  return (
    <>
      <Theme {...props} />
      <ChatWidgetWrapper />
    </>
  );
}
```

This will wrap every page with the chat widget automatically.

## Step 3: Add to Individual Pages (Optional)

If you only want the widget on specific pages, you can import and use it directly:

```jsx
// In any MDX file or React component
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

export default function MyPage() {
  return (
    <div>
      <h1>My Page</h1>
      <p>Some content...</p>
      <ChatWidget 
        apiUrl={process.env.REACT_APP_API_URL || 'http://localhost:8000'}
        apiKey={process.env.REACT_APP_API_KEY || 'your-api-key'}
      />
    </div>
  );
}
```

## Step 4: Configuration Options

### Environment Variables

- `REACT_APP_API_URL`: The URL of your RAG backend (default: http://localhost:8000)
- `REACT_APP_API_KEY`: The API key for authentication

### Custom Configuration

If you need more control, you can directly import and configure the widget:

```jsx
<ChatWidget 
  apiUrl="https://your-custom-api.com"
  apiKey="your-custom-key"
/>
```

## Step 5: Build and Test

1. Restart your Docusaurus development server:
```bash
npm run start
# or
yarn start
```

2. Navigate to any page and verify the chat widget appears as a floating button in the bottom-right corner.

3. Test the functionality:
   - Click the chat button to open the widget
   - Select text on the page
   - Click "Answer from selected text" 
   - Ask a question about the selected text
   - Verify that the response includes source links

## Troubleshooting

1. **Widget doesn't appear**: Check that environment variables are set correctly and restart the development server.

2. **API errors**: Verify that your API URL and key are correct in the environment variables.

3. **CORS issues**: Make sure your RAG backend allows requests from your Docusaurus domain.

4. **Text selection not working**: The widget requires users to select text on the page before clicking "Answer from selected text".

## Customization

You can customize the appearance by modifying `ChatWidget.css` or by adding custom CSS classes that override the default styles.

## Security Note

Remember to use a proper API key in production and ensure your RAG backend is secured against unauthorized access.