// Client module to inject runtime configuration into the browser environment
// This makes environment variables available to the frontend components

// Create a global window.ENV object to store runtime configuration
if (typeof window !== 'undefined') {
  // Inject any runtime configuration that was defined server-side
  // or through environment variables during build process
  window.ENV = {
    // These values can be customized during deployment
    REACT_APP_API_URL: window.__RUNTIME_CONFIG__?.REACT_APP_API_URL || 'http://localhost:8000',
    API_URL: window.__RUNTIME_CONFIG__?.API_URL || 'http://localhost:8000',
    REACT_APP_API_KEY: window.__RUNTIME_CONFIG__?.REACT_APP_API_KEY || 'your-api-key-here',
    API_KEY: window.__RUNTIME_CONFIG__?.API_KEY || 'your-api-key-here',
    BETTER_AUTH_URL: window.__RUNTIME_CONFIG__?.BETTER_AUTH_URL || 'http://localhost:4000',
    AUTH_API_URL: window.__RUNTIME_CONFIG__?.AUTH_API_URL || 'http://localhost:4000',
    RAG_BACKEND_URL: window.__RUNTIME_CONFIG__?.RAG_BACKEND_URL || 'http://localhost:8000'
  };
}