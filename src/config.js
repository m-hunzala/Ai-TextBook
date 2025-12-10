// src/config.js
// Runtime configuration for the application

const config = {
  BETTER_AUTH_URL: typeof window !== 'undefined'
    ? window.ENV?.BETTER_AUTH_URL || 'http://localhost:4000'
    : process.env.BETTER_AUTH_URL || 'http://localhost:4000',
  
  AUTH_API_URL: typeof window !== 'undefined'
    ? window.ENV?.AUTH_API_URL || 'http://localhost:4000'
    : process.env.AUTH_API_URL || 'http://localhost:4000',
    
  RAG_BACKEND_URL: typeof window !== 'undefined'
    ? window.ENV?.RAG_BACKEND_URL || 'http://localhost:8000'
    : process.env.RAG_BACKEND_URL || 'http://localhost:8000',
};

export default config;