// Runtime configuration for the translation feature
// This can be imported by other components to access translation settings

export const translationConfig = {
  // API endpoints
  authApiUrl: process.env.AUTH_API_URL || 'http://localhost:4000',
  agentTranslatorUrl: process.env.AGENT_TRANSLATOR_URL || 'http://localhost:8003',
  
  // Supported languages
  supportedLanguages: ['ur', 'en'], // Urdu, English
  
  // Default target language
  defaultTargetLanguage: 'ur',
  
  // Content extraction settings
  contentExtraction: {
    selectors: ['.markdown', 'article', '.doc-content', 'main'],
    excludeSelectors: ['.code-block', '.admonition', '.button', 'nav', 'header', 'footer']
  }
};