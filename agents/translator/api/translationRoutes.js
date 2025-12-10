const express = require('express');
const cors = require('cors');
const cookieParser = require('cookie-parser');
const TranslationService = require('../../src/services/translationService');

const router = express.Router();

// Enable CORS and JSON parsing
router.use(cors());
router.use(express.json({ limit: '10mb' }));
router.use(cookieParser());

// Initialize translation service
const translationService = new TranslationService();

// Translation endpoint
router.post('/translate', async (req, res) => {
  const { content, targetLanguage = 'ur' } = req.body;

  try {
    // Check if user is authenticated (using better-auth)
    // Note: We'll need to verify the auth headers or cookies
    const token = req.headers.authorization?.replace('Bearer ', '');
    
    if (!token) {
      return res.status(401).json({ 
        error: 'Authentication required. Please log in to use translation feature.' 
      });
    }

    // Basic validation
    if (!content) {
      return res.status(400).json({ error: 'Content is required for translation' });
    }

    if (targetLanguage !== 'ur') {
      return res.status(400).json({ error: 'Currently only Urdu translation is supported' });
    }

    // Perform translation
    const translatedContent = await translationService.translateWithFormattingPreservation(content);
    
    res.json({
      success: true,
      originalContent: content,
      translatedContent: translatedContent,
      targetLanguage: targetLanguage
    });

  } catch (error) {
    console.error('Translation API error:', error);
    res.status(500).json({ 
      error: 'Translation failed', 
      message: error.message 
    });
  }
});

// Health check endpoint
router.get('/health', (req, res) => {
  res.json({ status: 'OK', service: 'Translation API' });
});

module.exports = router;