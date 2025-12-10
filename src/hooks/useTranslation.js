import { useState, useCallback } from 'react';

const useTranslation = () => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [translationError, setTranslationError] = useState(null);
  const [isUrdu, setIsUrdu] = useState(false);

  const translateContent = useCallback(async (chapterId, originalContent, authToken = null) => {
    setIsTranslating(true);
    setTranslationError(null);

    try {
      // Use the translation endpoint from the auth service
      const authApiUrl = process.env.AUTH_API_URL || 'http://localhost:4000';

      const response = await fetch(`${authApiUrl}/api/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapterId: chapterId || 'current',
          originalContent: originalContent,
          target: 'ur'
        })
      });

      if (!response.ok) {
        throw new Error(`Translation API error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();

      setIsUrdu(true);
      return data.translatedContent || data.translated_text; // Handle different possible response formats
    } catch (error) {
      console.error('Translation error:', error);
      setTranslationError(error.message);
      throw error;
    } finally {
      setIsTranslating(false);
    }
  }, []);

  const resetTranslation = useCallback(() => {
    setIsUrdu(false);
    setTranslationError(null);
  }, []);

  return {
    isTranslating,
    translationError,
    isUrdu,
    translateContent,
    resetTranslation
  };
};

export default useTranslation;