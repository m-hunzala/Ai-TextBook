import React, { useState } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

const TranslationButton = ({ chapterId, chapterContent, onTranslationComplete }) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [translationError, setTranslationError] = useState(null);
  const [isUrdu, setIsUrdu] = useState(false);

  // Simple client-side mock translation (in production, you'd call your API)
  const mockTranslate = (text) => {
    // This is a very basic mock translation - in reality you'd call your API
    return text; // For now, just return the same text
  };

  const translateChapter = async () => {
    setIsTranslating(true);
    setTranslationError(null);

    try {
      // Get the current document content
      const content = chapterContent || getCurrentPageContent();
      
      // Mock translation
      const translated = mockTranslate(content);
      
      setIsUrdu(true);
      
      // Notify parent component of translation completion
      if (onTranslationComplete) {
        onTranslationComplete(translated);
      }
    } catch (err) {
      console.error('Translation error:', err);
      setTranslationError(err.message);
    } finally {
      setIsTranslating(false);
    }
  };

  const toggleTranslation = () => {
    if (isUrdu) {
      setIsUrdu(false);
      if (onTranslationComplete) {
        onTranslationComplete(null); // Reset to original content
      }
    } else {
      translateChapter();
    }
  };

  const getCurrentPageContent = () => {
    // Extract content from the current page
    if (ExecutionEnvironment.canUseDOM) {
      // Try multiple selectors to find the main content area
      const selectors = ['.markdown', '.theme-doc-markdown', 'article.markdown', 'main div'];

      for (const selector of selectors) {
        const element = document.querySelector(selector);
        if (element && element.textContent && element.textContent.trim().length > 50) {
          // Get the content - prefer textContent for better markdown structure
          return element.textContent || element.innerText || '';
        }
      }
    }
    return '';
  };

  return (
    <div className="translation-button-container">
      {translationError && (
        <div className="alert alert--error" style={{ marginBottom: '1rem' }}>
          Translation Error: {translationError}
        </div>
      )}

      <button
        onClick={toggleTranslation}
        disabled={isTranslating}
        className={`button button--${isUrdu ? 'secondary' : 'primary'} translation-toggle-button`}
        title={isUrdu ? 'Translate to English' : 'Translate to Urdu'}
        style={{
          margin: '1rem 0',
          display: 'flex',
          alignItems: 'center',
          gap: '0.5rem'
        }}
      >
        {isTranslating ? (
          <>
            <span className="spinner-border spinner-border-sm" role="status"></span>
            {' '}Translating...
          </>
        ) : (
          <>
            <span className="translation-icon">🌐</span>
            {isUrdu ? 'Show in English' : '.Translate to Urdu'}
          </>
        )}
      </button>

      {isUrdu && (
        <p className="translation-note">
          <small>
            Content translated to Urdu.{' '}
            <button
              onClick={() => {
                setIsUrdu(false);
                if (onTranslationComplete) {
                  onTranslationComplete(null);
                }
              }}
              className="button button--link"
              style={{ textDecoration: 'underline' }}
            >
              Show original
            </button>
          </small>
        </p>
      )}
    </div>
  );
};

export default TranslationButton;