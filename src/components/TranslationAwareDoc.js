import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import { useUserData } from '../contexts/UserContext';
import useTranslation from '../hooks/useTranslation';
import { useCurrentChapter } from '../hooks/useCurrentChapter';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';

/**
 * A translation-aware document component that manages content replacement
 */
const TranslationAwareDoc = ({ children }) => {
  const { isTranslating, translationError, isUrdu, translateContent, resetTranslation } = useTranslation();
  const { userData, isAuthenticated } = useUserData();
  const { docId, title, getRawContent } = useCurrentChapter();
  const location = useLocation();
  
  const [translatedContent, setTranslatedContent] = useState(null);
  const [showTranslation, setShowTranslation] = useState(false);
  const [contentToDisplay, setContentToDisplay] = useState(children);

  // Toggle translation state
  const toggleTranslation = async () => {
    if (showTranslation) {
      // Switch back to original
      setShowTranslation(false);
      setTranslatedContent(null);
      setContentToDisplay(children);
    } else {
      // Translate content
      if (!isAuthenticated) {
        alert('Please log in to use the translation feature');
        return;
      }

      try {
        const rawContent = await getRawContent();
        if (!rawContent) {
          throw new Error('Could not retrieve chapter content');
        }

        const translated = await translateContent(docId, rawContent, userData?.token);
        setTranslatedContent(translated);
        setShowTranslation(true);
      } catch (err) {
        console.error('Translation error:', err);
      }
    }
  };

  // Update content to display when translation state changes
  useEffect(() => {
    if (showTranslation && translatedContent) {
      setContentToDisplay(
        <div className="markdown">
          <ReactMarkdown remarkPlugins={[remarkGfm]}>
            {translatedContent}
          </ReactMarkdown>
        </div>
      );
    } else {
      setContentToDisplay(children);
    }
  }, [showTranslation, translatedContent, children]);

  return (
    <div className="translation-aware-doc">
      <div className="translation-controls" style={{ margin: '1rem 0' }}>
        <button
          onClick={toggleTranslation}
          disabled={isTranslating || (!isAuthenticated && showTranslation)}
          className={`button button--${showTranslation ? 'secondary' : 'primary'}`}
          style={{ 
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
              <span>🌐</span>
              {showTranslation ? 'Show in English' : '.Translate to Urdu'}
            </>
          )}
        </button>
        
        {translationError && (
          <div className="alert alert--error" style={{ margin: '0.5rem 0 0' }}>
            {translationError}
          </div>
        )}
        
        {showTranslation && (
          <p style={{ fontSize: '0.85em', color: '#666', marginTop: '0.5rem' }}>
            Content translated to Urdu.{' '}
            <button 
              onClick={() => {
                setShowTranslation(false);
                setTranslatedContent(null);
                setContentToDisplay(children);
              }}
              className="button button--link"
              style={{ textDecoration: 'underline' }}
            >
              Show original
            </button>
          </p>
        )}
      </div>
      
      <div className="doc-content">
        {contentToDisplay}
      </div>
    </div>
  );
};

export default TranslationAwareDoc;