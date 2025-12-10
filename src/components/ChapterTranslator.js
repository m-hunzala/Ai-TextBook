import React, { useState, useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import { useUserData } from '../contexts/UserContext';
import useTranslation from '../hooks/useTranslation';
import { useCurrentChapter } from '../hooks/useCurrentChapter';

/**
 * A comprehensive translation component that handles content replacement
 */
const ChapterTranslator = () => {
  const { isTranslating, translationError, isUrdu, translateContent, resetTranslation } = useTranslation();
  const { userData, isAuthenticated } = useUserData();
  const { docId, getRawContent } = useCurrentChapter();
  const location = useLocation();
  
  const [translatedHtml, setTranslatedHtml] = useState(null);
  const [originalHtml, setOriginalHtml] = useState(null);
  const contentRef = useRef(null);
  
  // Store original content when component mounts
  useEffect(() => {
    const storeOriginalContent = async () => {
      if (contentRef.current && !originalHtml) {
        setOriginalHtml(contentRef.current.innerHTML);
      } else {
        // Try to get content from the main markdown area
        const mainElement = document.querySelector('main .markdown');
        if (mainElement && !originalHtml) {
          setOriginalHtml(mainElement.innerHTML);
        }
      }
    };
    
    storeOriginalContent();
  }, [originalHtml]);
  
  // Apply translated content to the DOM
  useEffect(() => {
    if (translatedHtml && contentRef.current) {
      contentRef.current.innerHTML = translatedHtml;
    } else if (originalHtml && contentRef.current && !translatedHtml) {
      contentRef.current.innerHTML = originalHtml;
    }
  }, [translatedHtml, originalHtml]);

  const translateCurrentChapter = async () => {
    if (!isAuthenticated) {
      alert('Please log in to use the translation feature');
      return;
    }

    try {
      // Get raw markdown content
      const rawContent = await getRawContent();
      if (!rawContent) {
        throw new Error('Could not retrieve chapter content');
      }

      const translatedContent = await translateContent(docId, rawContent, userData?.token);
      
      // Convert markdown to HTML for display
      // In a real implementation, you'd use a markdown-to-HTML converter
      // that's compatible with your existing Docusaurus setup
      setTranslatedHtml(convertMarkdownToHtml(translatedContent));
    } catch (err) {
      console.error('Translation error:', err);
    }
  };

  // Simple markdown to HTML conversion (simplified)
  // In practice, you'd use a library like marked or react-markdown
  const convertMarkdownToHtml = (markdown) => {
    // This is a simplified implementation
    // Real implementation would use proper markdown parser
    let html = markdown
      // Headers
      .replace(/^###### (.*$)/gim, '<h6>$1</h6>')
      .replace(/^##### (.*$)/gim, '<h5>$1</h5>')
      .replace(/^#### (.*$)/gim, '<h4>$1</h4>')
      .replace(/^### (.*$)/gim, '<h3>$1</h3>')
      .replace(/^## (.*$)/gim, '<h2>$1</h2>')
      .replace(/^# (.*$)/gim, '<h1>$1</h1>')
      // Bold
      .replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>')
      .replace(/__(.*?)__/g, '<strong>$1</strong>')
      // Italic
      .replace(/\*(.*?)\*/g, '<em>$1</em>')
      .replace(/_(.*?)_/g, '<em>$1</em>')
      // Links
      .replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2">$1</a>')
      // Images
      .replace(/!\[([^\]]*)\]\(([^)]+)\)/g, '<img alt="$1" src="$2" />')
      // Code blocks
      .replace(/```([\s\S]*?)```/g, '<pre><code>$1</code></pre>')
      // Inline code
      .replace(/`([^`]+)`/g, '<code>$1</code>')
      // Unordered lists
      .replace(/^- (.*$)/gim, '<li>$1</li>')
      // Line breaks
      .replace(/\n\n/g, '</p><p>')
      .replace(/\n/g, '<br />');
    
    // Wrap in paragraph tags if needed
    if (!html.startsWith('<')) {
      html = `<p>${html}</p>`;
    }
    
    return html;
  };

  const toggleTranslation = () => {
    if (isUrdu) {
      // Reset to original
      resetTranslation();
      setTranslatedHtml(null);
    } else {
      // Translate
      translateCurrentChapter();
    }
  };

  return (
    <div className="chapter-translator">
      <button
        onClick={toggleTranslation}
        disabled={isTranslating || (!isAuthenticated && !isUrdu)}
        className={`button button--${isUrdu ? 'secondary' : 'primary'}`}
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
            <span>🌐</span>
            {isUrdu ? 'Show in English' : '.Translate to Urdu'}
          </>
        )}
      </button>
      
      {translationError && (
        <div className="alert alert--error" style={{ margin: '1rem 0' }}>
          {translationError}
        </div>
      )}
      
      <div 
        ref={contentRef} 
        className="chapter-content"
        style={{ minHeight: '100px' }}
      >
        {/* Content will be managed by useEffect */}
      </div>
      
      {isUrdu && (
        <p style={{ fontSize: '0.85em', color: '#666', marginTop: '1rem' }}>
          Content translated to Urdu.{' '}
          <button 
            onClick={() => {
              resetTranslation();
              setTranslatedHtml(null);
            }}
            className="button button--link"
            style={{ textDecoration: 'underline' }}
          >
            Show original
          </button>
        </p>
      )}
    </div>
  );
};

export default ChapterTranslator;