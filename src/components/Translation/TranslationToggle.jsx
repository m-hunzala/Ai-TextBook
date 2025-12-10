import React, { useState, useEffect } from 'react';
import './TranslationToggle.css';

const TranslationToggle = ({ apiUrl, chapterUrl, initialContent }) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [translationStatus, setTranslationStatus] = useState(null); // null, 'loading', 'success', 'error'
  const [currentContent, setCurrentContent] = useState(initialContent);
  const [originalContent, setOriginalContent] = useState(initialContent);
  const [translatedContent, setTranslatedContent] = useState(null);
  const [error, setError] = useState('');
  const [viewMode, setViewMode] = useState('english'); // 'english', 'urdu', 'side-by-side'
  const [availableLanguages, setAvailableLanguages] = useState(['en']);

  // Function to extract and preserve code blocks during translation
  const preserveCodeBlocks = (content) => {
    // This is a simple implementation - in a real system, you'd have more robust parsing
    const codeBlockRegex = /(```[\s\S]*?```|`[^`]*`)/g;
    const matches = content.match(codeBlockRegex) || [];
    
    // Replace code blocks with placeholders to preserve them during translation
    let processedContent = content;
    matches.forEach((match, index) => {
      const placeholder = `__CODE_BLOCK_${index}__`;
      processedContent = processedContent.replace(match, placeholder);
    });
    
    return { processedContent, originalBlocks: matches };
  };

  // Function to restore code blocks after translation
  const restoreCodeBlocks = (translatedContent, originalBlocks) => {
    let restoredContent = translatedContent;
    originalBlocks.forEach((block, index) => {
      const placeholder = `__CODE_BLOCK_${index}__`;
      restoredContent = restoredContent.replace(placeholder, block);
    });
    
    return restoredContent;
  };

  const translateToUrdu = async () => {
    if (!chapterUrl) {
      setError('Chapter URL not provided');
      return;
    }

    setIsTranslating(true);
    setTranslationStatus('loading');
    setError('');

    try {
      const response = await fetch(`${apiUrl}/translation/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_url: chapterUrl,
          original_content: originalContent,
          target_language: 'ur',
          force_regenerate: false
        })
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to translate content');
      }

      const data = await response.json();
      
      if (data.translated_content) {
        setTranslatedContent(data.translated_content);
        setCurrentContent(data.translated_content);
        setTranslationStatus('success');
        setViewMode('urdu'); // Switch to Urdu view after successful translation
        
        // Update available languages
        if (!availableLanguages.includes('ur')) {
          setAvailableLanguages([...availableLanguages, 'ur']);
        }
      } else {
        throw new Error('No translated content returned');
      }
    } catch (err) {
      console.error('Translation error:', err);
      setError(err.message || 'Failed to translate content');
      setTranslationStatus('error');
    } finally {
      setIsTranslating(false);
    }
  };

  const toggleViewMode = (mode) => {
    setViewMode(mode);
    switch (mode) {
      case 'english':
        setCurrentContent(originalContent);
        break;
      case 'urdu':
        if (translatedContent) {
          setCurrentContent(translatedContent);
        } else {
          translateToUrdu(); // If no translation exists, trigger translation
        }
        break;
      case 'side-by-side':
        // For side-by-side, we'll render both versions in the UI
        setCurrentContent(null); // We'll handle side-by-side separately
        break;
      default:
        setCurrentContent(originalContent);
    }
  };

  const clearTranslation = async () => {
    if (!chapterUrl) return;

    try {
      const response = await fetch(`${apiUrl}/translation/content/${encodeURIComponent(chapterUrl)}/ur`, {
        method: 'DELETE'
      });

      if (response.ok) {
        setTranslatedContent(null);
        setCurrentContent(originalContent);
        setViewMode('english');
        setAvailableLanguages(availableLanguages.filter(lang => lang !== 'ur'));
      }
    } catch (err) {
      console.error('Error clearing translation:', err);
    }
  };

  // Update content when initialContent changes
  useEffect(() => {
    setOriginalContent(initialContent);
    if (viewMode === 'english') {
      setCurrentContent(initialContent);
    }
  }, [initialContent]);

  // Get available languages for this chapter
  useEffect(() => {
    const fetchAvailableLanguages = async () => {
      try {
        const response = await fetch(`${apiUrl}/translation/languages/${encodeURIComponent(chapterUrl)}`);
        if (response.ok) {
          const data = await response.json();
          setAvailableLanguages(data.languages || ['en']);
        }
      } catch (err) {
        console.error('Error fetching available languages:', err);
      }
    };

    if (chapterUrl) {
      fetchAvailableLanguages();
    }
  }, [chapterUrl]);

  return (
    <div className="translation-container">
      {error && <div className="translation-error">{error}</div>}
      
      <div className="translation-controls">
        <div className="translation-button-group">
          <button 
            className={`translate-button ${translatedContent ? 'translated' : ''}`}
            onClick={translateToUrdu}
            disabled={isTranslating || translationStatus === 'loading'}
            title="Translate this chapter to Urdu"
          >
            {isTranslating || translationStatus === 'loading' 
              ? 'Translating...' 
              : translatedContent ? 'Re-translate to Urdu' : '.Translate to Urdu'}
          </button>
          
          {translatedContent && (
            <button 
              className="clear-translation-button"
              onClick={clearTranslation}
              title="Clear translated version"
            >
              Clear Translation
            </button>
          )}
        </div>
        
        {translatedContent && (
          <div className="view-mode-toggle">
            <button 
              className={`view-mode-btn ${viewMode === 'english' ? 'active' : ''}`}
              onClick={() => toggleViewMode('english')}
            >
              English
            </button>
            <button 
              className={`view-mode-btn ${viewMode === 'urdu' ? 'active' : ''}`}
              onClick={() => toggleViewMode('urdu')}
            >
              Urdu
            </button>
            <button 
              className={`view-mode-btn ${viewMode === 'side-by-side' ? 'active' : ''}`}
              onClick={() => toggleViewMode('side-by-side')}
            >
              Side-by-Side
            </button>
          </div>
        )}
      </div>

      <div className="translation-content">
        {viewMode === 'side-by-side' ? (
          <div className="side-by-side-view">
            <div className="content-column english-column">
              <div className="content-header">
                <span className="language-badge english">English</span>
              </div>
              <div 
                className="content-body" 
                dangerouslySetInnerHTML={{ __html: originalContent }} 
              />
            </div>
            <div className="content-column urdu-column">
              <div className="content-header">
                <span className="language-badge urdu">اردو</span>
              </div>
              <div 
                className="content-body" 
                dangerouslySetInnerHTML={{ __html: translatedContent }} 
              />
            </div>
          </div>
        ) : (
          <div className={`single-view ${viewMode}`}>
            <div className="content-header">
              <span className={`language-badge ${viewMode}`}>
                {viewMode === 'english' ? 'English' : 'اردو'}
              </span>
            </div>
            <div 
              className="content-body" 
              dangerouslySetInnerHTML={{ __html: currentContent }} 
            />
          </div>
        )}
      </div>
    </div>
  );
};

export default TranslationToggle;