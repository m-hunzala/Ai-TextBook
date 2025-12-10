import React, { useState, useEffect } from 'react';
import './PersonalizationToggle.css';

const PersonalizationToggle = ({ apiUrl, chapterUrl, initialContent, isAuthenticated }) => {
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isProcessing, setIsProcessing] = useState(false);
  const [currentContent, setCurrentContent] = useState(initialContent);
  const [originalContent, setOriginalContent] = useState(initialContent);
  const [error, setError] = useState('');
  const [showOriginal, setShowOriginal] = useState(false);

  // Get auth token from wherever it's stored in your app
  const getAuthToken = () => {
    // This should match how you store and retrieve the Better-Auth token
    return localStorage.getItem('better-auth-token') || sessionStorage.getItem('better-auth-token');
  };

  const personalizeChapter = async () => {
    if (!isAuthenticated) {
      setError('Please sign in to use personalization features');
      return;
    }

    if (!chapterUrl) {
      setError('Chapter URL not provided');
      return;
    }

    setIsProcessing(true);
    setError('');

    try {
      const token = getAuthToken();
      if (!token) {
        throw new Error('Authentication token not found');
      }

      const response = await fetch(`${apiUrl}/personalization/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify({
          chapter_url: chapterUrl,
          original_content: originalContent,
          force_regenerate: false
        })
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to personalize content');
      }

      const data = await response.json();
      
      if (data.personalized_content) {
        setCurrentContent(data.personalized_content);
        setIsPersonalized(true);
        setShowOriginal(false);
      } else {
        throw new Error('No personalized content returned');
      }
    } catch (err) {
      console.error('Personalization error:', err);
      setError(err.message || 'Failed to personalize content');
    } finally {
      setIsProcessing(false);
    }
  };

  const toggleContent = () => {
    if (isPersonalized) {
      setShowOriginal(!showOriginal);
    }
  };

  const clearPersonalization = async () => {
    if (!isPersonalized || !chapterUrl) return;

    try {
      const token = getAuthToken();
      if (!token) return;

      const response = await fetch(`${apiUrl}/personalization/content/${encodeURIComponent(chapterUrl)}`, {
        method: 'DELETE',
        headers: {
          'Authorization': `Bearer ${token}`
        }
      });

      if (response.ok) {
        setIsPersonalized(false);
        setShowOriginal(false);
        setCurrentContent(originalContent);
      }
    } catch (err) {
      console.error('Error clearing personalization:', err);
    }
  };

  // Update content when initialContent changes
  useEffect(() => {
    setOriginalContent(initialContent);
    if (!isPersonalized) {
      setCurrentContent(initialContent);
    }
  }, [initialContent]);

  return (
    <div className="personalization-container">
      {error && <div className="personalization-error">{error}</div>}
      
      <div className="personalization-controls">
        {!isPersonalized ? (
          <button 
            className="personalize-button"
            onClick={personalizeChapter}
            disabled={isProcessing || !isAuthenticated}
            title={isAuthenticated ? "Personalize this chapter based on your profile" : "Sign in to personalize content"}
          >
            {isProcessing ? 'Personalizing...' : 'Personalize this chapter'}
          </button>
        ) : (
          <div className="personalization-active-controls">
            <button 
              className={`content-toggle-button ${showOriginal ? 'active' : ''}`}
              onClick={toggleContent}
              disabled={isProcessing}
            >
              {showOriginal ? 'View Personalized' : 'View Original'}
            </button>
            
            <button 
              className="regenerate-button"
              onClick={personalizeChapter}
              disabled={isProcessing}
              title="Regenerate personalized content"
            >
              {isProcessing ? 'Regenerating...' : 'Regenerate'}
            </button>
            
            <button 
              className="clear-button"
              onClick={clearPersonalization}
              title="Clear personalized version"
            >
              Clear
            </button>
          </div>
        )}
      </div>

      <div className="personalization-content">
        {showOriginal ? (
          <div className="original-content">
            <div className="content-badge original">Original Content</div>
            <div dangerouslySetInnerHTML={{ __html: originalContent }} />
          </div>
        ) : (
          <div className="personalized-content">
            {isPersonalized && <div className="content-badge personalized">Personalized for you</div>}
            <div dangerouslySetInnerHTML={{ __html: currentContent }} />
          </div>
        )}
      </div>
    </div>
  );
};

export default PersonalizationToggle;