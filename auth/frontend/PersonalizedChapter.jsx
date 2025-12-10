// PersonalizedChapter.js - Example Docusaurus component for personalized chapters

import React, { useState, useEffect } from 'react';
import { getUserProfile, personalizeChapter } from './clientPersonalize';

const PersonalizedChapter = ({ chapterId, originalMarkdown }) => {
  const [personalizedContent, setPersonalizedContent] = useState(originalMarkdown);
  const [loading, setLoading] = useState(false);
  const [userProfile, setUserProfile] = useState(null);
  const [showPersonalizeButton, setShowPersonalizeButton] = useState(false);

  // Check if user is authenticated and get their profile
  useEffect(() => {
    const checkAuthAndProfile = async () => {
      try {
        const profile = await getUserProfile();
        setUserProfile(profile);
        setShowPersonalizeButton(true); // Show button if user is authenticated
      } catch (error) {
        console.log('User not authenticated, showing default content');
        setShowPersonalizeButton(false);
      }
    };

    checkAuthAndProfile();
  }, []);

  // Function to handle personalization
  const handlePersonalize = async () => {
    if (!userProfile) {
      alert('Please sign in to get personalized content');
      return;
    }

    setLoading(true);
    try {
      const result = await personalizeChapter(chapterId, originalMarkdown);
      setPersonalizedContent(result.personalizedContent);
    } catch (error) {
      console.error('Error personalizing chapter:', error);
      alert('Error personalizing content. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  // Render markdown as HTML
  const renderMarkdown = (content) => {
    // In a real implementation, you'd use a markdown parser like
    // dangerouslySetInnerHTML with a converted markdown string
    // or a React markdown component
    return <div dangerouslySetInnerHTML={{ __html: content }} />;
  };

  return (
    <div className="personalized-chapter-container">
      {showPersonalizeButton && (
        <div className="personalize-controls">
          <button 
            onClick={handlePersonalize} 
            disabled={loading}
            className="personalize-btn"
          >
            {loading ? 'Personalizing...' : 'Personalize for me'}
          </button>
          <div className="user-info">
            {userProfile?.experienceLevel && <span>Level: {userProfile.experienceLevel}</span>}
            {userProfile?.hardwareAvailable && userProfile.hardwareAvailable.length > 0 && (
              <span>Hardware: {userProfile.hardwareAvailable.join(', ')}</span>
            )}
          </div>
        </div>
      )}

      <div className="chapter-content">
        {renderMarkdown(personalizedContent)}
      </div>
    </div>
  );
};

export default PersonalizedChapter;