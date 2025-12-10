import React, { useState, useEffect } from 'react';
import useClientAuth from '../hooks/useClientAuth';

const PersonalizeChapterButton = ({ onPersonalize, chapterId }) => {
  const { client: authClient, session, isReady } = useClientAuth();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');

  const handlePersonalize = async () => {
    if (!session?.user) {
      setError('Please sign in to personalize content');
      return;
    }

    setIsLoading(true);
    setError('');

    try {
      // First, get the user's profile data
      const profileResponse = await fetch(`/api/auth/profile/${session.user.email}`);

      if (!profileResponse.ok) {
        throw new Error('Failed to fetch user profile');
      }

      const profileData = await profileResponse.json();

      // Prepare personalization parameters
      const personalizationParams = {
        userId: session.user.id,
        userEmail: session.user.email,
        programmingBackground: profileData.programming_background,
        aiExperienceLevel: profileData.experience_level,
        hardwareAvailable: profileData.hardware_owned,
        osPreference: profileData.primary_os,
        mainDomain: profileData.main_domain,
        country: profileData.country,
        gpu: profileData.gpu
      };

      // Call the personalization function with user profile
      if (onPersonalize && typeof onPersonalize === 'function') {
        await onPersonalize(personalizationParams);
      } else {
        // If no callback provided, trigger a default personalization effect
        console.log('Personalizing chapter based on user profile:', personalizationParams);

        // Instead of just an alert, we could trigger a visual change to indicate personalization
        // This would typically involve updating content dynamically
        if (typeof document !== 'undefined') {
          document.querySelectorAll('p, li, code, pre').forEach(element => {
            // Example: Temporarily highlight elements that could be personalized
            element.style.transition = 'background-color 0.3s';
            element.style.backgroundColor = '#e6f7ff';
            setTimeout(() => {
              element.style.backgroundColor = '';
            }, 1000);
          });
        }

        // Alert with user's profile info
        alert(`Content will be personalized based on your profile:\n- Programming Background: ${profileData.programming_background || 'Not specified'}\n- AI Experience: ${profileData.experience_level || 'Not specified'}\n- Hardware: ${Array.isArray(profileData.hardware_owned) ? profileData.hardware_owned.join(', ') : 'Not specified'}`);
      }
    } catch (err) {
      setError(err.message);
      console.error('Personalization error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  // Only show the button if the user is logged in (and not still loading)
  // Don't render anything if auth is still loading
  if (!isReady || !session?.user) {
    return null;
  }

  return (
    <div className="margin-bottom--lg">
      <button
        onClick={handlePersonalize}
        disabled={isLoading}
        className="button button--primary button--lg"
        style={{ marginBottom: '1rem' }}
      >
        {isLoading ? 'Personalizing...' : 'Personalize this Chapter'}
      </button>

      {error && (
        <div className="alert alert--danger margin-top--sm">
          {error}
        </div>
      )}
    </div>
  );
};

export default PersonalizeChapterButton;