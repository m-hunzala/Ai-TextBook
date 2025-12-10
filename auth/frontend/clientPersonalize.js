// clientPersonalize.js - Client-side snippet for chapter personalization

// Function to get user profile
const getUserProfile = async () => {
  const response = await fetch('/api/auth/profile');
  if (!response.ok) {
    if (response.status === 401) {
      throw new Error('User not authenticated');
    }
    throw new Error('Failed to get user profile');
  }
  return response.json();
};

// Function to personalize chapter content
const personalizeChapter = async (chapterId, markdownContent) => {
  const response = await fetch(`/api/chapters/${chapterId}/personalize`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ markdown: markdownContent })
  });
  
  if (!response.ok) {
    throw new Error('Failed to personalize chapter');
  }
  
  return response.json();
};

// Function to convert markdown to HTML (using a library like marked.js)
const markdownToHtml = (markdown) => {
  // This would typically use a library like marked.js or showdown.js
  // For now, returning as-is, but in practice you'd convert markdown to HTML
  return markdown;
};

// Main function to replace chapter content with personalized version
export const replaceChapterContent = async (chapterId, originalContentElementId = 'chapter-content') => {
  try {
    // Get user profile to determine personalization rules
    const userProfile = await getUserProfile();
    
    // Get original markdown content
    const originalContent = document.getElementById(originalContentElementId)?.textContent || '';
    
    // Personalize the chapter
    const personalizationResult = await personalizeChapter(chapterId, originalContent);
    
    // Update the DOM with personalized content
    const contentElement = document.getElementById(originalContentElementId);
    if (contentElement) {
      // Convert markdown to HTML and insert
      contentElement.innerHTML = markdownToHtml(personalizationResult.personalizedContent);
      
      // Optionally trigger any Docusaurus-specific lifecycle methods
      if (window.location && window.location.reload) {
        // If needed, trigger any re-rendering for Docusaurus
        // This might include triggering MathJax, code highlighting, etc.
      }
    }
    
    return personalizationResult.personalizedContent;
  } catch (error) {
    console.error('Error personalizing chapter:', error);
    throw error;
  }
};

// React component example
export const PersonalizedChapter = ({ chapterId, defaultContent }) => {
  const [content, setContent] = React.useState(defaultContent);
  const [loading, setLoading] = React.useState(true);
  const [error, setError] = React.useState(null);

  React.useEffect(() => {
    const loadPersonalizedContent = async () => {
      try {
        // Get personalized content
        const result = await personalizeChapter(chapterId, defaultContent);
        setContent(result.personalizedContent);
      } catch (err) {
        setError(err.message);
        console.error('Error loading personalized content:', err);
      } finally {
        setLoading(false);
      }
    };

    loadPersonalizedContent();
  }, [chapterId]);

  if (loading) return <div>Loading personalized content...</div>;
  if (error) return <div>Error: {error}</div>;

  return (
    <div 
      className="personalized-chapter" 
      dangerouslySetInnerHTML={{ __html: markdownToHtml(content) }} 
    />
  );
};

// Standalone function to add "Personalize for me" button
export const addPersonalizeButton = (chapterId, containerElementId = 'chapter-container') => {
  // Create personalize button
  const button = document.createElement('button');
  button.id = 'personalize-button';
  button.textContent = 'Personalize for me';
  button.className = 'personalize-btn';
  button.onclick = async () => {
    try {
      button.textContent = 'Personalizing...';
      button.disabled = true;
      
      await replaceChapterContent(chapterId);
      
      button.textContent = 'Personalized!';
      setTimeout(() => {
        button.textContent = 'Personalize for me';
        button.disabled = false;
      }, 2000);
    } catch (error) {
      console.error('Personalization failed:', error);
      button.textContent = 'Error - try again';
      setTimeout(() => {
        button.textContent = 'Personalize for me';
        button.disabled = false;
      }, 2000);
    }
  };

  // Insert button into the page
  const container = document.getElementById(containerElementId);
  if (container) {
    container.insertBefore(button, container.firstChild);
  }
};