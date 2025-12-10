// minimalPersonalize.js - Minimal client-side personalization function

// Minimal function to personalize chapter content as shown in the example
async function personalizeChapter(chapterId) {
  try {
    const res = await fetch(`/api/chapters/${chapterId}/personalize`);
    
    if (!res.ok) {
      throw new Error(`Failed to personalize chapter: ${res.status}`);
    }
    
    const { personalizedContent } = await res.json();
    
    // Update the content (assuming marked.js is available for markdown parsing)
    const contentElement = document.querySelector('.doc-content');
    if (contentElement) {
      contentElement.innerHTML = window.marked 
        ? marked.parse(personalizedContent) 
        : personalizedContent; // fallback if marked isn't available
    }
    
    return personalizedContent;
  } catch (error) {
    console.error('Error personalizing chapter:', error);
    throw error;
  }
}

// Enhanced version with error handling and loading states
async function personalizeChapterWithUI(chapterId, options = {}) {
  const {
    contentSelector = '.doc-content',
    loadingSelector = '.loading',
    buttonSelector = '.personalize-btn'
  } = options;

  // Show loading state
  const button = document.querySelector(buttonSelector);
  const originalButtonText = button?.textContent;
  if (button) {
    button.textContent = 'Personalizing...';
    button.disabled = true;
  }

  try {
    const res = await fetch(`/api/chapters/${chapterId}/personalize`);
    
    if (!res.ok) {
      if (res.status === 401) {
        alert('Please sign in to get personalized content');
        window.location.href = '/login'; // Redirect to login
        return;
      }
      throw new Error(`Failed to personalize chapter: ${res.status}`);
    }
    
    const { personalizedContent } = await res.json();
    
    // Update the content
    const contentElement = document.querySelector(contentSelector);
    if (contentElement) {
      contentElement.innerHTML = window.marked 
        ? marked.parse(personalizedContent) 
        : personalizedContent;
    }
    
    // Update button text to indicate success
    if (button) {
      button.textContent = 'Personalized!';
      setTimeout(() => {
        button.textContent = originalButtonText;
        button.disabled = false;
      }, 2000);
    }
    
    return personalizedContent;
  } catch (error) {
    console.error('Error personalizing chapter:', error);
    
    // Revert button state on error
    if (button) {
      button.textContent = 'Error - Retry';
      setTimeout(() => {
        button.textContent = originalButtonText;
        button.disabled = false;
      }, 3000);
    }
    
    throw error;
  }
}

// Usage example:
// Add this button to your chapter pages:
// <button onclick="personalizeChapterWithUI('chapter-1')" class="personalize-btn">Personalize for me</button>

export { personalizeChapter, personalizeChapterWithUI };