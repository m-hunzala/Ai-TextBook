// translateChapter.js - Client-side Urdu translation function

/**
 * Translates a chapter to Urdu while preserving markdown structure
 * @param {string} chapterId - The ID of the chapter to translate
 * @param {Object} options - Additional options for translation
 * @param {string} options.contentSelector - CSS selector for the content element (default: '.doc-content')
 * @param {string} options.buttonSelector - CSS selector for the translate button (default: '.translate-btn')
 * @param {string} options.targetLang - Target language code (default: 'ur')
 */
async function translateChapter(chapterId, options = {}) {
  const {
    contentSelector = '.doc-content',
    buttonSelector = '.translate-btn',
    targetLang = 'ur'
  } = options;

  // Get references to DOM elements
  const contentElement = document.querySelector(contentSelector);
  const button = document.querySelector(buttonSelector);

  if (!contentElement) {
    throw new Error(`Content element with selector "${contentSelector}" not found`);
  }

  // Show loading state
  const originalButtonText = button?.textContent;
  if (button) {
    button.textContent = 'Translating...';
    button.disabled = true;
  }

  try {
    // Get the original markdown content (this would come from your Docusaurus setup)
    // For this example, we'll assume it's stored in a data attribute or fetched separately
    let originalContent = contentElement.textContent || '';
    
    // In a real Docusaurus setup, you might get the original markdown differently
    // For example, from a hidden element or by making an API call to get the source
    
    // Call the translation API
    const res = await fetch('/api/translate', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        chapterId,
        target: targetLang,
        originalContent // Send the original content to be translated
      })
    });

    if (!res.ok) {
      if (res.status === 401) {
        alert('Please sign in to use translation feature');
        window.location.href = '/login';
        return;
      }
      
      // Handle error response
      const errorData = await res.json();
      console.error('Translation failed:', errorData);
      
      // Show error but keep original content
      if (button) {
        button.textContent = 'Translation failed';
        setTimeout(() => {
          button.textContent = originalButtonText;
          button.disabled = false;
        }, 3000);
      }
      
      // If there's fallback content in the response, use it
      if (errorData.translatedContent) {
        contentElement.innerHTML = window.marked 
          ? marked.parse(errorData.translatedContent) 
          : errorData.translatedContent;
      }
      
      throw new Error(errorData.error || `Translation failed with status: ${res.status}`);
    }

    const { translatedContent, fromCache } = await res.json();

    // Update the content with the translated version
    contentElement.innerHTML = window.marked 
      ? marked.parse(translatedContent) 
      : translatedContent;

    // Update button to show success
    if (button) {
      button.textContent = fromCache ? 'Urdu (cached)' : 'Urdu (translated)';
      
      // Add a class to indicate translated state
      button.classList.add('translated');
    }

    return translatedContent;
  } catch (error) {
    console.error('Error in translateChapter:', error);

    // Revert button state on error
    if (button) {
      button.textContent = 'Error - Retry';
      setTimeout(() => {
        button.textContent = originalButtonText;
        button.disabled = false;
        button.classList.remove('translated');
      }, 3000);
    }

    throw error;
  }
}

/**
 * Toggles translation between original and Urdu
 */
let isTranslated = false;
let originalContent = '';

async function toggleTranslation(chapterId, options = {}) {
  const contentElement = document.querySelector(options.contentSelector || '.doc-content');
  
  if (!contentElement) {
    console.error('Content element not found');
    return;
  }

  if (isTranslated) {
    // Restore original content
    contentElement.innerHTML = window.marked 
      ? marked.parse(originalContent) 
      : originalContent;
    
    // Update button text
    const button = document.querySelector(options.buttonSelector || '.translate-btn');
    if (button) {
      button.textContent = '.Translate to Urdu';
      button.classList.remove('translated');
    }
    
    isTranslated = false;
  } else {
    // Store original content before translating
    originalContent = contentElement.textContent || '';
    
    // Translate to Urdu
    try {
      await translateChapter(chapterId, options);
      isTranslated = true;
    } catch (error) {
      console.error('Translation failed:', error);
    }
  }
}

// Example usage:
// Add this button to your chapter pages:
// <button onclick="toggleTranslation('chapter-1')" class="translate-btn">.Translate to Urdu</button>

export { translateChapter, toggleTranslation };