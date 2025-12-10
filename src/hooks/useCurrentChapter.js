import { useLocation } from '@docusaurus/router';
import { useActiveDocContext } from '@docusaurus/useActiveDocContext';
import { docsVersion } from '@docusaurus/plugin-content-docs/client';

/**
 * Custom hook to detect the current chapter and get its raw content
 * @returns Object with current chapter info and content
 */
export const useCurrentChapter = () => {
  const location = useLocation();
  const activeDoc = useActiveDocContext();
  
  // Extract the document ID from the current location
  const currentDocId = activeDoc?.metadata?.unversionedId || null;
  
  // Try to get the raw markdown content
  const getRawContent = async () => {
    if (!currentDocId) {
      return null;
    }

    try {
      // Try to construct the path to the raw markdown file
      // This follows Docusaurus conventions
      const docRoute = activeDoc?.metadata?.source || '';
      
      // For Docusaurus, the source field contains path relative to the docs directory
      // We'll fetch the raw content from the appropriate route
      if (docRoute) {
        // This is a simplified implementation
        // In a real implementation, you would need to either:
        // 1. Store the raw content in the build process
        // 2. Fetch it from your content source (GitHub, CMS, etc.)
        // 3. Use a custom plugin to expose raw content
        
        // For the purpose of this implementation, we'll use a runtime approach
        // to get the content that's currently rendered
        if (typeof document !== 'undefined') {
          const mainElement = document.querySelector('main .markdown');
          if (mainElement) {
            // Return the content as best we can extract it
            // In a real implementation, you'd want to get the actual raw markdown
            return mainElement.innerText || mainElement.textContent || '';
          }
        }
      }
    } catch (error) {
      console.error('Error getting raw content:', error);
    }
    
    return null;
  };

  // Function to extract the current path-based doc ID
  const getCurrentDocId = () => {
    if (activeDoc?.metadata?.unversionedId) {
      return activeDoc.metadata.unversionedId;
    }
    
    // Fallback to parsing the URL
    const pathParts = location.pathname.split('/').filter(part => part);
    
    // Docusaurus docs paths are usually /docs/category/document or /docs/document
    if (pathParts[0] === 'docs' && pathParts.length > 1) {
      // Join the remaining parts to form the doc ID
      return pathParts.slice(1).join('/');
    }
    
    return null;
  };

  return {
    docId: currentDocId || getCurrentDocId(),
    title: activeDoc?.metadata?.title || '',
    description: activeDoc?.metadata?.description || '',
    getRawContent,
    currentPath: location.pathname
  };
};