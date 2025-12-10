import React, { useState, useEffect } from 'react';
import { MDXProvider } from '@mdx-js/react';
import TranslationButton from '../components/TranslationButton';

/**
 * A wrapper component for Docusaurus docs that adds translation functionality
 */
const TranslatableDoc = ({ children, content: docContent }) => {
  const [translatedContent, setTranslatedContent] = useState(null);
  const [currentContent, setCurrentContent] = useState(children);

  // This component would be used to wrap doc content
  // In practice, you'd integrate this with Docusaurus swizzling or a layout component

  useEffect(() => {
    // When translated content is available, update what's displayed
    if (translatedContent) {
      setCurrentContent(<div className="markdown" dangerouslySetInnerHTML={{ __html: translatedContent }} />);
    } else {
      setCurrentContent(children);
    }
  }, [translatedContent, children]);

  // Extract doc ID from the URL or metadata if available
  const getDocId = () => {
    // Implementation would extract doc ID from current route
    if (typeof window !== 'undefined') {
      const pathParts = window.location.pathname.split('/').filter(p => p);
      if (pathParts[0] === 'docs' && pathParts.length > 1) {
        return pathParts.slice(1).join('-');
      }
    }
    return 'current';
  };

  return (
    <div className="translatable-doc-container">
      <TranslationButton 
        chapterId={getDocId()} 
        chapterContent={typeof children === 'string' ? children : 
          (children.props?.metadata?.content || 'Content not available')}
        onTranslationComplete={setTranslatedContent}
      />
      
      <div className="doc-content">
        {currentContent}
      </div>
    </div>
  );
};

export default TranslatableDoc;