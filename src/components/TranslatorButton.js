import { useState, useEffect } from 'react';

const TranslatorButton = ({ 
  chapterId, 
  chapterContent, 
  onTranslate, 
  className = '',
  language = 'ur' 
}) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [error, setError] = useState(null);
  const [showMessage, setShowMessage] = useState(false);

  const handleTranslate = async () => {
    if (isTranslating) return;
    
    setIsTranslating(true);
    setError(null);
    
    try {
      const response = await fetch('/api/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          content: chapterContent,
          language: language
        })
      });
      
      const data = await response.json();
      
      if (!response.ok) {
        throw new Error(data.error || 'Translation failed');
      }
      
      // Update the parent component with the translated content
      if (onTranslate && data.translated_content) {
        onTranslate(data.translated_content, data.cached);
      }
      
      setIsTranslated(true);
      setShowMessage(true);
      setTimeout(() => setShowMessage(false), 3000); // Hide message after 3 seconds
      
    } catch (err) {
      console.error('Translation error:', err);
      setError(err.message);
      
      // If there's a partial/fallback translation in the response, use that
      if (err.translated_content) {
        if (onTranslate) {
          onTranslate(err.translated_content, false);
        }
        setIsTranslated(true);
      }
    } finally {
      setIsTranslating(false);
    }
  };

  // Reset translation status if content changes
  useEffect(() => {
    setIsTranslated(false);
  }, [chapterContent]);

  return (
    <div className={`translator-container ${className}`}>
      <button
        onClick={handleTranslate}
        disabled={isTranslating}
        className={`
          translate-btn
          ${isTranslated ? 'translated' : ''}
          ${isTranslating ? 'translating' : ''}
          bg-blue-600 hover:bg-blue-700 text-white font-medium py-2 px-4 rounded-md 
          inline-flex items-center transition-colors duration-200
          disabled:opacity-50 disabled:cursor-not-allowed
        `}
      >
        {isTranslating ? (
          <>
            <svg className="animate-spin -ml-1 mr-2 h-4 w-4 text-white" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
              <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
              <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
            </svg>
            ترجمہ ہو رہا ہے...
          </>
        ) : isTranslated ? (
          <>
            <svg className="w-4 h-4 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth="2" d="M5 13l4 4L19 7"></path>
            </svg>
            اردو میں ترجمہ شدہ
          </>
        ) : (
          'اردو میں ترجمہ کریں'
        )}
      </button>
      
      {showMessage && (
        <div className="mt-2 p-2 bg-green-100 text-green-700 rounded-md text-sm inline-block ml-2">
          {isTranslated ? 'چیپٹر کا ترجمہ مکمل ہو گیا ہے!' : 'ترجمہ مکمل ہو گیا!'}
        </div>
      )}
      
      {error && (
        <div className="mt-2 p-2 bg-red-100 text-red-700 rounded-md text-sm inline-block ml-2">
          خامی: {error}
        </div>
      )}
      
      <style jsx>{`
        .translate-btn {
          transition: all 0.3s ease;
        }
        .translate-btn:hover:not(:disabled) {
          transform: translateY(-1px);
          box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }
        .translator-container {
          margin-bottom: 1rem;
        }
      `}</style>
    </div>
  );
};

export default TranslatorButton;