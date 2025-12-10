import React from 'react';
import OriginalDocItem from '@theme-original/DocItem';
import PersonalizeChapterButton from '@site/src/components/PersonalizeChapterButton';
import TranslationButton from '@site/src/components/TranslationButton';

export default function DocItem(props) {
  const { content: DocContent } = props;
  const { metadata } = DocContent;

  return (
    <>
      <div className="translation-controls-container" style={{ marginBottom: '1rem' }}>
        <TranslationButton
          chapterId={metadata.unversionedId}
          chapterContent={metadata.content || null} // Pass available content if available
        />
      </div>
      <OriginalDocItem {...props} />
      <div className="container">
        <PersonalizeChapterButton chapterId={metadata.unversionedId} />
      </div>
    </>
  );
}