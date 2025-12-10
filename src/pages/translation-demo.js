import React, { useState } from 'react';
import Layout from '@theme/Layout';
import TranslatorButton from '@site/src/components/TranslatorButton';

export default function TranslationDemoPage() {
  const [content, setContent] = useState(`# Translation Feature Demo

This is a demonstration of the translation feature. The content can be translated to Urdu while preserving technical terms and code.

## Sample Code Block

\`\`\`python
def hello_robot():
    print("Hello, Robot!")
    return "Robot initialized"
\`\`\`

This function demonstrates a simple robot initialization routine.

## Technical Terms

- ROS (Robot Operating System)
- LIDAR (Light Detection and Ranging)
- URDF (Unified Robot Description Format)
- PID (Proportional-Integral-Derivative) controller

These technical terms will remain in English during translation to maintain accuracy.

## More Complex Example

\`\`\`urdf
<robot name="simple_robot">
  <link name="chassis">
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
  </link>
</robot>
\`\`\`

This URDF snippet defines a simple robot with a chassis link.
`);

  const [isTranslated, setIsTranslated] = useState(false);

  return (
    <Layout title="Translation Feature Demo" description="Demonstrate translation to Urdu">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <h1>Translation Feature Demo</h1>
            
            <TranslatorButton 
              chapterId="translation-demo"
              chapterContent={content}
              onTranslate={(translatedContent, wasCached) => {
                setContent(translatedContent);
                setIsTranslated(true);
              }}
              language="ur"
            />
            
            {isTranslated && (
              <div className="alert alert--success margin-bottom--md">
                Content has been translated to Urdu. Technical terms and code blocks have been preserved.
              </div>
            )}
            
            <div className="markdown">
              <div dangerouslySetInnerHTML={{ __html: content.replace(/\n/g, '<br />') }} />
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}