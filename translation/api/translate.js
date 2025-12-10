// pages/api/translate.js (for Next.js) or equivalent serverless function

import { neon } from '@neondatabase/serverless';
import OpenAI from 'openai';

const sql = neon(process.env.NEON_DB_URL || '');
const openai = new OpenAI({
  apiKey: process.env.OPENAI_API_KEY,
});

export default async function handler(req, res) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { chapter_id, content, language = 'ur' } = req.body;

    if (!chapter_id || !content) {
      return res.status(400).json({ error: 'chapter_id and content are required' });
    }

    if (language !== 'ur') {
      return res.status(400).json({ error: 'Only Urdu (ur) translation is supported' });
    }

    // Check cache first
    const cacheResult = await sql(
      'SELECT translated_content FROM chapter_translations WHERE chapter_id = $1 AND language = $2',
      [chapter_id, language]
    );

    if (cacheResult.length > 0) {
      return res.status(200).json({
        translated_content: cacheResult[0].translated_content,
        cached: true,
        message: 'Retrieved from cache'
      });
    }

    // Proceed with translation
    const translatedContent = await translateToUrdu(content);

    // Cache the result
    await sql(
      `INSERT INTO chapter_translations (chapter_id, language, original_content, translated_content, created_at)
       VALUES ($1, $2, $3, $4, CURRENT_TIMESTAMP)
       ON CONFLICT (chapter_id, language)
       DO UPDATE SET
         original_content = EXCLUDED.original_content,
         translated_content = EXCLUDED.translated_content,
         created_at = CURRENT_TIMESTAMP`,
      [chapter_id, language, content, translatedContent]
    );

    res.status(200).json({
      translated_content: translatedContent,
      cached: false,
      message: 'Successfully translated and cached'
    });

  } catch (error) {
    console.error('Translation error:', error);
    
    // Return error but include a fallback response
    res.status(500).json({
      error: 'Translation failed',
      translated_content: createFallbackTranslation(content),
      message: 'Translation failed, showing original content as fallback'
    });
  }
}

async function translateToUrdu(content) {
  // Define technical terms to preserve
  const technicalTerms = [
    // Programming terms
    'def', 'class', 'import', 'from', 'as', 'for', 'while', 'if', 'else', 'elif',
    'try', 'except', 'finally', 'with', 'lambda', 'return', 'yield', 'pass', 'break',
    'continue', 'True', 'False', 'None', 'self', 'cls', 'async', 'await',
    
    // Common tech terms
    'API', 'JSON', 'XML', 'HTML', 'CSS', 'URL', 'HTTP', 'HTTPS', 'SQL', 'NoSQL',
    'AI', 'ML', 'NLP', 'CV', 'IoT', 'GPU', 'CPU', 'RAM', 'ROM', 'OS', 'SDK', 'IDE',
    'URDF', 'ROS', 'SLAM', 'PID', 'LIDAR', 'IMU', 'SLAM', 'Docker', 'Kubernetes',
    'Git', 'GitHub', 'GitLab', 'CI/CD', 'DevOps', 'API', 'SDK', 'TCP/IP', 'UDP',
    
    // Common variables and classes
    'robot', 'sensor', 'actuator', 'controller', 'motion', 'navigation', 'planning',
    'kinematics', 'dynamics', 'perception', 'manipulation', 'grasping', 
    'localization', 'mapping', 'vision', 'camera', 'lidar', 'imu', 'gyro', 'accelerometer',
    
    // Robotics libraries
    'numpy', 'pandas', 'matplotlib', 'scipy', 'sklearn', 'opencv', 'torch', 'tensorflow',
    'pytorch', 'keras', 'seaborn', 'plotly', 'scikit', 'robotics', 'moveit', 'gazebo',
    'rviz', 'ros', 'ros2', 'rclpy', 'rospy', 'catkin', 'ament', 'colcon', 'ament',
  ];

  // Extract code blocks to preserve them
  const codeBlocks = [];
  let tempContent = content;

  // Match code blocks (```lang\n...code...\n```)
  const codeBlockRegex = /```[\s\S]*?```/g;
  let match;
  while ((match = codeBlockRegex.exec(tempContent)) !== null) {
    const placeholder = `__CODE_BLOCK_${codeBlocks.length}__`;
    codeBlocks.push(match[0]);
    tempContent = tempContent.replace(match[0], placeholder);
  }

  // Now send the content without code blocks to the LLM for translation
  const translationPrompt = `
Translate the following text to formal Urdu. Preserve all technical terms, proper nouns, and code placeholders as they are.
Do not translate code blocks or technical identifiers.
Here is the text to translate:

${tempContent}

Remember to keep technical terms, variable names, and code placeholders in English.
`;

  try {
    const response = await openai.chat.completions.create({
      model: "gpt-3.5-turbo",
      messages: [
        {
          role: "system",
          content: "You are a professional translator specializing in technical content. Translate to formal Urdu while preserving technical terms, code blocks, and identifiers in English. Maintain the structure and formatting of the original content."
        },
        {
          role: "user",
          content: translationPrompt
        }
      ],
      temperature: 0.3,
      max_tokens: Math.min(content.length * 4, 4000), // Rough estimate: 4x the original length
    });

    let translatedContent = response.choices[0].message.content;

    // Restore code blocks
    for (let i = 0; i < codeBlocks.length; i++) {
      const placeholder = `__CODE_BLOCK_${i}__`;
      translatedContent = translatedContent.replace(placeholder, codeBlocks[i]);
    }

    return translatedContent;

  } catch (llmError) {
    console.error('LLM Translation error:', llmError);
    throw new Error(`LLM translation failed: ${llmError.message}`);
  }
}

function createFallbackTranslation(content) {
  // Create a simple side-by-side view when translation fails
  return `
## ترجمہ ناکام ہوا (Translation Failed)  
کچھ مسئلہ آ گیا ہے۔ درج ذیل اصل مواد دکھایا جا رہا ہے۔

---

**اصل/Original Content:**
\`\`\`
${content}
\`\`\`

\n\n---
\n\n**نوٹ/Note:**
ترجمہ کی سہولت دستیاب نہیں ہے۔ بعد میں دوبارہ کوشش کریں۔
Translation service unavailable. Please try again later.
`;
}

export const config = {
  api: {
    bodyParser: true,
    maxDuration: 30, // Allow longer execution time for translation
  },
};