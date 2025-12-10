import axios from 'axios';

interface TranslationConfig {
  provider: 'agent' | 'api';
  agentUrl?: string;
  apiKey?: string;
}

// Configuration for translation provider
const TRANSLATION_CONFIG: TranslationConfig = {
  provider: process.env.TRANSLATION_PROVIDER === 'api' ? 'api' : 'agent',
  agentUrl: process.env.AGENT_TRANSLATOR_URL || 'http://localhost:8003', // Default to our agent
  apiKey: process.env.TRANSLATION_API_KEY
};

/**
 * Translates markdown content while preserving code blocks and structure
 * @param markdown - Original markdown content to translate
 * @param targetLang - Target language code (e.g., 'ur' for Urdu)
 * @returns Translated markdown content
 */
export async function translateMarkdown(markdown: string, targetLang: string): Promise<string> {
  // Extract code blocks and other non-translatable elements
  const extractedElements = extractMarkdownElements(markdown);
  
  // Get the content without code blocks for translation
  let contentForTranslation = extractedElements.contentWithoutElements;
  
  // Translate the content
  const translatedContent = await callTranslationService(contentForTranslation, targetLang);
  
  // Restore the extracted elements back to the translated content
  const finalTranslatedMarkdown = restoreMarkdownElements(
    translatedContent, 
    extractedElements.originalElements
  );
  
  return finalTranslatedMarkdown;
}

/**
 * Extracts code blocks, headers, links, etc. from markdown to preserve during translation
 */
interface ExtractedElements {
  contentWithoutElements: string;
  originalElements: Array<{
    placeholder: string;
    original: string;
    type: string;
  }>;
}

function extractMarkdownElements(markdown: string): ExtractedElements {
  const originalElements: Array<{
    placeholder: string;
    original: string;
    type: string;
  }> = [];
  
  let content = markdown;
  let idCounter = 0;
  
  // Extract fenced code blocks (```code```)
  const fencedCodeBlocks = content.match(/```[\s\S]*?```/g) || [];
  fencedCodeBlocks.forEach(block => {
    const placeholder = `__FENCED_CODE_BLOCK_${idCounter++}__`;
    originalElements.push({
      placeholder,
      original: block,
      type: 'fenced_code_block'
    });
    content = content.replace(block, placeholder);
  });
  
  // Extract inline code blocks (`code`)
  const inlineCodeBlocks = content.match(/`[^`]*`/g) || [];
  inlineCodeBlocks.forEach(block => {
    // Skip if it's part of a fenced block that was already replaced
    if (!originalElements.some(el => el.original === block)) {
      const placeholder = `__INLINE_CODE_BLOCK_${idCounter++}__`;
      originalElements.push({
        placeholder,
        original: block,
        type: 'inline_code_block'
      });
      content = content.replace(block, placeholder);
    }
  });
  
  // Extract links [text](url)
  const links = content.match(/\[([^\]]+)\]\([^)]+\)/g) || [];
  links.forEach(link => {
    const placeholder = `__LINK_${idCounter++}__`;
    originalElements.push({
      placeholder,
      original: link,
      type: 'link'
    });
    content = content.replace(link, placeholder);
  });
  
  // Extract images ![alt](url)
  const images = content.match(/!\[([^\]]*)\]\([^)]+\)/g) || [];
  images.forEach(img => {
    const placeholder = `__IMAGE_${idCounter++}__`;
    originalElements.push({
      placeholder,
      original: img,
      type: 'image'
    });
    content = content.replace(img, placeholder);
  });
  
  // Extract headers to maintain structure (keep header markers, translate content)
  const headers = content.match(/^(#{1,6})\s+(.*?)$/gm) || [];
  headers.forEach(header => {
    // Extract header content and preserve the # markers
    const headerMatch = header.match(/^(#{1,6})\s+(.*?)$/);
    if (headerMatch) {
      const fullHeader = headerMatch[0];
      const level = headerMatch[1]; // e.g., "###"
      const headerText = headerMatch[2]; // e.g., "My Header"
      
      const placeholder = `__HEADER_${idCounter++}__`;
      originalElements.push({
        placeholder: `${level} ${placeholder}`,
        original: fullHeader,
        type: 'header'
      });
      
      // Replace the header in content with just the placeholder
      content = content.replace(fullHeader, `${level} ${placeholder}`);
    }
  });
  
  // Extract bold text (**text** or __text__)
  const boldText = content.match(/(\*\*|__)(.*?)\1/g) || [];
  boldText.forEach(bold => {
    const placeholder = `__BOLD_${idCounter++}__`;
    originalElements.push({
      placeholder,
      original: bold,
      type: 'bold'
    });
    content = content.replace(bold, placeholder);
  });
  
  // Extract italic text (*text* or _text_)
  const italicText = content.match(/(^|[^*])((\*|_)([^*_\s][^*_]*?|[^*_])\3)(?!\w)/gm) || [];
  italicText.forEach(italic => {
    const placeholder = `__ITALIC_${idCounter++}__`;
    originalElements.push({
      placeholder,
      original: italic.trim(),
      type: 'italic'
    });
    content = content.replace(italic, placeholder);
  });
  
  return {
    contentWithoutElements: content,
    originalElements
  };
}

/**
 * Restores extracted elements back to the translated content
 */
function restoreMarkdownElements(
  translatedContent: string, 
  originalElements: Array<{
    placeholder: string;
    original: string;
    type: string;
  }>
): string {
  let result = translatedContent;
  
  // Restore in reverse order to handle nested elements correctly
  for (const element of [...originalElements].reverse()) {
    result = result.replace(element.placeholder, element.original);
  }
  
  return result;
}

/**
 * Calls the appropriate translation service (agent or external API)
 */
async function callTranslationService(text: string, targetLang: string): Promise<string> {
  if (TRANSLATION_CONFIG.provider === 'agent') {
    // Call our TranslatorAgent
    return await callAgentTranslator(text, targetLang);
  } else {
    // Call external translation API
    return await callExternalApi(text, targetLang);
  }
}

/**
 * Calls the TranslatorAgent we created earlier
 */
async function callAgentTranslator(text: string, targetLang: string): Promise<string> {
  try {
    const response = await axios.post(`${TRANSLATION_CONFIG.agentUrl}/translate`, {
      text,
      target_lang: targetLang
    }, {
      headers: {
        'Content-Type': 'application/json'
      },
      timeout: 30000 // 30 second timeout
    });
    
    return response.data.translated_text;
  } catch (error) {
    console.error("Error calling TranslatorAgent:", error);
    throw new Error(`TranslatorAgent error: ${error.message}`);
  }
}

/**
 * Calls an external translation API (e.g., Google Translate, DeepL)
 */
async function callExternalApi(text: string, targetLang: string): Promise<string> {
  if (!TRANSLATION_CONFIG.apiKey) {
    throw new Error("Translation API key not configured");
  }
  
  // Example using Google Translate API (you would need to install google-translate-api or similar)
  // This is just an example implementation:
  try {
    // In a real implementation, you'd use a service like:
    // const { Translate } = require('@google-cloud/translate').v2;
    // const translate = new Translate({key: TRANSLATION_CONFIG.apiKey});
    // const [translation] = await translate.translate(text, targetLang);
    
    // For now, we'll throw an error to indicate this needs implementation
    throw new Error("External API translation not implemented");
  } catch (error) {
    console.error("Error calling external translation API:", error);
    throw new Error(`External Translation API error: ${error.message}`);
  }
}