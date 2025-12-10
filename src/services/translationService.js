import { GoogleGenerativeAI } from '@google/generative-ai';

class TranslationService {
  constructor() {
    this.apiKey = process.env.GOOGLE_GEMINI_API_KEY || process.env.REACT_APP_GOOGLE_GEMINI_API_KEY;
    this.genAI = this.apiKey ? new GoogleGenerativeAI(this.apiKey) : null;
    this.model = this.genAI?.getGenerativeModel({ model: 'gemini-pro' });
  }

  /**
   * Translates content from English to Urdu while preserving markdown formatting
   * @param {string} content - The markdown/MDX content to translate
   * @returns {Promise<string>} - The translated content
   */
  async translateToUrdu(content) {
    if (!this.model) {
      throw new Error('Translation service not initialized: API key is missing');
    }

    const prompt = `
      Translate the following English content to Urdu. Preserve all markdown formatting including:
      - Headings (# ## ###)
      - Bold and italic text (** **, * *)
      - Lists (-, *, 1., 2.)
      - Code blocks (\`\`\`)
      - Inline code (\`)
      - Links ([text](url))
      - Images (![alt](url))
      - Tables (| column | column |)
      - Blockquotes (>)
      - Horizontal rules (---)
      - Any special Docusaurus/admonition syntax
      
      Content to translate:
      ${content}
    `;

    try {
      const result = await this.model.generateContent(prompt);
      const response = await result.response;
      return response.text();
    } catch (error) {
      console.error('Translation error:', error);
      throw new Error(`Translation failed: ${error.message}`);
    }
  }

  /**
   * Alternative method using OpenAI if Google Gemini is not available
   */
  async translateToUrduOpenAI(content) {
    // Placeholder implementation - would need OpenAI API integration
    // This is for cases where Google Gemini isn't available
    throw new Error('OpenAI translation not implemented yet');
  }

  /**
   * Extracts content elements while preserving structure for translation
   * @param {string} content - Raw markdown content
   * @returns {Array} - Array of content blocks to translate
   */
  extractContentBlocks(content) {
    // Regular expressions to identify different markdown elements
    const patterns = [
      { type: 'code_block', regex: /```[\s\S]*?```/g },
      { type: 'inline_code', regex: /`[^`]+`/g },
      { type: 'heading', regex: /^#+\s+.*/gm },
      { type: 'link', regex: /\[([^\]]+)\]\([^)]+\)/g },
      { type: 'image', regex: /!\[([^\]]*)\]\([^)]+\)/g },
      { type: 'list_item', regex: /^[*-]\s+.*/gm },
      { type: 'ordered_list_item', regex: /^\d+\.\s+.*/gm },
      { type: 'blockquote', regex: /^>\s+.*/gm },
      { type: 'bold_italic', regex: /\*{2,3}[^*]+\*{2,3}/g },
      { type: 'text', regex: /.+/g }
    ];

    // Separate content into translatable and non-translatable blocks
    const blocks = [];
    let currentIndex = 0;
    
    // First pass: identify all fixed format elements (code, etc.)
    const codeMatches = [...content.matchAll(/```[\s\S]*?```/g)];
    const inlineCodeMatches = [...content.matchAll(/`[^`]+`/g)];
    
    // For simplicity in this implementation, we'll translate all text while preserving code blocks
    return this.preserveCodeBlocks(content);
  }

  /**
   * Preserves code blocks during translation by temporarily replacing them
   * @param {string} content - Original content
   * @returns {Object} - Object with processed content and placeholders
   */
  preserveCodeBlocks(content) {
    const placeholders = [];
    let processedContent = content;
    
    // Replace code blocks with placeholders
    processedContent = processedContent.replace(/```([\s\S]*?)```/g, (match, code) => {
      const index = placeholders.length;
      placeholders.push({ type: 'code_block', content: `\`\`\`${code}\`\`\``, index });
      return `__CODE_BLOCK_${index}__`;
    });

    // Replace inline code with placeholders
    processedContent = processedContent.replace(/`([^`]+)`/g, (match, code) => {
      const index = placeholders.length;
      placeholders.push({ type: 'inline_code', content: `\`${code}\``, index });
      return `__INLINE_CODE_${index}__`;
    });

    return {
      content: processedContent,
      placeholders
    };
  }

  /**
   * Restores code blocks after translation
   * @param {string} translatedContent - Translated content with placeholders
   * @param {Array} placeholders - Original code blocks
   * @returns {string} - Content with code blocks restored
   */
  restoreCodeBlocks(translatedContent, placeholders) {
    let result = translatedContent;

    // Restore code blocks in reverse order to maintain indices
    for (let i = placeholders.length - 1; i >= 0; i--) {
      const placeholder = placeholders[i];
      if (placeholder.type === 'code_block') {
        result = result.replace(`__CODE_BLOCK_${i}__`, placeholder.content);
      } else if (placeholder.type === 'inline_code') {
        result = result.replace(`__INLINE_CODE_${i}__`, placeholder.content);
      }
    }

    return result;
  }

  /**
   * Performs intelligent translation that preserves formatting
   * @param {string} content - Content to translate
   * @returns {Promise<string>} - Translated content
   */
  async translateWithFormattingPreservation(content) {
    const preserved = this.preserveCodeBlocks(content);
    const translatedContent = await this.translateToUrdu(preserved.content);
    return this.restoreCodeBlocks(translatedContent, preserved.placeholders);
  }
}

export default TranslationService;