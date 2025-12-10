from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import re
import os
from typing import Optional
import google.generativeai as genai
from concurrent.futures import ThreadPoolExecutor
import asyncio

app = FastAPI(
    title="TranslatorAgent",
    description="A text translation service that preserves markdown formatting while translating to target languages.",
    version="1.0.0"
)

class TranslateRequest(BaseModel):
    text: str
    target_lang: str
    source_lang: Optional[str] = "en"


class TranslateResponse(BaseModel):
    translated_text: str


# Initialize Google Generative AI
GOOGLE_API_KEY = os.getenv("GOOGLE_GEMINI_API_KEY")
if GOOGLE_API_KEY:
    genai.configure(api_key=GOOGLE_API_KEY)
    model = genai.GenerativeModel('gemini-pro')
else:
    model = None
    print("Warning: GOOGLE_GEMINI_API_KEY not found. Using fallback translation.")

async def translate_with_gemini(text: str, target_lang: str = "ur") -> str:
    """
    Translate text using Google Gemini AI model with proper markdown preservation
    """
    if not model:
        raise Exception("Gemini model not initialized. Please set GOOGLE_GEMINI_API_KEY.")

    # More detailed prompt for markdown preservation
    prompt = f"""
    Translate the following markdown content to {target_lang}.
    CRITICAL: You must preserve ALL markdown formatting exactly as in the original:
    - Headers (# ## ###)
    - Bold text (**text**)
    - Italic text (*text* or _text_)
    - Code blocks (```...```)
    - Inline code (`code`)
    - Lists (- item, * item, 1. item)
    - Blockquotes (> text)
    - Links ([text](url))
    - Images (![alt](url))
    - Horizontal rules (---)
    - Any other markdown elements

    Only translate the actual text content, NOT the markdown syntax, URLs, code content, or technical terms.

    Content to translate:
    {text}
    """

    try:
        loop = asyncio.get_event_loop()
        response = await loop.run_in_executor(None, lambda: model.generate_content(prompt))
        return response.text.strip()
    except Exception as e:
        print(f"Gemini API error: {e}")
        raise e

# Common markdown patterns to preserve
MARKDOWN_PATTERNS = [
    (r'\*\*(.*?)\*\*', r'**\1**'),  # Bold
    (r'\*(.*?)\*', r'*\1*'),        # Italic
    (r'`(.*?)`', r'`\1`'),          # Inline code
    (r'^# (.*?)$', r'# \1'),        # H1
    (r'^## (.*?)$', r'## \1'),      # H2
    (r'^### (.*?)$', r'### \1'),    # H3
    (r'^- (.*?)$', r'- \1'),        # List items
    (r'^1\. (.*?)$', r'1. \1'),    # Numbered list items
    (r'\[(.*?)\]\((.*?)\)', r'[\1](\2)'),  # Links
    (r'!\[(.*?)\]\((.*?)\)', r'![\1](\2)') # Images
]


def advanced_markdown_translate(text: str, target_lang: str) -> str:
    """
    Advanced translation function that preserves markdown formatting.
    This version properly handles nested markdown elements and preserves structure.
    """
    if target_lang == "en":
        return text  # No translation needed

    # First, extract and temporarily store all code blocks and special elements
    placeholders = []
    temp_content = text

    # Extract code blocks (```...```)
    import re
    code_blocks = []
    def replace_code_block(match):
        code_content = match.group(0)
        idx = len(placeholders)
        placeholders.append(('code_block', code_content, idx))
        return f"[CODE_BLOCK_PLACEHOLDER_{idx}]"

    temp_content = re.sub(r'```[\s\S]*?```', replace_code_block, temp_content)

    # Extract inline code (`...`)
    def replace_inline_code(match):
        code_content = match.group(0)
        idx = len(placeholders)
        placeholders.append(('inline_code', code_content, idx))
        return f"[INLINE_CODE_PLACEHOLDER_{idx}]"

    temp_content = re.sub(r'`[^`]*`', replace_inline_code, temp_content)

    # Extract links [text](url)
    def replace_link(match):
        link_content = match.group(0)
        idx = len(placeholders)
        placeholders.append(('link', link_content, idx))
        return f"[LINK_PLACEHOLDER_{idx}]"

    temp_content = re.sub(r'\[([^\]]+)\]\([^)]+\)', replace_link, temp_content)

    # Extract images ![alt](url)
    def replace_image(match):
        img_content = match.group(0)
        idx = len(placeholders)
        placeholders.append(('image', img_content, idx))
        return f"[IMAGE_PLACEHOLDER_{idx}]"

    temp_content = re.sub(r'!\[([^\]]*)\]\([^)]+\)', replace_image, temp_content)

    # Now translate the remaining text content
    translated_content = translate_text_content(temp_content, target_lang)

    # Restore the placeholders
    for placeholder_type, original_content, idx in placeholders:
        if placeholder_type == 'code_block':
            translated_content = translated_content.replace(f"[CODE_BLOCK_PLACEHOLDER_{idx}]", original_content)
        elif placeholder_type == 'inline_code':
            translated_content = translated_content.replace(f"[INLINE_CODE_PLACEHOLDER_{idx}]", original_content)
        elif placeholder_type == 'link':
            translated_content = translated_content.replace(f"[LINK_PLACEHOLDER_{idx}]", original_content)
        elif placeholder_type == 'image':
            translated_content = translated_content.replace(f"[IMAGE_PLACEHOLDER_{idx}]", original_content)

    return translated_content


def translate_text_content(text: str, target_lang: str) -> str:
    """
    Translate the actual text content while handling markdown elements properly
    """
    import re

    # Split text into markdown elements and plain text
    # This regex identifies markdown elements while preserving them
    parts = []
    last_end = 0

    # Pattern to identify markdown elements
    markdown_pattern = re.compile(r'(\*\*.*?\*\*|_.*?_|`.*?`|!\[.*?\]\(.*?\)|\[.*?\]\(.*?\)|^#{1,6}.*$|^>.*$|^[-*+]\s.*$|^\d+\.\s.*$)', re.MULTILINE)

    for match in markdown_pattern.finditer(text):
        # Add text before the match
        if match.start() > last_end:
            before_text = text[last_end:match.start()]
            if before_text.strip():
                parts.append(('text', before_text))

        # Add the markdown element
        markdown_element = match.group(1)
        if markdown_element.startswith('**') and markdown_element.endswith('**'):  # Bold
            content = markdown_element[2:-2]
            translated_content = translate_plain_text(content, target_lang)
            parts.append(('markdown', f"**{translated_content}**"))
        elif markdown_element.startswith('*') and markdown_element.endswith('*') and not markdown_element.startswith('**'):  # Italic
            content = markdown_element[1:-1]
            translated_content = translate_plain_text(content, target_lang)
            parts.append(('markdown', f"*{translated_content}*"))
        elif markdown_element.startswith('`') and markdown_element.endswith('`'):  # Code (shouldn't happen here since we extracted code first)
            parts.append(('markdown', markdown_element))
        elif markdown_element.startswith('#'):  # Headers
            # Extract header level and content
            header_match = re.match(r'^(#{1,6})\s+(.*)', markdown_element)
            if header_match:
                level = header_match.group(1)
                content = header_match.group(2)
                translated_content = translate_plain_text(content, target_lang)
                parts.append(('markdown', f"{level} {translated_content}"))
        elif markdown_element.startswith('>'):  # Blockquotes
            content = markdown_element[1:].lstrip()
            translated_content = translate_plain_text(content, target_lang)
            parts.append(('markdown', f"> {translated_content}"))
        elif re.match(r'^[-*+]\s', markdown_element):  # Unordered lists
            content = re.sub(r'^[-*+]\s+', '', markdown_element)
            translated_content = translate_plain_text(content, target_lang)
            prefix = markdown_element[:2]  # Keep the list marker
            parts.append(('markdown', f"{prefix}{translated_content}"))
        elif re.match(r'^\d+\.\s', markdown_element):  # Ordered lists
            content = re.sub(r'^\d+\.\s+', '', markdown_element)
            translated_content = translate_plain_text(content, target_lang)
            # Find the original number prefix
            num_match = re.match(r'^(\d+\.\s)', markdown_element)
            prefix = num_match.group(1) if num_match else "1. "
            parts.append(('markdown', f"{prefix}{translated_content}"))
        else:  # Other markdown
            parts.append(('markdown', markdown_element))

        last_end = match.end()

    # Add remaining text after the last match
    if last_end < len(text):
        remaining_text = text[last_end:]
        if remaining_text.strip():
            parts.append(('text', remaining_text))

    # Process each part
    result_parts = []
    for part_type, content in parts:
        if part_type == 'text':
            result_parts.append(translate_plain_text(content, target_lang))
        else:  # markdown
            result_parts.append(content)

    return ''.join(result_parts)


def translate_plain_text(text: str, target_lang: str) -> str:
    """
    Translate plain text content using the simple translation dictionary
    """
    if target_lang == "en":
        return text

    # Split into sentences/paragraphs to better preserve context
    sentences = re.split(r'([.!?]+\s+|\n+)', text)
    translated_sentences = []

    for sentence in sentences:
        if re.match(r'^[.!?]+\s*$', sentence) or re.match(r'^\n+$', sentence):
            # Keep punctuation and newlines as-is
            translated_sentences.append(sentence)
        else:
            # Process actual text content
            words = re.split(r'(\s+|[^\w\s]+)', sentence)
            translated_words = []

            i = 0
            while i < len(words):
                word = words[i]
                if word.isspace() or re.match(r'^[^\w\s]+$', word):
                    # Keep whitespace and punctuation as-is
                    translated_words.append(word)
                elif word.lower() in SIMPLE_TRANSLATIONS and target_lang in SIMPLE_TRANSLATIONS[word.lower()]:
                    # Translate the word
                    translation = SIMPLE_TRANSLATIONS[word.lower()][target_lang]
                    # Preserve capitalization
                    if word[0].isupper():
                        translation = translation.capitalize()
                        if word.isupper():
                            translation = translation.upper()
                    translated_words.append(translation)
                elif word.strip().lower() in SIMPLE_TRANSLATIONS and target_lang in SIMPLE_TRANSLATIONS[word.strip().lower()]:
                    # Handle words with attached punctuation
                    clean_word = word.strip().lower()
                    punctuation = word[len(word.strip()):]  # Trailing punctuation
                    translation = SIMPLE_TRANSLATIONS[clean_word][target_lang]
                    # Preserve capitalization
                    if word.strip()[0].isupper():
                        translation = translation.capitalize()
                    translated_words.append(translation + punctuation)
                else:
                    # Keep original if no translation is available
                    translated_words.append(word)

                i += 1

            translated_sentences.append(''.join(translated_words))

    result = ''.join(translated_sentences)
    return result


async def simple_translate(text: str, target_lang: str) -> str:
    """
    Main translation function that preserves markdown formatting.
    Uses Gemini API when available, with fallback to rule-based translation.
    """
    if model is not None:
        # Use Gemini AI for better translation quality
        try:
            return await translate_with_gemini(text, target_lang)
        except Exception as e:
            print(f"Falling back to rule-based translation due to Gemini error: {e}")
            # Continue to fallback method
    else:
        print("Using rule-based translation as Gemini is not configured")

    # Fallback to rule-based translation
    return advanced_markdown_translate(text, target_lang)


@app.post("/translate", response_model=TranslateResponse)
async def translate_text(request: TranslateRequest):
    """
    Translate text to the target language while preserving markdown formatting.

    Args:
        request (TranslateRequest): Contains the text to translate and target language

    Returns:
        TranslateResponse: Contains the translated text
    """
    try:
        if not request.text or not request.target_lang:
            raise HTTPException(status_code=400, detail="Text and target language are required")

        translated_text = await simple_translate(request.text, request.target_lang.lower())

        return TranslateResponse(translated_text=translated_text)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing translation: {str(e)}")


@app.get("/")
async def root():
    return {"message": "TranslatorAgent is running", "version": "1.0.0"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8003)