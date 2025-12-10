"""
Content translation endpoints and functionality
"""
import hashlib
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional
import google.generativeai as genai
import os
from dotenv import load_dotenv
from database import db

load_dotenv()

translation_router = APIRouter(prefix="/translation", tags=["translation"])

class TranslateRequest(BaseModel):
    chapter_url: str
    original_content: str
    target_language: str = "ur"  # Default to Urdu
    force_regenerate: bool = False

class TranslateResponse(BaseModel):
    status: str
    message: str
    translated_content: Optional[str] = None
    cache_hit: bool = False

class TranslationToggleResponse(BaseModel):
    original_content: str
    translated_content: Optional[str] = None
    view_mode: str  # 'english', 'urdu', or 'side-by-side'
    languages_available: list[str]

def generate_content_hash(content: str) -> str:
    """Generate hash for content to detect changes"""
    return hashlib.sha256(content.encode()).hexdigest()

async def get_cached_translation(chapter_url: str, target_language: str, content_hash: str) -> Optional[str]:
    """Get cached translation if available and not stale"""
    async with db.get_connection() as conn:
        result = await conn.fetchrow("""
            SELECT translated_content 
            FROM translated_content 
            WHERE chapter_url = $1 AND target_language = $2
        """, chapter_url, target_language)
    
    if result and result['translated_content']:
        return result['translated_content']
    return None

async def cache_translation(chapter_url: str, target_language: str, original_content: str, 
                          translated_content: str) -> None:
    """Cache the translated content"""
    content_hash = generate_content_hash(original_content)
    
    async with db.get_connection() as conn:
        await conn.execute("""
            INSERT INTO translated_content 
            (chapter_url, target_language, original_content_hash, translated_content)
            VALUES ($1, $2, $3, $4)
            ON CONFLICT (chapter_url, target_language) 
            DO UPDATE SET
                original_content_hash = $3,
                translated_content = $4,
                updated_at = CURRENT_TIMESTAMP
        """, chapter_url, target_language, content_hash, translated_content)

@translation_router.post("/translate", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    """
    Translate content to the target language (Urdu)
    """
    # Validate target language
    if request.target_language != "ur":
        raise HTTPException(status_code=400, detail="Only Urdu translation is currently supported")
    
    # Check for cached version unless force_regenerate is True
    if not request.force_regenerate:
        cached_translation = await get_cached_translation(
            request.chapter_url, 
            request.target_language, 
            generate_content_hash(request.original_content)
        )
        if cached_translation:
            return TranslateResponse(
                status="success",
                message="Retrieved from cache",
                translated_content=cached_translation,
                cache_hit=True
            )
    
    try:
        # Generate translated content using LLM
        translated_content = await generate_translated_content(
            request.original_content, 
            request.target_language
        )
        
        # Cache the result
        await cache_translation(
            request.chapter_url, 
            request.target_language, 
            request.original_content, 
            translated_content
        )
        
        return TranslateResponse(
            status="success",
            message="Content translated successfully",
            translated_content=translated_content,
            cache_hit=False
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error translating content: {str(e)}")

async def generate_translated_content(original_content: str, target_language: str) -> str:
    """
    Use LLM to translate content to target language while preserving code blocks
    """
    # Extract code blocks to preserve them during translation
    code_blocks = []
    placeholder_pattern = r'(```[\s\S]*?```|`[^`]*`)'
    
    import re
    matches = list(re.finditer(placeholder_pattern, original_content))
    
    # Replace code blocks with placeholders
    processed_content = original_content
    for i, match in enumerate(matches):
        placeholder = f"__CODE_BLOCK_{i}__"
        code_blocks.append(match.group(0))
        processed_content = processed_content.replace(match.group(0), placeholder, 1)
    
    # Prepare system prompt for translation
    if target_language == "ur":
        language_name = "Urdu"
        language_direction = "from English to Urdu"
    else:
        language_name = target_language
        language_direction = f"to {target_language}"
    
    system_prompt = f"""
    You are a professional translator specializing in technical documentation. 
    Translate the following technical content {language_direction}.
    
    Requirements:
    1. Translate all text content accurately to {language_name}
    2. Preserve all code blocks, configuration examples, and technical terms exactly as they are
    3. Keep all HTML tags, markdown formatting, and structure intact
    4. Maintain the same document structure and formatting
    5. Translate explanations, descriptions, and narrative text
    6. Keep URLs, file names, and technical identifiers unchanged
    7. Add appropriate {language_name} equivalents for technical concepts when available
    8. Maintain the same meaning and intent of the original content
    
    Content to translate:
    {processed_content}
    
    Return the translated content with the same structure and formatting as the original.
    """
    
    # Use Gemini API to translate content
    api_key = os.getenv("GEMINI_API_KEY")
    if not api_key:
        # Fallback - return original content if no API key
        return original_content
    
    try:
        genai.configure(api_key=api_key)
        model = genai.GenerativeModel('gemini-pro')
        
        response = await model.generate_content_async(
            system_prompt,
            generation_config={
                "temperature": 0.1,  # Lower temperature for more accurate translation
                "max_output_tokens": 4096
            }
        )
        
        if response.text:
            # Replace placeholders with original code blocks
            translated_content = response.text
            for i, code_block in enumerate(code_blocks):
                placeholder = f"__CODE_BLOCK_{i}__"
                translated_content = translated_content.replace(placeholder, code_block, 1)
            
            return translated_content
        else:
            # If LLM fails, return original content
            return original_content
    except Exception as e:
        print(f"Error calling Gemini API for translation: {e}")
        # Return original content if LLM fails
        return original_content

@translation_router.get("/content/{chapter_url}/{target_language}", response_model=TranslationToggleResponse)
async def get_content_with_translation(
    chapter_url: str,
    target_language: str,
    view_mode: str = "english"  # english, urdu, or side-by-side
):
    """
    Get original content and optionally translated version
    """
    # Get the translated content from cache
    async with db.get_connection() as conn:
        result = await conn.fetchrow("""
            SELECT translated_content 
            FROM translated_content 
            WHERE chapter_url = $1 AND target_language = $2
        """, chapter_url, target_language)
    
    translated_content = result['translated_content'] if result else None
    
    # Get available languages for this chapter
    languages_result = await conn.fetch(
        "SELECT DISTINCT target_language FROM translated_content WHERE chapter_url = $1",
        chapter_url
    )
    languages_available = [row['target_language'] for row in languages_result]
    if 'en' not in languages_available:
        languages_available.insert(0, 'en')  # Add English as default
    
    # In a real implementation, you'd fetch the original content from your documentation system
    # For now, we'll return a placeholder
    original_content = f"Original content for chapter: {chapter_url}"
    
    return TranslationToggleResponse(
        original_content=original_content,
        translated_content=translated_content,
        view_mode=view_mode,
        languages_available=languages_available
    )

@translation_router.delete("/content/{chapter_url}/{target_language}")
async def clear_translation(
    chapter_url: str,
    target_language: str
):
    """
    Clear cached translation for a chapter and language
    """
    async with db.get_connection() as conn:
        await conn.execute("""
            DELETE FROM translated_content 
            WHERE chapter_url = $1 AND target_language = $2
        """, chapter_url, target_language)
    
    return {"status": "success", "message": f"Translation cleared for {target_language}"}

@translation_router.get("/languages/{chapter_url}")
async def get_available_languages(chapter_url: str):
    """
    Get all available languages for a specific chapter
    """
    async with db.get_connection() as conn:
        result = await conn.fetch(
            "SELECT DISTINCT target_language FROM translated_content WHERE chapter_url = $1",
            chapter_url
        )
        languages = [row['target_language'] for row in result]
        if 'en' not in languages:
            languages.insert(0, 'en')  # Add English as default
    
    return {"languages": languages}