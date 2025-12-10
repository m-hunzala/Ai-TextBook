"""
Content personalization endpoints and functionality
"""
import hashlib
import json
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional, Dict, Any
from database import db
from middleware import verify_better_auth_token
from datastore import datastore
import google.generativeai as genai
import os
from dotenv import load_dotenv

load_dotenv()

personalization_router = APIRouter(prefix="/personalization", tags=["personalization"])

class PersonalizeRequest(BaseModel):
    chapter_url: str
    original_content: str
    force_regenerate: bool = False

class PersonalizeResponse(BaseModel):
    status: str
    message: str
    personalized_content: Optional[str] = None
    cache_hit: bool = False

class ContentToggleResponse(BaseModel):
    original_content: str
    personalized_content: Optional[str] = None
    is_personalized: bool = False

async def get_user_profile(user_id: int) -> Dict[str, Any]:
    """Get user profile from database"""
    async with db.get_connection() as conn:
        result = await conn.fetchrow(
            "SELECT primary_os, gpu_model, experience_level, preferred_hardware FROM user_profiles WHERE user_id = $1",
            user_id
        )
    
    if not result:
        return {}
    
    return {
        'primary_os': result['primary_os'],
        'gpu_model': result['gpu_model'],
        'experience_level': result['experience_level'],
        'preferred_hardware': result['preferred_hardware']
    }

def generate_content_hash(content: str) -> str:
    """Generate hash for content to detect changes"""
    return hashlib.sha256(content.encode()).hexdigest()

async def get_cached_personalization(user_id: int, chapter_url: str, content_hash: str) -> Optional[str]:
    """Get cached personalized content if available and not stale"""
    async with db.get_connection() as conn:
        result = await conn.fetchrow("""
            SELECT personalized_content 
            FROM personalized_content 
            WHERE user_id = $1 AND chapter_url = $2
        """, user_id, chapter_url)
    
    if result and result['personalized_content']:
        return result['personalized_content']
    return None

async def cache_personalization(user_id: int, chapter_url: str, original_content: str, 
                               personalized_content: str, settings: Dict[str, Any]) -> None:
    """Cache the personalized content"""
    content_hash = generate_content_hash(original_content)
    
    async with db.get_connection() as conn:
        await conn.execute("""
            INSERT INTO personalized_content 
            (user_id, chapter_url, original_content_hash, personalized_content, personalization_settings)
            VALUES ($1, $2, $3, $4, $5)
            ON CONFLICT (user_id, chapter_url) 
            DO UPDATE SET
                original_content_hash = $3,
                personalized_content = $4,
                personalization_settings = $5,
                updated_at = CURRENT_TIMESTAMP
        """, user_id, chapter_url, content_hash, personalized_content, json.dumps(settings))

@personalization_router.post("/personalize", response_model=PersonalizeResponse)
async def personalize_content(
    request: PersonalizeRequest, 
    current_user: dict = Depends(verify_better_auth_token)
):
    """
    Personalize content based on user profile
    """
    user_id = current_user['user_id']
    user_profile = await get_user_profile(user_id)
    
    if not user_profile:
        raise HTTPException(status_code=404, detail="User profile not found. Please complete your profile.")
    
    # Check for cached version unless force_regenerate is True
    if not request.force_regenerate:
        cached_content = await get_cached_personalization(user_id, request.chapter_url, generate_content_hash(request.original_content))
        if cached_content:
            return PersonalizeResponse(
                status="success",
                message="Retrieved from cache",
                personalized_content=cached_content,
                cache_hit=True
            )
    
    try:
        # Generate personalized content using LLM
        personalized_content = await generate_personalized_content(
            request.original_content, 
            user_profile
        )
        
        # Cache the result
        await cache_personalization(
            user_id, 
            request.chapter_url, 
            request.original_content, 
            personalized_content, 
            user_profile
        )
        
        return PersonalizeResponse(
            status="success",
            message="Content personalized successfully",
            personalized_content=personalized_content,
            cache_hit=False
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error personalizing content: {str(e)}")

async def generate_personalized_content(original_content: str, user_profile: Dict[str, Any]) -> str:
    """
    Use LLM to personalize content based on user profile
    """
    # Prepare personalization instructions based on user profile
    instructions = []
    
    # Adjust content based on experience level
    experience_level = user_profile.get('experience_level', 'Beginner')
    if experience_level == 'Beginner':
        instructions.append("Simplify explanations and add more examples")
        instructions.append("Include step-by-step instructions")
        instructions.append("Add definitions for technical terms")
    elif experience_level == 'Intermediate':
        instructions.append("Provide moderate level of detail")
        instructions.append("Include some advanced concepts")
    elif experience_level == 'Advanced':
        instructions.append("Provide detailed and technical explanations")
        instructions.append("Include advanced concepts and optimizations")
        instructions.append("Add optional exercises and challenges")
    
    # Add hardware-specific instructions
    preferred_hardware = user_profile.get('preferred_hardware', '')
    if preferred_hardware:
        instructions.append(f"Include examples specific to {preferred_hardware} hardware")
    
    primary_os = user_profile.get('primary_os', '')
    if primary_os:
        instructions.append(f"Provide command examples for {primary_os}")
    
    gpu_model = user_profile.get('gpu_model', '')
    if gpu_model and gpu_model != "None":
        instructions.append(f"Include optimization tips for {gpu_model}")
    
    # Create system prompt for the LLM
    system_prompt = f"""
    You are a content personalization assistant. Your goal is to adapt technical documentation content 
    for a specific user based on their profile.
    
    User preferences:
    - Experience Level: {experience_level}
    - Preferred Hardware: {preferred_hardware or 'Not specified'}
    - Primary OS: {primary_os or 'Not specified'}
    - GPU Model: {gpu_model or 'Not specified'}
    
    Apply these instructions: {', '.join(instructions)}
    
    Original content:
    {original_content}
    
    Return the personalized content that follows these preferences while maintaining technical accuracy.
    """
    
    # Use Gemini API to generate personalized content
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
                "temperature": 0.3,  # Lower temperature for more consistent output
                "max_output_tokens": 4096
            }
        )
        
        if response.text:
            return response.text
        else:
            # If LLM fails, return original content
            return original_content
    except Exception as e:
        print(f"Error calling Gemini API for personalization: {e}")
        # Return original content if LLM fails
        return original_content

@personalization_router.get("/content/{chapter_url}", response_model=ContentToggleResponse)
async def get_content_with_personalization(
    chapter_url: str,
    include_personalized: bool = True,
    current_user: dict = Depends(verify_better_auth_token)
):
    """
    Get original content and optionally personalized version
    """
    user_id = current_user['user_id']
    
    # For this endpoint, we need to get both original and personalized content
    # In a real implementation, you'd fetch the original content from your content source
    # For now, we'll return what's available
    
    async with db.get_connection() as conn:
        result = await conn.fetchrow("""
            SELECT personalized_content 
            FROM personalized_content 
            WHERE user_id = $1 AND chapter_url = $2
        """, user_id, chapter_url)
    
    personalized_content = result['personalized_content'] if result else None
    
    # In a real implementation, you'd fetch the original content from your documentation system
    # For now, we'll return a placeholder
    original_content = f"Original content for chapter: {chapter_url}"
    
    return ContentToggleResponse(
        original_content=original_content,
        personalized_content=personalized_content,
        is_personalized=bool(personalized_content)
    )

@personalization_router.delete("/content/{chapter_url}")
async def clear_personalized_content(
    chapter_url: str,
    current_user: dict = Depends(verify_better_auth_token)
):
    """
    Clear cached personalized content for a chapter
    """
    user_id = current_user['user_id']
    
    async with db.get_connection() as conn:
        await conn.execute("""
            DELETE FROM personalized_content 
            WHERE user_id = $1 AND chapter_url = $2
        """, user_id, chapter_url)
    
    return {"status": "success", "message": "Personalized content cleared"}