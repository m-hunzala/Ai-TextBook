"""
Authentication endpoints for Better-Auth integration
"""
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional
from database import db
from datastore import datastore
from middleware import verify_better_auth_token, get_user_id_from_token
from fastapi import Request

auth_router = APIRouter(prefix="/auth", tags=["auth"])

class UserProfile(BaseModel):
    primary_os: Optional[str] = None
    gpu_model: Optional[str] = None
    experience_level: Optional[str] = None
    preferred_hardware: Optional[str] = None

class UserProfileRequest(BaseModel):
    profile: UserProfile

class UserProfileResponse(BaseModel):
    status: str
    message: str

@auth_router.post("/user-profile", response_model=UserProfileResponse)
async def create_user_profile(request: UserProfileRequest, current_user: dict = Depends(verify_better_auth_token)):
    """
    Create or update user profile with additional information
    """
    try:
        user_id = current_user['user_id']
        async with db.get_connection() as conn:
            # Insert or update profile
            await conn.execute("""
                INSERT INTO user_profiles
                (user_id, primary_os, gpu_model, experience_level, preferred_hardware)
                VALUES ($1, $2, $3, $4, $5)
                ON CONFLICT (user_id)
                DO UPDATE SET
                    primary_os = EXCLUDED.primary_os,
                    gpu_model = EXCLUDED.gpu_model,
                    experience_level = EXCLUDED.experience_level,
                    preferred_hardware = EXCLUDED.preferred_hardware,
                    updated_at = CURRENT_TIMESTAMP
            """,
                user_id,
                request.profile.primary_os,
                request.profile.gpu_model,
                request.profile.experience_level,
                request.profile.preferred_hardware
            )

        return UserProfileResponse(
            status="success",
            message="User profile updated successfully"
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating user profile: {str(e)}")

@auth_router.get("/user-profile", response_model=UserProfile)
async def get_user_profile(current_user: dict = Depends(verify_better_auth_token)):
    """
    Get current user's profile information
    """
    try:
        user_id = current_user['user_id']
        async with db.get_connection() as conn:
            result = await conn.fetchrow(
                "SELECT primary_os, gpu_model, experience_level, preferred_hardware FROM user_profiles WHERE user_id = $1",
                user_id
            )

        if not result:
            # Return empty profile if not found
            return UserProfile()

        return UserProfile(
            primary_os=result['primary_os'],
            gpu_model=result['gpu_model'],
            experience_level=result['experience_level'],
            preferred_hardware=result['preferred_hardware']
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving user profile: {str(e)}")

@auth_router.get("/user-info")
async def get_user_info(current_user: dict = Depends(verify_better_auth_token)):
    """
    Get current user's basic information
    """
    return {
        "user_id": current_user['user_id'],
        "email": current_user.get('email'),
        "name": current_user.get('name')
    }