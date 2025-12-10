"""
Authentication middleware for Better-Auth integration
"""
from fastapi import HTTPException, Request
from fastapi.security import HTTPBearer
import jwt
from datetime import datetime
import os
from dotenv import load_dotenv

load_dotenv()

# Better-Auth secret (in a real app, this would be configurable)
BETTER_AUTH_SECRET = os.getenv("BETTER_AUTH_SECRET", "dev-secret-key-change-in-production")

security = HTTPBearer()

async def verify_better_auth_token(request: Request):
    """
    Verify Better-Auth token from request header
    """
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="No authorization token provided")
    
    token = auth_header.split(" ")[1]
    
    try:
        # Decode the token using Better-Auth secret
        # Note: In a real Better-Auth integration, you'd validate using Better-Auth's own validation
        # For now, we'll simulate the validation
        payload = jwt.decode(token, BETTER_AUTH_SECRET, algorithms=["HS256"])
        
        # Check if token is expired
        if 'exp' in payload and datetime.fromtimestamp(payload['exp']) < datetime.utcnow():
            raise HTTPException(status_code=401, detail="Token has expired")
        
        # Return user info from token
        return {
            'user_id': payload.get('userId'),
            'email': payload.get('email'),
            'name': payload.get('name')
        }
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token has expired")
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="Invalid token")

def get_user_id_from_token(request: Request) -> str:
    """
    Extract user ID from Better-Auth token
    """
    try:
        auth_header = request.headers.get("Authorization")
        if not auth_header or not auth_header.startswith("Bearer "):
            return None
        
        token = auth_header.split(" ")[1]
        payload = jwt.decode(token, BETTER_AUTH_SECRET, algorithms=["HS256"])
        return payload.get('userId')
    except:
        return None