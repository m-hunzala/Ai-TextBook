import openai
from typing import List
from app.config import settings
import logging

logger = logging.getLogger(__name__)

# Set OpenAI API key
openai.api_key = settings.OPENAI_API_KEY

async def get_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Get embeddings for a list of texts using OpenAI
    """
    try:
        # Use the EMBED_MODEL from environment if available, otherwise fallback to OPENAI_EMBEDDING_MODEL
        model = getattr(settings, 'EMBED_MODEL', settings.OPENAI_EMBEDDING_MODEL)
        response = await openai.Embedding.acreate(
            input=texts,
            model=model
        )

        embeddings = [item.embedding for item in response.data]
        return embeddings
    except Exception as e:
        logger.error(f"Error getting embeddings: {str(e)}")
        raise Exception(f"Error getting embeddings: {str(e)}")

async def get_chat_completion(messages: List[dict], model: str = None) -> str:
    """
    Get chat completion from OpenAI
    """
    if model is None:
        model = settings.OPENAI_CHAT_MODEL

    try:
        response = await openai.ChatCompletion.acreate(
            model=model,
            messages=messages,
            temperature=0.7,
            max_tokens=1000
        )

        return response.choices[0].message.content
    except Exception as e:
        logger.error(f"Error getting chat completion: {str(e)}")
        raise Exception(f"Error getting chat completion: {str(e)}")

async def get_chat_completion_with_functions(messages: List[dict], functions: List[dict] = None, model: str = None) -> dict:
    """
    Get chat completion from OpenAI with function calling capability (for potential ChatKit integration)
    """
    if model is None:
        model = settings.OPENAI_CHAT_MODEL

    try:
        params = {
            "model": model,
            "messages": messages,
            "temperature": 0.7,
            "max_tokens": 1000
        }

        if functions:
            params["functions"] = functions
            params["function_call"] = "auto"

        response = await openai.ChatCompletion.acreate(**params)

        return {
            "content": response.choices[0].message.content,
            "function_call": response.choices[0].message.function_call
        }
    except Exception as e:
        logger.error(f"Error getting chat completion with functions: {str(e)}")
        raise Exception(f"Error getting chat completion with functions: {str(e)}")