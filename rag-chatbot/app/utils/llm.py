from typing import List
from app.utils.embeddings import get_chat_completion, get_chat_completion_with_functions

async def generate_response(query: str, context: str) -> str:
    """
    Generate a response to the user's query based on the provided context
    """
    messages = [
        {
            "role": "system",
            "content": "You are a helpful assistant that answers questions based on the provided book context. "
                       "Be concise and accurate. Only use information from the provided context to answer the question. "
                       "If the context doesn't contain enough information to answer, say so."
        },
        {
            "role": "user",
            "content": f"Context: {context}\n\nQuestion: {query}\n\nAnswer:"
        }
    ]

    try:
        response = await get_chat_completion(messages)
        return response
    except Exception as e:
        raise Exception(f"Error generating response: {str(e)}")

async def generate_response_with_selected_text(query: str, selected_text: str) -> str:
    """
    Generate a response using only the selected text as context
    """
    messages = [
        {
            "role": "system",
            "content": "You are a helpful assistant that answers questions based only on the provided selected text. "
                       "Be concise and accurate. Only use information from the selected text to answer the question. "
                       "If the selected text doesn't contain enough information to answer, say so."
        },
        {
            "role": "user",
            "content": f"Selected text: {selected_text}\n\nQuestion: {query}\n\nAnswer:"
        }
    ]

    try:
        response = await get_chat_completion(messages)
        return response
    except Exception as e:
        raise Exception(f"Error generating response: {str(e)}")

async def generate_structured_response(query: str, context: str, response_format: str = "text") -> dict:
    """
    Generate a structured response that could be used for more advanced processing
    """
    if response_format == "json":
        messages = [
            {
                "role": "system",
                "content": "You are a helpful assistant that answers questions based on the provided book context. "
                           "Respond in JSON format with 'answer' and 'confidence' fields. "
                           "Confidence should be a number between 0 and 1 indicating how confident you are in your answer."
            },
            {
                "role": "user",
                "content": f"Context: {context}\n\nQuestion: {query}\n\nRespond in JSON format:"
            }
        ]

        try:
            response = await get_chat_completion(messages)
            # In a real implementation, we'd parse the JSON response
            # For now, we'll return a simple structure
            return {
                "answer": response,
                "confidence": 0.8,  # Placeholder value
                "format": "json"
            }
        except Exception as e:
            raise Exception(f"Error generating structured response: {str(e)}")
    else:
        # Default to text response
        response = await generate_response(query, context)
        return {
            "answer": response,
            "confidence": None,
            "format": "text"
        }