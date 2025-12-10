"""
Wrapper for OpenAI Chat/ChatKit Agents returning structured evidence
"""
import json
import logging
from typing import List, Dict, Any, Optional
from app.config import settings
import openai

logger = logging.getLogger(__name__)

def generate_answer_with_evidence(context_chunks: List[Dict[str, Any]], question: str) -> Dict[str, Any]:
    """
    Generate answer with evidence using OpenAI Chat with structured response
    
    Args:
        context_chunks: List of chunks with chunk_id, text, etc.
        question: The question to answer
        
    Returns:
        Dict with answer, summary, and evidence containing chunk IDs and snippets
    """
    # Prepare the context with chunk IDs
    context_text = ""
    chunk_map = {}
    for chunk in context_chunks:
        chunk_id = chunk.get('chunk_id', 'unknown')
        text = chunk.get('text', '')
        context_text += f"\n[Chunk ID: {chunk_id}]\n{text}\n"
        chunk_map[chunk_id] = text

    # Create a structured prompt for the agent
    system_prompt = """You are a helpful assistant. ONLY use the text inside the CONTEXT markers to answer. If the answer is not present, reply exactly: "I don't know based on the provided text.""""
    
    user_prompt = f"""
    CONTEXT:
    {context_text}
    ENDCONTEXT
    
    Question: {question}
    
    Respond in JSON format with these keys: answer, summary, evidence
    The evidence should be an array of objects with chunk_id and text_snippet
    """
    
    try:
        # Set the API key
        openai.api_key = settings.OPENAI_API_KEY
        
        response = openai.ChatCompletion.create(
            model=getattr(settings, 'EMBED_MODEL', settings.OPENAI_CHAT_MODEL),
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.0,
            functions=[{
                "name": "extract_answer_with_evidence",
                "description": "Extract answer with evidence from document chunks",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "answer": {"type": "string", "description": "The answer to the question"},
                        "summary": {"type": "string", "description": "A concise summary of the answer"},
                        "evidence": {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "chunk_id": {"type": "string", "description": "The ID of the chunk used as evidence"},
                                    "text_snippet": {"type": "string", "description": "A snippet of text from the chunk"}
                                },
                                "required": ["chunk_id", "text_snippet"]
                            }
                        }
                    },
                    "required": ["answer", "summary", "evidence"]
                }
            }],
            function_call={"name": "extract_answer_with_evidence"}
        )
        
        # Extract function arguments
        function_args = response.choices[0].message.function_call.arguments
        result = json.loads(function_args)
        
        return result
        
    except json.JSONDecodeError:
        # If JSON parsing fails, return a structured response based on raw text
        try:
            response = openai.ChatCompletion.create(
                model=getattr(settings, 'EMBED_MODEL', settings.OPENAI_CHAT_MODEL),
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.0
            )
            
            # Return a basic structure with the raw answer and evidence
            answer = response.choices[0].message.content
            return {
                "answer": answer,
                "summary": f"Answer to: {question}",
                "evidence": [{"chunk_id": chunk.get('chunk_id', 'unknown'), 
                             "text_snippet": chunk.get('text', '')[:100] + "..." 
                             if len(chunk.get('text', '')) > 100 else chunk.get('text', '')}
                            for chunk in context_chunks[:3]]  # Limit to first 3 chunks
            }
        except Exception as e:
            logger.error(f"Fallback completion also failed: {e}")
            return {
                "answer": f"Could not generate answer due to an error: {str(e)}",
                "summary": "Error occurred while processing",
                "evidence": []
            }
    except Exception as e:
        logger.error(f"Error with completion: {e}")
        return {
            "answer": f"Could not generate answer due to an error: {str(e)}",
            "summary": "Error occurred while processing",
            "evidence": []
        }


# Optional: ChatKit/Agents specific wrapper (if available)
def generate_answer_with_chatkit_agents(context_chunks: List[Dict[str, Any]], question: str) -> Dict[str, Any]:
    """
    Wrapper for OpenAI ChatKit/Agents SDK (fallback to regular completion if not available)
    """
    try:
        # Try to import OpenAI client for newer SDK
        from openai import OpenAI
        client = OpenAI(api_key=settings.OPENAI_API_KEY)
        
        # Prepare the context with chunk IDs
        context_text = ""
        for chunk in context_chunks:
            chunk_id = chunk.get('chunk_id', 'unknown')
            text = chunk.get('text', '')
            context_text += f"\n[Chunk ID: {chunk_id}]\n{text}\n"

        # Create a structured prompt
        system_prompt = """You are a helpful assistant. ONLY use the text inside the CONTEXT markers to answer. If the answer is not present, reply exactly: "I don't know based on the provided text.""""
        
        user_prompt = f"""
        CONTEXT:
        {context_text}
        ENDCONTEXT
        
        Question: {question}
        
        Respond in JSON format with these keys: answer, summary, evidence
        The evidence should be an array of objects with chunk_id and text_snippet
        """
        
        response = client.chat.completions.create(
            model=getattr(settings, 'EMBED_MODEL', settings.OPENAI_CHAT_MODEL),
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.0,
            response_format={"type": "json_object"}  # Request JSON response
        )
        
        # Parse the JSON response
        result = json.loads(response.choices[0].message.content)
        
        return result
        
    except Exception as e:
        logger.warning(f"ChatKit/Agents not available or failed: {e}, falling back to regular completion")
        # Fall back to the regular OpenAI completion method
        return generate_answer_with_evidence(context_chunks, question)


def generate_answer_with_evidence_fallback(context_chunks: List[Dict[str, Any]], question: str) -> Dict[str, Any]:
    """
    Main function that uses either ChatKit/Agents or regular completion
    """
    return generate_answer_with_chatkit_agents(context_chunks, question)