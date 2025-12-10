"""
Module for handling multi-step reasoning with OpenAI ChatKit/Agents or fallback completion
"""
import json
import logging
from typing import List, Dict, Any, Optional
from app.config import settings

logger = logging.getLogger(__name__)

# Try to import OpenAI ChatKit/Agents SDK
try:
    import openai
    from openai import OpenAI
    CHATKIT_AVAILABLE = True
except ImportError:
    CHATKIT_AVAILABLE = False
    import openai  # Use basic OpenAI for fallback


class AgentManager:
    """
    Wrapper for OpenAI ChatKit/Agents functionality with fallback
    """
    def __init__(self):
        self.client = None
        if CHATKIT_AVAILABLE:
            try:
                self.client = OpenAI(api_key=settings.OPENAI_API_KEY)
                logger.info("OpenAI ChatKit/Agents SDK initialized")
            except Exception as e:
                logger.warning(f"Could not initialize OpenAI ChatKit/Agents SDK: {e}")
                CHATKIT_AVAILABLE = False
                self.client = None

    def generate_answer_with_evidence(self, context_chunks: List[Dict[str, Any]], question: str) -> Dict[str, Any]:
        """
        Generate answer with evidence using either ChatKit/Agents or fallback completion
        """
        if CHATKIT_AVAILABLE and self.client:
            return self._generate_with_chatkit(context_chunks, question)
        else:
            return self._generate_with_fallback(context_chunks, question)

    def _generate_with_chatkit(self, context_chunks: List[Dict[str, Any]], question: str) -> Dict[str, Any]:
        """
        Generate answer using OpenAI ChatKit/Agents SDK (if available)
        """
        # Construct the context with chunk IDs
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
            response = self.client.chat.completions.create(
                model=getattr(settings, 'EMBED_MODEL', settings.OPENAI_CHAT_MODEL),
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                response_format={"type": "json_object"}  # Request JSON response
            )
            
            # Parse the JSON response
            response_text = response.choices[0].message.content
            result = json.loads(response_text)
            
            return result
            
        except Exception as e:
            logger.error(f"Error with ChatKit/Agents: {e}")
            # Fall back to basic completion
            return self._generate_with_fallback(context_chunks, question)

    def _generate_with_fallback(self, context_chunks: List[Dict[str, Any]], question: str) -> Dict[str, Any]:
        """
        Generate answer using basic OpenAI completion as fallback
        """
        # Construct the context with chunk IDs
        context_text = ""
        chunk_map = {}
        for chunk in context_chunks:
            chunk_id = chunk.get('chunk_id', 'unknown')
            text = chunk.get('text', '')
            page = chunk.get('page', '')
            context_text += f"\n[Chunk ID: {chunk_id}, Page: {page}]\n{text}\n"
            chunk_map[chunk_id] = text

        # Create a structured prompt for basic completion
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
            # Set the API key if using basic OpenAI
            openai.api_key = settings.OPENAI_API_KEY
            
            response = openai.ChatCompletion.create(
                model=getattr(settings, 'EMBED_MODEL', settings.OPENAI_CHAT_MODEL),
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
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
                    temperature=0.3
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
            logger.error(f"Error with fallback completion: {e}")
            return {
                "answer": f"Could not generate answer due to an error: {str(e)}",
                "summary": "Error occurred while processing",
                "evidence": []
            }


# Global agent manager instance
agent_manager = AgentManager()


def generate_answer_with_evidence(context_chunks: List[Dict[str, Any]], question: str) -> Dict[str, Any]:
    """
    Main function to generate answer with evidence using either ChatKit/Agents or fallback
    
    Args:
        context_chunks: List of chunks with chunk_id, text, etc.
        question: The question to answer
        
    Returns:
        Dict with answer, summary, and evidence containing chunk IDs and snippets
    """
    return agent_manager.generate_answer_with_evidence(context_chunks, question)


# Example usage function
def example_usage():
    """
    Example of how to use the module
    """
    context_chunks = [
        {
            "chunk_id": "chunk_1",
            "text": "The theory of relativity was developed by Albert Einstein. It revolutionized physics by redefining concepts of space and time.",
            "page": 45
        },
        {
            "chunk_id": "chunk_2", 
            "text": "Special relativity, published in 1905, deals with objects moving at constant speeds, particularly those approaching the speed of light.",
            "page": 47
        },
        {
            "chunk_id": "chunk_3",
            "text": "General relativity, completed in 1915, expanded the theory to include acceleration and gravity as curvature of spacetime.",
            "page": 52
        }
    ]
    
    question = "What are the two main components of Einstein's theory of relativity?"
    
    result = generate_answer_with_evidence(context_chunks, question)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    example_usage()