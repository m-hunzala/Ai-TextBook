"""
Base class for Claude Code subagents
"""
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
import os
import anthropic
from dotenv import load_dotenv

load_dotenv()

class BaseSubagent(ABC):
    """
    Abstract base class for Claude Code Subagents
    """
    
    def __init__(self, name: str, description: str):
        self.name = name
        self.description = description
        self.anthropic_client = None
        
        # Initialize Anthropic client if API key is available
        api_key = os.getenv("ANTHROPIC_API_KEY")
        if api_key:
            self.anthropic_client = anthropic.Anthropic(api_key=api_key)
    
    @abstractmethod
    async def execute(self, query: str, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Execute the subagent with the given query and context
        
        Args:
            query: User query to process
            context: Additional context that might be useful
        
        Returns:
            Dictionary containing the result with relevant information
        """
        pass
    
    def get_claude_completion(self, prompt: str, system_prompt: str = None) -> str:
        """
        Helper method to get completion from Claude
        
        Args:
            prompt: The user prompt to send to Claude
            system_prompt: Optional system prompt to guide Claude's behavior
        
        Returns:
            Claude's response as a string
        """
        if not self.anthropic_client:
            # If no API key provided, return a mock response for testing purposes
            return f"[MOCK RESPONSE for prompt: {prompt[:50]}...]"
        
        try:
            messages = [{"role": "user", "content": prompt}]
            
            response = self.anthropic_client.messages.create(
                model="claude-3-5-sonnet-20241022",  # Using Claude 3.5 Sonnet
                max_tokens=1024,
                temperature=0.1,
                system=system_prompt or self.description,
                messages=messages
            )
            
            return response.content[0].text if response.content else ""
        except Exception as e:
            return f"Error calling Claude API: {str(e)}"