"""
Registry for managing Claude Code subagents
"""
from typing import Dict, List, Optional
from .base_subagent import BaseSubagent


class SubagentRegistry:
    """
    Registry for managing available subagents
    """
    
    def __init__(self):
        self.subagents: Dict[str, BaseSubagent] = {}
    
    def register_subagent(self, name: str, subagent: BaseSubagent):
        """
        Register a subagent with the registry
        
        Args:
            name: Unique name for the subagent
            subagent: Instance of BaseSubagent
        """
        self.subagents[name] = subagent
    
    def get_subagent(self, name: str) -> Optional[BaseSubagent]:
        """
        Get a subagent by name
        
        Args:
            name: Name of the subagent to retrieve
        
        Returns:
            Subagent instance if found, None otherwise
        """
        return self.subagents.get(name)
    
    def list_subagents(self) -> List[str]:
        """
        Get a list of all registered subagent names
        
        Returns:
            List of subagent names
        """
        return list(self.subagents.keys())
    
    def get_subagent_description(self, name: str) -> Optional[str]:
        """
        Get the description of a subagent
        
        Args:
            name: Name of the subagent
        
        Returns:
            Description of the subagent if found, None otherwise
        """
        subagent = self.subagents.get(name)
        return subagent.description if subagent else None
    
    async def execute_subagent(self, name: str, query: str, context: dict = None) -> dict:
        """
        Execute a subagent with the given query and context
        
        Args:
            name: Name of the subagent to execute
            query: Query to process
            context: Additional context for the subagent
        
        Returns:
            Result from the subagent execution
        """
        subagent = self.get_subagent(name)
        if not subagent:
            return {
                "error": f"Subagent '{name}' not found",
                "available_subagents": self.list_subagents()
            }
        
        return await subagent.execute(query, context)