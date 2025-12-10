"""
Simulation Helper Subagent for Claude Code
Suggests Gazebo commands and simulation configurations
"""
from .base_subagent import BaseSubagent
from typing import Dict, Any, Optional
import re
import json


class SimHelper(BaseSubagent):
    """
    Simulation Helper subagent that suggests Gazebo commands and configurations
    """
    
    def __init__(self):
        super().__init__(
            name="sim-helper",
            description="A specialist in robotics simulation using Gazebo, Ignition, and related tools. Can suggest simulation commands, create world files, configure simulation parameters, and provide advice on simulation best practices."
        )
    
    async def execute(self, query: str, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Execute the Simulation Helper subagent
        
        Args:
            query: User query about simulation
            context: Additional context
            
        Returns:
            Dictionary containing simulation suggestions and configurations
        """
        # Define a system prompt specific to simulation
        system_prompt = """
You are a robotics simulation expert specializing in Gazebo, Ignition, and related simulation tools. Your role is to help users create simulation environments, suggest appropriate commands, and configure simulation parameters.

When responding:
1. Provide valid Gazebo world files in SDF format when needed
2. Suggest appropriate simulation commands for launching and controlling simulations
3. Include configuration files for robots, sensors, and environments
4. Follow best practices for simulation performance and accuracy
5. Consider physics engines, sensors, and realistic environments

Format your response as a JSON object with the following structure:
{
  "summary": "Brief summary of what you're providing",
  "config_files": [
    {
      "type": "sdf|urdf|world|launch|config",
      "title": "File title",
      "content": "Actual file content",
      "description": "What this config does"
    }
  ],
  "commands": ["gazebo...", "ign gazebo...", etc.],
  "explanation": "Detailed explanation of the simulation setup"
}
"""
        
        # Create the prompt for Claude
        prompt = f"""
The user has a question about robotics simulation, likely related to Gazebo, Ignition, or simulation environments:

Query: {query}

Context: {context or 'No additional context provided'}

Please provide appropriate simulation configurations, commands, and explanations.
"""
        
        # Get response from Claude
        claude_response = self.get_claude_completion(prompt, system_prompt)
        
        # Try to parse the response as JSON
        try:
            # Attempt to extract JSON from Claude's response
            json_match = re.search(r'\{.*\}', claude_response, re.DOTALL)
            if json_match:
                result = json.loads(json_match.group())
            else:
                # If no JSON found, return a structured response
                result = {
                    "summary": f"Simulation assistance for: {query}",
                    "config_files": [{
                        "type": "world",
                        "title": "Sample Gazebo World",
                        "content": "<sdf version='1.7'>\n  <world name='default'>\n    <!-- Simulation world content would be here -->\n  </world>\n</sdf>",
                        "description": "Sample Gazebo world file based on your request"
                    }],
                    "commands": ["gazebo --verbose your_world.world"],
                    "explanation": claude_response
                }
        except json.JSONDecodeError:
            # If JSON parsing fails, create a structured response
            result = {
                "summary": f"Simulation assistance for: {query}",
                "config_files": [{
                    "type": "text",
                    "title": "Response",
                    "content": claude_response,
                    "description": "Claude's response to your simulation query"
                }],
                "commands": [],
                "explanation": "Response generated from Claude's text"
            }
        
        return result