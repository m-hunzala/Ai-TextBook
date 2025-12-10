"""
Claude Code Subagents for RAG system
"""
from .subagent_registry import SubagentRegistry
from .base_subagent import BaseSubagent
from .ros_helper import RosHelper
from .sim_helper import SimHelper

# Initialize the registry with available subagents
registry = SubagentRegistry()
registry.register_subagent("ros-helper", RosHelper())
registry.register_subagent("sim-helper", SimHelper())