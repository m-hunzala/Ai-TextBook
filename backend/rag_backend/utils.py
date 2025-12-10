import os
import re
from typing import List, Dict, Tuple
from pathlib import Path
import yaml
from dotenv import load_dotenv

load_dotenv()

def extract_markdown_docs(docs_path: str = "../../docs") -> List[Dict[str, str]]:
    """
    Extract all markdown documents from the specified directory
    """
    docs_path = os.path.abspath(docs_path)
    documents = []
    
    for root, dirs, files in os.walk(docs_path):
        for file in files:
            if file.endswith('.md') or file.endswith('.mdx'):
                file_path = os.path.join(root, file)
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    
                    # Extract frontmatter if present
                    frontmatter = {}
                    if content.startswith('---'):
                        try:
                            end_frontmatter = content.find('---', 3)
                            frontmatter_str = content[3:end_frontmatter].strip()
                            frontmatter = yaml.safe_load(frontmatter_str)
                            content = content[end_frontmatter+3:].strip()
                        except:
                            pass
                    
                    # Clean up content by removing Docusaurus-specific elements
                    # Remove import/export statements
                    content = re.sub(r'^\s*import.*?;$', '', content, flags=re.MULTILINE)
                    content = re.sub(r'^\s*export.*?;$', '', content, flags=re.MULTILINE)
                    # Remove JSX components
                    content = re.sub(r'<\s*\/?\s*\w+.*?>', '', content, flags=re.MULTILINE)
                    
                    doc_info = {
                        'id': str(Path(file_path).relative_to(docs_path).with_suffix('')),
                        'title': frontmatter.get('title', ''),
                        'content': content,
                        'source': str(Path(file_path).relative_to(docs_path)),
                        'tags': frontmatter.get('tags', []),
                        'last_modified': os.path.getmtime(file_path)
                    }
                    
                    documents.append(doc_info)
    
    return documents

def extract_docusaurus_docs() -> List[Dict[str, str]]:
    """
    Specifically target Docusaurus-style documentation
    """
    # Look for common Docusaurus directories
    potential_paths = [
        os.path.join(os.path.dirname(__file__), "../../docs"),
        os.path.join(os.path.dirname(__file__), "../../../docs"),
        os.path.join(os.path.dirname(__file__), "../../blog"),
        os.path.join(os.path.dirname(__file__), "../../pages"),
    ]
    
    for path in potential_paths:
        abs_path = os.path.abspath(path)
        if os.path.exists(abs_path):
            return extract_markdown_docs(abs_path)
    
    # If specific paths don't exist, look for any markdown in the project
    current_dir = os.path.dirname(__file__)
    parent_dir = os.path.dirname(current_dir)
    
    for path in [parent_dir, os.path.join(parent_dir, "docs"), os.path.join(parent_dir, "blog")]:
        abs_path = os.path.abspath(path)
        if os.path.exists(abs_path):
            return extract_markdown_docs(abs_path)
    
    return []