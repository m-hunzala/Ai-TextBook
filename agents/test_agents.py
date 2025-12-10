import subprocess
import time
import requests
import json

def test_agents():
    print("Testing Docusaurus Book App Agents...")
    
    # Test Summarizer Agent
    print("\n--- Testing Summarizer Agent ---")
    try:
        summarizer_data = {
            "text": "Artificial intelligence (AI) is intelligence demonstrated by machines. It is a field that studies intelligent agents that perceive their environment and take actions to maximize their goals.",
            "max_lines": 3
        }
        response = requests.post("http://localhost:8001/summarize", json=summarizer_data)
        if response.status_code == 200:
            result = response.json()
            print(f"✓ Summarizer Agent: {result['summary']}")
        else:
            print(f"✗ Summarizer Agent failed: {response.text}")
    except Exception as e:
        print(f"✗ Summarizer Agent error: {str(e)}")
    
    # Test Code Runner Agent
    print("\n--- Testing Code Runner Agent ---")
    try:
        code_data = {
            "code": "print('Hello from Python!')\nprint(2 + 3)",
            "language": "python"
        }
        response = requests.post("http://localhost:8002/run_code", json=code_data)
        if response.status_code == 200:
            result = response.json()
            print(f"✓ Code Runner Agent: stdout='{result['stdout']}', result='{result['result']}'")
        else:
            print(f"✗ Code Runner Agent failed: {response.text}")
    except Exception as e:
        print(f"✗ Code Runner Agent error: {str(e)}")
    
    # Test Translator Agent
    print("\n--- Testing Translator Agent ---")
    try:
        translation_data = {
            "text": "Hello world! This has **markdown** formatting.",
            "target_lang": "es"
        }
        response = requests.post("http://localhost:8003/translate", json=translation_data)
        if response.status_code == 200:
            result = response.json()
            print(f"✓ Translator Agent: {result['translated_text']}")
        else:
            print(f"✗ Translator Agent failed: {response.text}")
    except Exception as e:
        print(f"✗ Translator Agent error: {str(e)}")

if __name__ == "__main__":
    test_agents()