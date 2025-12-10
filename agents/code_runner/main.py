from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import subprocess
import tempfile
import os
import io
import sys
from contextlib import redirect_stdout, redirect_stderr
import json

app = FastAPI(
    title="CodeExampleRunnerAgent",
    description="A code execution service that safely runs JS/Python code snippets and returns results.",
    version="1.0.0"
)

class CodeRunRequest(BaseModel):
    code: str
    language: str  # "python" or "javascript"


class CodeRunResponse(BaseModel):
    stdout: str
    result: str
    error: str = None


@app.post("/run_code", response_model=CodeRunResponse)
async def run_code(request: CodeRunRequest):
    """
    Execute a code snippet and return stdout, result, and any errors.
    
    Args:
        request (CodeRunRequest): Contains the code snippet and language
        
    Returns:
        CodeRunResponse: Contains stdout, result, and error information
    """
    try:
        if request.language.lower() == "python":
            return run_python_code(request.code)
        elif request.language.lower() == "javascript" or request.language.lower() == "js":
            return run_javascript_code(request.code)
        else:
            raise HTTPException(status_code=400, detail=f"Unsupported language: {request.language}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error running code: {str(e)}")


def run_python_code(code: str):
    """Safely execute Python code and capture output."""
    # Create a temporary file with the code
    with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
        f.write(code)
        temp_file = f.name

    try:
        # Execute the Python code with timeout and capture output
        result = subprocess.run(
            [sys.executable, temp_file],
            capture_output=True,
            text=True,
            timeout=10  # 10 second timeout
        )
        
        stdout_output = result.stdout
        stderr_output = result.stderr
        return_code = result.returncode
        
        if return_code == 0:
            return CodeRunResponse(stdout=stdout_output, result="Execution successful")
        else:
            return CodeRunResponse(
                stdout=stdout_output, 
                result="Execution failed", 
                error=stderr_output
            )
    except subprocess.TimeoutExpired:
        return CodeRunResponse(
            stdout="", 
            result="Execution timed out", 
            error="Code execution exceeded time limit"
        )
    except Exception as e:
        return CodeRunResponse(
            stdout="", 
            result="Execution error", 
            error=str(e)
        )
    finally:
        # Clean up the temporary file
        if os.path.exists(temp_file):
            os.remove(temp_file)


def run_javascript_code(code: str):
    """Execute JavaScript code with Node.js."""
    # Create a temporary file with the code
    with tempfile.NamedTemporaryFile(mode='w', suffix='.js', delete=False) as f:
        f.write(code)
        temp_file = f.name

    try:
        # Execute the JavaScript code with Node.js
        result = subprocess.run(
            ["node", temp_file],
            capture_output=True,
            text=True,
            timeout=10  # 10 second timeout
        )
        
        stdout_output = result.stdout
        stderr_output = result.stderr
        return_code = result.returncode
        
        if return_code == 0:
            return CodeRunResponse(stdout=stdout_output, result="Execution successful")
        else:
            return CodeRunResponse(
                stdout=stdout_output, 
                result="Execution failed", 
                error=stderr_output
            )
    except subprocess.TimeoutExpired:
        return CodeRunResponse(
            stdout="", 
            result="Execution timed out", 
            error="Code execution exceeded time limit"
        )
    except FileNotFoundError:
        return CodeRunResponse(
            stdout="", 
            result="Node.js not found", 
            error="Node.js is not installed or not in PATH"
        )
    except Exception as e:
        return CodeRunResponse(
            stdout="", 
            result="Execution error", 
            error=str(e)
        )
    finally:
        # Clean up the temporary file
        if os.path.exists(temp_file):
            os.remove(temp_file)


@app.get("/")
async def root():
    return {"message": "CodeExampleRunnerAgent is running", "version": "1.0.0"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8002)