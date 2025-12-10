from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI()

class Req(BaseModel):
    text: str

@app.post("/summarize")
def summarize(req: Req):
    # simple heuristic summarizer (placeholder)
    lines = req.text.strip().split("\n")
    first = lines[0] if lines else ""
    return {"summary": first[:300] + ("..." if len(first) > 300 else "")}

@app.get("/")
def root():
    return {"message": "Simple Summarizer Agent is running"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)