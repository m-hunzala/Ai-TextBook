// callSummarizer(text) - corrected to work with the actual summarizer agent
async function callSummarizer(text){
  const res = await fetch('http://localhost:8001/summarize', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({text})
  });
  return res.json();
}

// Example usage:
// const summary = await callSummarizer("Your long text here...");
// console.log(summary);