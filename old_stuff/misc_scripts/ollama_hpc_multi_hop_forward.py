from fastapi import FastAPI, Request
import httpx
import uvicorn

app = FastAPI()
OLLAMA_URL = "http://localhost:11434"

@app.api_route("/api/{path:path}", methods=["GET", "POST", "DELETE"])
async def proxy(request: Request, path: str):
    """Proxy all Ollama API requests"""
    url = f"{OLLAMA_URL}/api/{path}"
    
    async with httpx.AsyncClient() as client:
        # Forward request
        if request.method == "GET":
            response = await client.get(url, params=request.query_params)
        elif request.method == "POST":
            body = await request.json()
            response = await client.post(url, json=body)
        elif request.method == "DELETE":
            response = await client.delete(url)
        else:
            return {"error": "Method not allowed"}, 405
    
    return response.json()

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)