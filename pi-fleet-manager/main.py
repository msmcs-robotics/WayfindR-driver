from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from pathlib import Path

# Import API routers
from api.hosts import router as hosts_router
from api.users import router as users_router
from api.ssh_keys import router as ssh_keys_router
from api.commands import router as commands_router
from api.files import router as files_router
from api.terminal import router as terminal_router

from core.database import init_db

app = FastAPI(title="Pi Fleet Manager", version="1.0.0")

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize database
init_db()

# Include API routers
app.include_router(hosts_router, prefix="/api/hosts", tags=["hosts"])
app.include_router(users_router, prefix="/api/users", tags=["users"])
app.include_router(ssh_keys_router, prefix="/api/keys", tags=["ssh_keys"])
app.include_router(commands_router, prefix="/api/commands", tags=["commands"])
app.include_router(files_router, prefix="/api/files", tags=["files"])
app.include_router(terminal_router, prefix="/api/terminal", tags=["terminal"])

# Serve static files
static_path = Path(__file__).parent / "frontend"
if static_path.exists():
    app.mount("/static", StaticFiles(directory=str(static_path)), name="static")

@app.get("/", response_class=HTMLResponse)
async def root():
    """Serve the main page"""
    index_file = static_path / "index.html"
    if index_file.exists():
        return index_file.read_text()
    return "<h1>Pi Fleet Manager</h1><p>Frontend not found</p>"

@app.get("/health")
async def health():
    """Health check endpoint"""
    return {"status": "healthy"}

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)