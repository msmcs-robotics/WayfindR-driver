# Jetson Web Control Chat Deployment

**Date:** 2026-03-26
**Status:** Deployed and verified

## Deployment Summary

The AMBOT web control chat app was deployed to the Jetson Orin Nano and is running
in simulation mode with RAG API integration.

### What Was Done

1. **Rsync** — `web_control/` synced from local to `jetson:~/ambot/web_control/`
   (was already present, incremental update only)
2. **Dependencies verified** — Flask 3.1.3, Flask-SocketIO 5.6.1 already installed
3. **Started web dashboard** — simulation mode, port 5000, RAG connected
4. **SSH tunnel** — local port 5123 forwarded to Jetson port 5000

### Endpoints Verified

| Endpoint | Result |
|---|---|
| `GET /api/health` | `{"status":"ok"}` |
| `GET /api/chat/status` | `{"available":true,"url":"http://localhost:8000"}` |

### RAG Backend

The RAG API at `http://localhost:8000` on the Jetson is healthy:
- Database: OK
- Redis: OK
- Embedding service: OK
- LLM (Ollama): OK

## How to Access

From the development machine (WSL2):

```bash
# Set up SSH tunnel (if not already running)
ssh -f -N -L 5123:localhost:5000 jetson

# Open in browser
# http://localhost:5123
```

## How to Restart

```bash
# Kill existing
ssh jetson "pkill -f 'web_control/run.py'"

# Start again
ssh jetson "cd ~/ambot && RAG_API_URL=http://localhost:8000 nohup python3 web_control/run.py --simulate --port 5000 > /tmp/web_control.log 2>&1 &"
```

## Process Details

- **PID location:** background process on Jetson
- **Log file:** `/tmp/web_control.log` on Jetson
- **Bind address:** `0.0.0.0:5000` (accessible from Jetson LAN and via tunnel)
- **Mode:** SIMULATION (no hardware attached)
- **Environment:** `RAG_API_URL=http://localhost:8000`

## Notes

- The Werkzeug development server warning is expected; this is a dev/test deployment
- Flask-SocketIO `__version__` attribute is deprecated; use `importlib.metadata` instead
- The process runs in the background via `nohup`; it will survive SSH disconnects
  but will NOT survive a Jetson reboot (no systemd service configured)
