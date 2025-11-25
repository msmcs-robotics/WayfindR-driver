from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException
from pydantic import BaseModel
from typing import List, Dict
import asyncio
import json
from core.ssh_manager import SSHManager
from core.database import get_item, get_collection

router = APIRouter()
ssh_manager = SSHManager()

class CommandExecute(BaseModel):
    command: str
    host_ids: List[str]
    username: str
    key_id: str

@router.post("/execute")
async def execute_command(cmd_data: CommandExecute):
    """Execute command on multiple hosts"""
    # Get key
    key = get_item("ssh_keys", cmd_data.key_id)
    if not key:
        raise HTTPException(status_code=404, detail="Key not found")
    
    results = {}
    
    for host_id in cmd_data.host_ids:
        host = get_item("hosts", host_id)
        if not host:
            results[host_id] = {
                "error": "Host not found",
                "host_name": host_id
            }
            continue
        
        try:
            # Get jumphost if needed
            jumphost = None
            if host.get('jumphost_id'):
                jumphost_data = get_item("hosts", host['jumphost_id'])
                if jumphost_data:
                    jumphost = {
                        'host': jumphost_data['host'],
                        'port': jumphost_data.get('port', 22),
                        'username': cmd_data.username
                    }
            
            # Connect and execute
            client = ssh_manager.connect(
                host['host'],
                host.get('port', 22),
                cmd_data.username,
                key['private_key'],
                jumphost
            )
            
            result = ssh_manager.execute_command(client, cmd_data.command)
            ssh_manager.close(client)
            
            results[host_id] = {
                "host_name": host['name'],
                "host": host['host'],
                "stdout": result['stdout'],
                "stderr": result['stderr'],
                "exit_code": result['exit_code']
            }
            
        except Exception as e:
            results[host_id] = {
                "host_name": host.get('name', host_id),
                "host": host.get('host', ''),
                "error": str(e),
                "exit_code": 1
            }
    
    return {"results": results}

class WebSocketManager:
    """Manage WebSocket connections for streaming command output"""
    
    def __init__(self):
        self.active_connections: Dict[str, WebSocket] = {}
    
    async def connect(self, websocket: WebSocket, connection_id: str):
        await websocket.accept()
        self.active_connections[connection_id] = websocket
    
    def disconnect(self, connection_id: str):
        if connection_id in self.active_connections:
            del self.active_connections[connection_id]
    
    async def send_message(self, connection_id: str, message: dict):
        if connection_id in self.active_connections:
            try:
                await self.active_connections[connection_id].send_json(message)
            except:
                self.disconnect(connection_id)

ws_manager = WebSocketManager()

@router.websocket("/stream/{connection_id}")
async def websocket_endpoint(websocket: WebSocket, connection_id: str):
    """WebSocket endpoint for streaming command execution"""
    await ws_manager.connect(websocket, connection_id)
    
    try:
        while True:
            data = await websocket.receive_json()
            
            # Expected data format:
            # {
            #   "command": "ls -la",
            #   "host_ids": ["host1", "host2"],
            #   "username": "pi",
            #   "key_id": "key123"
            # }
            
            command = data.get('command')
            host_ids = data.get('host_ids', [])
            username = data.get('username')
            key_id = data.get('key_id')
            
            if not all([command, host_ids, username, key_id]):
                await ws_manager.send_message(connection_id, {
                    "error": "Missing required fields"
                })
                continue
            
            # Get key
            key = get_item("ssh_keys", key_id)
            if not key:
                await ws_manager.send_message(connection_id, {
                    "error": "Key not found"
                })
                continue
            
            # Execute on each host
            for host_id in host_ids:
                host = get_item("hosts", host_id)
                if not host:
                    await ws_manager.send_message(connection_id, {
                        "host_id": host_id,
                        "error": "Host not found"
                    })
                    continue
                
                try:
                    # Get jumphost if needed
                    jumphost = None
                    if host.get('jumphost_id'):
                        jumphost_data = get_item("hosts", host['jumphost_id'])
                        if jumphost_data:
                            jumphost = {
                                'host': jumphost_data['host'],
                                'port': jumphost_data.get('port', 22),
                                'username': username
                            }
                    
                    # Send start message
                    await ws_manager.send_message(connection_id, {
                        "host_id": host_id,
                        "host_name": host['name'],
                        "status": "connecting"
                    })
                    
                    # Connect
                    client = ssh_manager.connect(
                        host['host'],
                        host.get('port', 22),
                        username,
                        key['private_key'],
                        jumphost
                    )
                    
                    await ws_manager.send_message(connection_id, {
                        "host_id": host_id,
                        "host_name": host['name'],
                        "status": "executing"
                    })
                    
                    # Execute command
                    result = ssh_manager.execute_command(client, command)
                    ssh_manager.close(client)
                    
                    # Send result
                    await ws_manager.send_message(connection_id, {
                        "host_id": host_id,
                        "host_name": host['name'],
                        "status": "complete",
                        "stdout": result['stdout'],
                        "stderr": result['stderr'],
                        "exit_code": result['exit_code']
                    })
                    
                except Exception as e:
                    await ws_manager.send_message(connection_id, {
                        "host_id": host_id,
                        "host_name": host.get('name', host_id),
                        "status": "error",
                        "error": str(e)
                    })
            
            # Send completion message
            await ws_manager.send_message(connection_id, {
                "status": "all_complete"
            })
            
    except WebSocketDisconnect:
        ws_manager.disconnect(connection_id)
    except Exception as e:
        print(f"WebSocket error: {e}")
        ws_manager.disconnect(connection_id)