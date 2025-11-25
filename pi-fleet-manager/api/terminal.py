from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Dict
import paramiko
import asyncio
from core.database import get_item

router = APIRouter()

class TerminalSession:
    """Manage interactive SSH terminal session"""
    
    def __init__(self, host: str, port: int, username: str, key_path: str, jumphost=None):
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.channel = None
        
        # Connect
        if jumphost:
            # Connect through jumphost
            jump_client = paramiko.SSHClient()
            jump_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            jump_client.connect(
                hostname=jumphost['host'],
                port=jumphost.get('port', 22),
                username=jumphost['username'],
                key_filename=key_path,
                timeout=10
            )
            
            jump_transport = jump_client.get_transport()
            dest_addr = (host, port)
            local_addr = ('127.0.0.1', 0)
            channel = jump_transport.open_channel("direct-tcpip", dest_addr, local_addr)
            
            self.client.connect(
                hostname=host,
                port=port,
                username=username,
                key_filename=key_path,
                sock=channel,
                timeout=10
            )
        else:
            self.client.connect(
                hostname=host,
                port=port,
                username=username,
                key_filename=key_path,
                timeout=10
            )
        
        # Open interactive shell
        self.channel = self.client.invoke_shell(
            term='xterm-256color',
            width=80,
            height=24
        )
        self.channel.settimeout(0.1)
    
    def read(self) -> str:
        """Read output from terminal"""
        try:
            if self.channel.recv_ready():
                return self.channel.recv(4096).decode('utf-8', errors='ignore')
        except:
            pass
        return ""
    
    def write(self, data: str):
        """Write input to terminal"""
        try:
            self.channel.send(data)
        except Exception as e:
            print(f"Write error: {e}")
    
    def resize(self, width: int, height: int):
        """Resize terminal"""
        try:
            self.channel.resize_pty(width=width, height=height)
        except:
            pass
    
    def close(self):
        """Close session"""
        if self.channel:
            self.channel.close()
        if self.client:
            self.client.close()

class TerminalManager:
    """Manage multiple terminal sessions"""
    
    def __init__(self):
        self.sessions: Dict[str, TerminalSession] = {}
    
    def create_session(self, session_id: str, host: str, port: int, 
                      username: str, key_path: str, jumphost=None) -> TerminalSession:
        """Create new terminal session"""
        session = TerminalSession(host, port, username, key_path, jumphost)
        self.sessions[session_id] = session
        return session
    
    def get_session(self, session_id: str) -> TerminalSession:
        """Get existing session"""
        return self.sessions.get(session_id)
    
    def close_session(self, session_id: str):
        """Close and remove session"""
        if session_id in self.sessions:
            self.sessions[session_id].close()
            del self.sessions[session_id]

terminal_manager = TerminalManager()

@router.websocket("/ws/{session_id}")
async def terminal_websocket(websocket: WebSocket, session_id: str):
    """WebSocket endpoint for interactive terminal"""
    await websocket.accept()
    
    session = None
    
    try:
        # Wait for connection parameters
        init_data = await websocket.receive_json()
        
        host_id = init_data.get('host_id')
        username = init_data.get('username')
        key_id = init_data.get('key_id')
        
        if not all([host_id, username, key_id]):
            await websocket.send_json({"error": "Missing connection parameters"})
            await websocket.close()
            return
        
        # Get host and key info
        host = get_item("hosts", host_id)
        key = get_item("ssh_keys", key_id)
        
        if not host or not key:
            await websocket.send_json({"error": "Host or key not found"})
            await websocket.close()
            return
        
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
        
        # Create terminal session
        try:
            session = terminal_manager.create_session(
                session_id,
                host['host'],
                host.get('port', 22),
                username,
                key['private_key'],
                jumphost
            )
            
            await websocket.send_json({"status": "connected"})
            
        except Exception as e:
            await websocket.send_json({"error": f"Connection failed: {str(e)}"})
            await websocket.close()
            return
        
        # Start bidirectional communication
        async def read_from_terminal():
            """Read from SSH and send to WebSocket"""
            while True:
                try:
                    output = session.read()
                    if output:
                        await websocket.send_json({
                            "type": "output",
                            "data": output
                        })
                    await asyncio.sleep(0.05)  # Small delay to prevent busy loop
                except Exception as e:
                    break
        
        async def read_from_websocket():
            """Read from WebSocket and send to SSH"""
            while True:
                try:
                    data = await websocket.receive_json()
                    
                    if data.get('type') == 'input':
                        session.write(data.get('data', ''))
                    elif data.get('type') == 'resize':
                        session.resize(
                            data.get('width', 80),
                            data.get('height', 24)
                        )
                    
                except WebSocketDisconnect:
                    break
                except Exception as e:
                    break
        
        # Run both tasks concurrently
        await asyncio.gather(
            read_from_terminal(),
            read_from_websocket()
        )
        
    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"Terminal WebSocket error: {e}")
    finally:
        if session:
            terminal_manager.close_session(session_id)
        try:
            await websocket.close()
        except:
            pass