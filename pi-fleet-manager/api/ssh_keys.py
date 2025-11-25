from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, List
import uuid
from core.database import get_collection, add_item, delete_item, get_item
from core.key_manager import KeyManager

router = APIRouter()
key_manager = KeyManager()

class KeyGenerate(BaseModel):
    name: str
    key_size: int = 4096

class KeyCopy(BaseModel):
    key_id: str
    host_id: str
    username: str
    password: Optional[str] = None

@router.get("/")
async def list_keys():
    """List all registered SSH keys"""
    return {"keys": get_collection("ssh_keys")}

@router.post("/generate")
async def generate_key(key_data: KeyGenerate):
    """Generate a new SSH key pair"""
    try:
        key_info = key_manager.generate_key(key_data.name, key_data.key_size)
        
        # Store in database
        key_record = {
            "id": str(uuid.uuid4()),
            "name": key_info['name'],
            "private_key": key_info['private_key'],
            "public_key": key_info['public_key']
        }
        add_item("ssh_keys", key_record)
        
        return key_record
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/register")
async def register_existing_key(name: str, private_key_path: str):
    """Register an existing SSH key"""
    from pathlib import Path
    
    if not Path(private_key_path).exists():
        raise HTTPException(status_code=404, detail="Key file not found")
    
    pub_key_path = f"{private_key_path}.pub"
    if not Path(pub_key_path).exists():
        raise HTTPException(status_code=404, detail="Public key file not found")
    
    key_record = {
        "id": str(uuid.uuid4()),
        "name": name,
        "private_key": private_key_path,
        "public_key": pub_key_path
    }
    add_item("ssh_keys", key_record)
    
    return key_record

@router.delete("/{key_id}")
async def delete_key(key_id: str):
    """Delete a key registration (does not delete files)"""
    if not delete_item("ssh_keys", key_id):
        raise HTTPException(status_code=404, detail="Key not found")
    return {"message": "Key deleted"}

@router.post("/copy-to-host")
async def copy_key_to_host(copy_data: KeyCopy):
    """Copy SSH key to a host"""
    key = get_item("ssh_keys", copy_data.key_id)
    if not key:
        raise HTTPException(status_code=404, detail="Key not found")
    
    from core.database import get_item as get_db_item
    host = get_db_item("hosts", copy_data.host_id)
    if not host:
        raise HTTPException(status_code=404, detail="Host not found")
    
    try:
        success = key_manager.copy_key_to_host(
            key['private_key'],
            host['host'],
            host.get('port', 22),
            copy_data.username,
            copy_data.password
        )
        
        if success:
            return {"message": "Key copied successfully"}
        else:
            raise HTTPException(status_code=500, detail="Failed to copy key")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/setup-agent")
async def setup_ssh_agent():
    """Setup SSH agent and shell RC files"""
    try:
        agent_running = key_manager.ensure_agent_running()
        rc_setup = key_manager.setup_shell_rc()
        
        return {
            "agent_running": agent_running,
            "rc_files_updated": rc_setup
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/{key_id}/add-to-agent")
async def add_key_to_agent(key_id: str):
    """Add key to SSH agent"""
    key = get_item("ssh_keys", key_id)
    if not key:
        raise HTTPException(status_code=404, detail="Key not found")
    
    try:
        success = key_manager.add_key_to_agent(key['private_key'])
        if success:
            return {"message": "Key added to agent"}
        else:
            raise HTTPException(status_code=500, detail="Failed to add key to agent")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/scan-filesystem")
async def scan_filesystem_keys():
    """Scan ~/.ssh for existing keys"""
    try:
        keys = key_manager.list_keys()
        return {"keys": keys}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))