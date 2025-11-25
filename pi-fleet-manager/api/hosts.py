from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, List
import uuid
from core.database import get_collection, add_item, update_item, delete_item, get_item
from core.ssh_manager import SSHManager

router = APIRouter()
ssh_manager = SSHManager()

class Host(BaseModel):
    name: str
    host: str
    port: int = 22
    mac_address: Optional[str] = None
    jumphost_id: Optional[str] = None
    description: Optional[str] = None

class HostUpdate(BaseModel):
    name: Optional[str] = None
    host: Optional[str] = None
    port: Optional[int] = None
    mac_address: Optional[str] = None
    jumphost_id: Optional[str] = None
    description: Optional[str] = None

@router.get("/")
async def list_hosts():
    """List all hosts"""
    return {"hosts": get_collection("hosts")}

@router.post("/")
async def create_host(host: Host):
    """Create a new host"""
    host_dict = host.dict()
    host_dict['id'] = str(uuid.uuid4())
    add_item("hosts", host_dict)
    return host_dict

@router.get("/{host_id}")
async def get_host(host_id: str):
    """Get a specific host"""
    host = get_item("hosts", host_id)
    if not host:
        raise HTTPException(status_code=404, detail="Host not found")
    return host

@router.put("/{host_id}")
async def update_host(host_id: str, host_update: HostUpdate):
    """Update a host"""
    updates = {k: v for k, v in host_update.dict().items() if v is not None}
    if not update_item("hosts", host_id, updates):
        raise HTTPException(status_code=404, detail="Host not found")
    return get_item("hosts", host_id)

@router.delete("/{host_id}")
async def delete_host(host_id: str):
    """Delete a host"""
    if not delete_item("hosts", host_id):
        raise HTTPException(status_code=404, detail="Host not found")
    
    # Also delete host-user mappings
    from core.database import load_db, save_db
    db = load_db()
    db['host_users'] = [
        mapping for mapping in db.get('host_users', [])
        if mapping.get('host_id') != host_id
    ]
    save_db(db)
    
    return {"message": "Host deleted"}

@router.post("/{host_id}/test")
async def test_host_connection(host_id: str, username: str, key_path: str):
    """Test connection to a host"""
    host = get_item("hosts", host_id)
    if not host:
        raise HTTPException(status_code=404, detail="Host not found")
    
    # Get jumphost if specified
    jumphost = None
    if host.get('jumphost_id'):
        jumphost_data = get_item("hosts", host['jumphost_id'])
        if jumphost_data:
            jumphost = {
                'host': jumphost_data['host'],
                'port': jumphost_data.get('port', 22),
                'username': username
            }
    
    try:
        success = ssh_manager.test_connection(
            host['host'],
            host.get('port', 22),
            username,
            key_path,
            jumphost
        )
        return {"success": success}
    except Exception as e:
        return {"success": False, "error": str(e)}