from fastapi import APIRouter, HTTPException, UploadFile, File
from pydantic import BaseModel
from typing import List
import tempfile
import os
from pathlib import Path
from core.ssh_manager import SSHManager
from core.database import get_item

router = APIRouter()
ssh_manager = SSHManager()

class FileTransfer(BaseModel):
    host_ids: List[str]
    username: str
    key_id: str
    local_path: str
    remote_path: str = "/opt/bakery"

class BakerySetup(BaseModel):
    host_ids: List[str]
    username: str
    key_id: str

@router.post("/setup-bakery")
async def setup_bakery_folders(setup_data: BakerySetup):
    """Setup /opt/bakery folders on hosts"""
    key = get_item("ssh_keys", setup_data.key_id)
    if not key:
        raise HTTPException(status_code=404, detail="Key not found")
    
    results = {}
    
    for host_id in setup_data.host_ids:
        host = get_item("hosts", host_id)
        if not host:
            results[host_id] = {"error": "Host not found"}
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
                        'username': setup_data.username
                    }
            
            # Connect
            client = ssh_manager.connect(
                host['host'],
                host.get('port', 22),
                setup_data.username,
                key['private_key'],
                jumphost
            )
            
            # Create /opt/bakery with proper permissions
            commands = [
                'sudo chmod 777 /opt',
                'mkdir -p /opt/bakery',
                'chmod 755 /opt/bakery'
            ]
            
            outputs = []
            for cmd in commands:
                result = ssh_manager.execute_command(client, cmd)
                outputs.append({
                    "command": cmd,
                    "exit_code": result['exit_code']
                })
            
            ssh_manager.close(client)
            
            results[host_id] = {
                "success": True,
                "host_name": host['name'],
                "commands": outputs
            }
            
        except Exception as e:
            results[host_id] = {
                "success": False,
                "error": str(e)
            }
    
    return {"results": results}

@router.post("/transfer")
async def transfer_file(transfer_data: FileTransfer):
    """Transfer file or directory to multiple hosts"""
    key = get_item("ssh_keys", transfer_data.key_id)
    if not key:
        raise HTTPException(status_code=404, detail="Key not found")
    
    local_path = Path(transfer_data.local_path)
    if not local_path.exists():
        raise HTTPException(status_code=404, detail="Local path not found")
    
    results = {}
    
    for host_id in transfer_data.host_ids:
        host = get_item("hosts", host_id)
        if not host:
            results[host_id] = {"error": "Host not found"}
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
                        'username': transfer_data.username
                    }
            
            # Connect
            client = ssh_manager.connect(
                host['host'],
                host.get('port', 22),
                transfer_data.username,
                key['private_key'],
                jumphost
            )
            
            # Ensure remote directory exists
            remote_dir = transfer_data.remote_path
            mkdir_result = ssh_manager.execute_command(
                client,
                f'mkdir -p {remote_dir}'
            )
            
            # Transfer file or directory
            if local_path.is_file():
                remote_file = f"{remote_dir}/{local_path.name}"
                success = ssh_manager.copy_file(
                    client,
                    str(local_path),
                    remote_file
                )
            else:
                success = ssh_manager.copy_directory(
                    client,
                    str(local_path),
                    remote_dir
                )
            
            ssh_manager.close(client)
            
            results[host_id] = {
                "success": success,
                "host_name": host['name'],
                "remote_path": remote_dir
            }
            
        except Exception as e:
            results[host_id] = {
                "success": False,
                "error": str(e)
            }
    
    return {"results": results}

@router.post("/upload")
async def upload_and_transfer(
    file: UploadFile = File(...),
    host_ids: str = "",
    username: str = "",
    key_id: str = "",
    remote_path: str = "/opt/bakery"
):
    """Upload a file and transfer to hosts"""
    # Parse host_ids (sent as comma-separated string)
    host_id_list = [h.strip() for h in host_ids.split(',') if h.strip()]
    
    if not host_id_list:
        raise HTTPException(status_code=400, detail="No hosts specified")
    
    key = get_item("ssh_keys", key_id)
    if not key:
        raise HTTPException(status_code=404, detail="Key not found")
    
    # Save uploaded file temporarily
    with tempfile.NamedTemporaryFile(delete=False, suffix=f"_{file.filename}") as tmp:
        content = await file.read()
        tmp.write(content)
        tmp_path = tmp.name
    
    try:
        # Transfer to all hosts
        transfer_data = FileTransfer(
            host_ids=host_id_list,
            username=username,
            key_id=key_id,
            local_path=tmp_path,
            remote_path=remote_path
        )
        
        results = await transfer_file(transfer_data)
        
        return results
        
    finally:
        # Clean up temp file
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)

@router.post("/sync-directory")
async def sync_directory(transfer_data: FileTransfer):
    """
    Sync entire directory to hosts using rsync-like behavior
    This will recursively copy all files and subdirectories
    """
    return await transfer_file(transfer_data)