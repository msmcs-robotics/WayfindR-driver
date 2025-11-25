from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, List
import uuid
from core.database import get_collection, add_item, update_item, delete_item, get_item, load_db, save_db
router = APIRouter()

class User(BaseModel):
    username: str
    description: Optional[str] = None

class UserUpdate(BaseModel):
    username: Optional[str] = None
    description: Optional[str] = None

class HostUserMapping(BaseModel):
    user_id: str
    host_ids: List[str]

@router.get("/")
async def list_users():
    """List all users"""
    return {"users": get_collection("users")}

@router.post("/")
async def create_user(user: User):
    """Create a new user"""
    user_dict = user.dict()
    user_dict['id'] = str(uuid.uuid4())
    add_item("users", user_dict)
    return user_dict

@router.get("/{user_id}")
async def get_user(user_id: str):
    """Get a specific user"""
    user = get_item("users", user_id)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return user

@router.put("/{user_id}")
async def update_user(user_id: str, user_update: UserUpdate):
    """Update a user"""
    updates = {k: v for k, v in user_update.dict().items() if v is not None}
    if not update_item("users", user_id, updates):
        raise HTTPException(status_code=404, detail="User not found")
    return get_item("users", user_id)

@router.delete("/{user_id}")
async def delete_user(user_id: str):
    """Delete a user"""
    if not delete_item("users", user_id):
        raise HTTPException(status_code=404, detail="User not found")
    
    # Also delete host-user mappings
    db = load_db()
    db['host_users'] = [
        mapping for mapping in db.get('host_users', [])
        if mapping.get('user_id') != user_id
    ]
    save_db(db)
    
    return {"message": "User deleted"}

@router.post("/{user_id}/hosts")
async def map_user_to_hosts(user_id: str, mapping: HostUserMapping):
    """Map user to multiple hosts"""
    user = get_item("users", user_id)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    
    # Validate all hosts exist
    hosts = get_collection("hosts")
    host_ids = {h['id'] for h in hosts}
    for host_id in mapping.host_ids:
        if host_id not in host_ids:
            raise HTTPException(status_code=404, detail=f"Host {host_id} not found")
    
    # Update mappings
    db = load_db()
    
    # Remove existing mappings for this user
    db['host_users'] = [
        m for m in db.get('host_users', [])
        if m.get('user_id') != user_id
    ]
    
    # Add new mappings
    for host_id in mapping.host_ids:
        db['host_users'].append({
            'user_id': user_id,
            'host_id': host_id,
            'id': str(uuid.uuid4())
        })
    
    save_db(db)
    return {"message": "Mappings updated"}

@router.get("/{user_id}/hosts")
async def get_user_hosts(user_id: str):
    """Get all hosts mapped to a user"""
    user = get_item("users", user_id)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    
    db = load_db()
    host_ids = [
        m['host_id'] for m in db.get('host_users', [])
        if m.get('user_id') == user_id
    ]
    
    hosts = [h for h in get_collection("hosts") if h['id'] in host_ids]
    return {"hosts": hosts}