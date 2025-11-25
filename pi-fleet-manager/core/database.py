import json
from pathlib import Path
from typing import Dict, List, Any
import threading
import os


DATA_DIR = Path.cwd() / "pi_fleet_data"

DB_FILE = DATA_DIR / "fleet_data.json"

# Thread lock for database access
db_lock = threading.Lock()

def init_db():
    """Initialize the database file"""
    DATA_DIR.mkdir(exist_ok=True)
    
    if not DB_FILE.exists():
        default_data = {
            "hosts": [],
            "users": [],
            "ssh_keys": [],
            "host_users": []  # Mapping of users to hosts
        }
        save_db(default_data)

def load_db() -> Dict[str, Any]:
    """Load database from JSON file"""
    with db_lock:
        if not DB_FILE.exists():
            init_db()
        with open(DB_FILE, 'r') as f:
            return json.load(f)

def save_db(data: Dict[str, Any]):
    """Save database to JSON file"""
    with db_lock:
        with open(DB_FILE, 'w') as f:
            json.dump(data, f, indent=2)

def get_collection(name: str) -> List[Dict]:
    """Get a collection from the database"""
    db = load_db()
    return db.get(name, [])

def add_item(collection: str, item: Dict) -> Dict:
    """Add item to collection"""
    db = load_db()
    if collection not in db:
        db[collection] = []
    db[collection].append(item)
    save_db(db)
    return item

def update_item(collection: str, item_id: str, updates: Dict) -> bool:
    """Update item in collection"""
    db = load_db()
    if collection not in db:
        return False
    
    for i, item in enumerate(db[collection]):
        if item.get('id') == item_id:
            db[collection][i].update(updates)
            save_db(db)
            return True
    return False

def delete_item(collection: str, item_id: str) -> bool:
    """Delete item from collection"""
    db = load_db()
    if collection not in db:
        return False
    
    original_len = len(db[collection])
    db[collection] = [item for item in db[collection] if item.get('id') != item_id]
    
    if len(db[collection]) < original_len:
        save_db(db)
        return True
    return False

def get_item(collection: str, item_id: str) -> Dict:
    """Get single item from collection"""
    items = get_collection(collection)
    for item in items:
        if item.get('id') == item_id:
            return item
    return None