# Complete Directory Structure

Create this exact directory structure for the Pi Fleet Manager:

```
pi-fleet-manager/
│
├── main.py                          # FastAPI application entry point
├── requirements.txt                 # Python dependencies
├── README.md                        # Setup and usage documentation
│
├── api/                             # API endpoints (create this folder)
│   ├── __init__.py                 # Empty file to make it a package
│   ├── hosts.py                    # Host management endpoints
│   ├── users.py                    # User management endpoints
│   ├── ssh_keys.py                 # SSH key management endpoints
│   ├── commands.py                 # Command execution endpoints
│   ├── files.py                    # File transfer endpoints
│   └── terminal.py                 # Interactive terminal WebSocket
│
├── core/                            # Core business logic (create this folder)
│   ├── __init__.py                 # Empty file to make it a package
│   ├── database.py                 # JSON database operations
│   ├── ssh_manager.py              # SSH connection handling
│   └── key_manager.py              # SSH key generation and management
│
└── frontend/                        # Web interface (create this folder)
    ├── index.html                   # Main dashboard page
    │
    ├── css/                         # Stylesheets folder
    │   └── style.css                # Main stylesheet
    │
    ├── js/                          # JavaScript folder
    │   ├── dashboard.js             # Dashboard functionality
    │   ├── hosts.js                 # Host management UI
    │   ├── users.js                 # User management UI
    │   ├── keys.js                  # SSH key management UI
    │   ├── files.js                 # File upload UI
    │   ├── commands.js              # Command execution UI
    │   └── terminal.js              # Terminal emulation
    │
    └── pages/                       # Additional HTML pages
        ├── hosts.html               # Host management page
        ├── users.html               # User management page
        ├── keys.html                # SSH key management page
        ├── files.html               # File upload page
        ├── commands.html            # Command execution page
        └── terminal.html            # Interactive terminal page
```

## Setup Steps

### 1. Create the directory structure:

```bash
mkdir -p pi-fleet-manager/{api,core,frontend/{css,js,pages}}
cd pi-fleet-manager

# Create __init__.py files
touch api/__init__.py
touch core/__init__.py
```

### 2. Create all the Python files:
- Copy the content provided for each `.py` file into its respective location
- Make sure all files in `api/` and `core/` directories are created

### 3. Create all the frontend files:
- Copy `index.html` to `frontend/`
- Copy `style.css` to `frontend/css/`
- Copy `dashboard.js` to `frontend/js/`
- Create additional HTML pages in `frontend/pages/` (minimal examples provided below)

### 4. Install dependencies:

```bash
pip install -r requirements.txt
```

### 5. Run the application:

```bash
python main.py
```

The application will start on `http://localhost:8000`

## Data Storage

The application automatically creates:
- `~/.pi_fleet/` directory for data storage
- `~/.pi_fleet/fleet_data.json` for all configuration

## Minimal Frontend Pages

Since I provided the main files, here are minimal templates for the additional pages. Each should have:

**frontend/pages/hosts.html** (example):
```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Hosts - Pi Fleet Manager</title>
    <link rel="stylesheet" href="/static/css/style.css">
</head>
<body>
    <nav class="navbar">
        <div class="nav-brand">🥧 Pi Fleet Manager</div>
        <ul class="nav-menu">
            <li><a href="/">Dashboard</a></li>
            <li><a href="/static/pages/hosts.html" class="active">Hosts</a></li>
            <li><a href="/static/pages/users.html">Users</a></li>
            <li><a href="/static/pages/keys.html">SSH Keys</a></li>
            <li><a href="/static/pages/files.html">Files</a></li>
            <li><a href="/static/pages/commands.html">Commands</a></li>
        </ul>
    </nav>
    <div class="container">
        <h1>Host Management</h1>
        <!-- Add your host management UI here -->
        <button class="btn btn-primary" onclick="showAddHostModal()">Add Host</button>
        <div id="hosts-container"></div>
    </div>
    <script src="/static/js/hosts.js"></script>
</body>
</html>
```

Follow the same pattern for other pages (users.html, keys.html, etc.).

## JavaScript Files

Create corresponding JS files that make API calls:

**frontend/js/hosts.js** (example):
```javascript
const API_BASE = '/api';

async function loadHosts() {
    const res = await fetch(`${API_BASE}/hosts/`);
    const data = await res.json();
    displayHosts(data.hosts);
}

async function addHost(hostData) {
    const res = await fetch(`${API_BASE}/hosts/`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(hostData)
    });
    return await res.json();
}

// Add more functions as needed
document.addEventListener('DOMContentLoaded', loadHosts);
```

Follow similar patterns for other JavaScript files.

## Quick Start

After setting up the structure:

```bash
# 1. Create directories
mkdir -p pi-fleet-manager/{api,core,frontend/{css,js,pages}}

# 2. Copy all files to their locations

# 3. Create __init__.py files
touch pi-fleet-manager/api/__init__.py
touch pi-fleet-manager/core/__init__.py

# 4. Install and run
cd pi-fleet-manager
pip install -r requirements.txt
python main.py
```

Visit `http://localhost:8000` and start managing your Raspberry Pi fleet!