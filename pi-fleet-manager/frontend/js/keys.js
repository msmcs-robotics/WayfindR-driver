const API_BASE = '/api';

let keys = [];
let hosts = [];

async function loadKeys() {
    try {
        const res = await fetch(`${API_BASE}/keys/`);
        const data = await res.json();
        keys = data.keys;
        displayKeys();
        
        // Also load hosts for copy functionality
        const hostsRes = await fetch(`${API_BASE}/hosts/`);
        const hostsData = await hostsRes.json();
        hosts = hostsData.hosts;
    } catch (error) {
        console.error('Error loading keys:', error);
        showError('Failed to load SSH keys');
    }
}

function displayKeys() {
    const container = document.getElementById('keys-list');
    
    if (keys.length === 0) {
        container.innerHTML = '<p class="loading">No SSH keys registered yet.</p>';
        return;
    }

    container.innerHTML = keys.map(key => `
        <div class="list-item">
            <div class="list-item-info">
                <div class="list-item-title">${escapeHtml(key.name)}</div>
                <div class="list-item-subtitle">
                    Private: ${escapeHtml(key.private_key)}<br>
                    Public: ${escapeHtml(key.public_key || 'Not found')}
                </div>
            </div>
            <div class="list-item-actions">
                <button class="btn btn-small btn-primary" onclick="addKeyToAgent('${key.id}')">Add to Agent</button>
                <button class="btn btn-small btn-success" onclick="copyKeyToHost('${key.id}')">Copy to Host</button>
                <button class="btn btn-small btn-danger" onclick="deleteKey('${key.id}')">Delete</button>
            </div>
        </div>
    `).join('');
}

function showGenerateKeyModal() {
    document.getElementById('generateKeyForm').reset();
    document.getElementById('generateKeyModal').classList.add('active');
}

function hideGenerateKeyModal() {
    document.getElementById('generateKeyModal').classList.remove('active');
}

async function generateKey(event) {
    event.preventDefault();
    
    const keyData = {
        name: document.getElementById('keyName').value,
        key_size: parseInt(document.getElementById('keySize').value)
    };

    try {
        const res = await fetch(`${API_BASE}/keys/generate`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(keyData)
        });
        
        if (res.ok) {
            hideGenerateKeyModal();
            await loadKeys();
            showSuccess('SSH key generated successfully');
        } else {
            throw new Error('Failed to generate key');
        }
    } catch (error) {
        console.error('Error generating key:', error);
        showError('Failed to generate SSH key');
    }
}

function showRegisterKeyModal() {
    document.getElementById('registerKeyForm').reset();
    document.getElementById('registerKeyModal').classList.add('active');
}

function hideRegisterKeyModal() {
    document.getElementById('registerKeyModal').classList.remove('active');
}

async function registerKey(event) {
    event.preventDefault();
    
    const name = document.getElementById('registerKeyName').value;
    const privateKeyPath = document.getElementById('registerKeyPath').value;

    try {
        const res = await fetch(`${API_BASE}/keys/register?name=${encodeURIComponent(name)}&private_key_path=${encodeURIComponent(privateKeyPath)}`, {
            method: 'POST'
        });
        
        if (res.ok) {
            hideRegisterKeyModal();
            await loadKeys();
            showSuccess('SSH key registered successfully');
        } else {
            throw new Error('Failed to register key');
        }
    } catch (error) {
        console.error('Error registering key:', error);
        showError('Failed to register SSH key');
    }
}

async function copyKeyToHost(keyId) {
    const key = keys.find(k => k.id === keyId);
    if (!key) return;

    const content = `
        <h4>Copy "${escapeHtml(key.name)}" to Host</h4>
        <div class="form-group">
            <label for="copyHostSelect">Select Host</label>
            <select id="copyHostSelect" class="form-control">
                ${hosts.map(host => `
                    <option value="${host.id}">${escapeHtml(host.name)} (${escapeHtml(host.host)})</option>
                `).join('')}
            </select>
        </div>
        <div class="form-group">
            <label for="copyUsername">Username</label>
            <input type="text" id="copyUsername" value="pi" class="form-control">
        </div>
        <div class="form-group">
            <label for="copyPassword">Password (optional, for initial setup)</label>
            <input type="password" id="copyPassword" class="form-control">
        </div>
        <div class="button-group">
            <button class="btn btn-primary" onclick="performKeyCopy('${keyId}')">Copy Key</button>
            <button class="btn btn-danger" onclick="hideCopyKeyModal()">Cancel</button>
        </div>
    `;

    document.getElementById('copyKeyContent').innerHTML = content;
    document.getElementById('copyKeyModal').classList.add('active');
}

async function performKeyCopy(keyId) {
    const hostId = document.getElementById('copyHostSelect').value;
    const username = document.getElementById('copyUsername').value;
    const password = document.getElementById('copyPassword').value;

    const copyData = {
        key_id: keyId,
        host_id: hostId,
        username: username,
        password: password || null
    };

    try {
        const res = await fetch(`${API_BASE}/keys/copy-to-host`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(copyData)
        });
        
        if (res.ok) {
            hideCopyKeyModal();
            showSuccess('SSH key copied to host successfully');
        } else {
            throw new Error('Failed to copy key to host');
        }
    } catch (error) {
        console.error('Error copying key:', error);
        showError('Failed to copy SSH key to host');
    }
}

function hideCopyKeyModal() {
    document.getElementById('copyKeyModal').classList.remove('active');
}

async function addKeyToAgent(keyId) {
    try {
        const res = await fetch(`${API_BASE}/keys/${keyId}/add-to-agent`, {
            method: 'POST'
        });
        
        if (res.ok) {
            showSuccess('SSH key added to agent successfully');
        } else {
            throw new Error('Failed to add key to agent');
        }
    } catch (error) {
        console.error('Error adding key to agent:', error);
        showError('Failed to add SSH key to agent');
    }
}

async function setupSSHAgent() {
    try {
        const res = await fetch(`${API_BASE}/keys/setup-agent`, {
            method: 'POST'
        });
        const result = await res.json();
        
        if (res.ok) {
            let message = 'SSH agent setup completed. ';
            if (result.agent_running) message += 'Agent is running. ';
            if (result.rc_files_updated) message += 'Shell RC files updated.';
            showSuccess(message);
        } else {
            throw new Error('Failed to setup SSH agent');
        }
    } catch (error) {
        console.error('Error setting up SSH agent:', error);
        showError('Failed to setup SSH agent');
    }
}

async function scanFilesystemKeys() {
    try {
        const res = await fetch(`${API_BASE}/keys/scan-filesystem`);
        const data = await res.json();
        
        if (res.ok) {
            showSuccess(`Found ${data.keys.length} SSH keys in filesystem`);
            // Optionally auto-register found keys
        } else {
            throw new Error('Failed to scan filesystem');
        }
    } catch (error) {
        console.error('Error scanning filesystem:', error);
        showError('Failed to scan filesystem for SSH keys');
    }
}

async function deleteKey(keyId) {
    const key = keys.find(k => k.id === keyId);
    if (!key) return;

    if (!confirm(`Are you sure you want to delete key "${key.name}"? This will only remove the registration, not the key files.`)) {
        return;
    }

    try {
        const res = await fetch(`${API_BASE}/keys/${keyId}`, {
            method: 'DELETE'
        });
        
        if (res.ok) {
            await loadKeys();
            showSuccess('SSH key registration deleted successfully');
        } else {
            throw new Error('Failed to delete key');
        }
    } catch (error) {
        console.error('Error deleting key:', error);
        showError('Failed to delete SSH key registration');
    }
}

function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

function showSuccess(message) {
    const container = document.querySelector('.container');
    const alert = document.createElement('div');
    alert.className = 'alert alert-success';
    alert.textContent = message;
    container.insertBefore(alert, container.firstChild);
    
    setTimeout(() => alert.remove(), 5000);
}

function showError(message) {
    const container = document.querySelector('.container');
    const alert = document.createElement('div');
    alert.className = 'alert alert-danger';
    alert.textContent = message;
    container.insertBefore(alert, container.firstChild);
    
    setTimeout(() => alert.remove(), 5000);
}

// Load keys on page load
document.addEventListener('DOMContentLoaded', loadKeys);