const API_BASE = '/api';

let hosts = [];
let currentEditingHost = null;

async function loadHosts() {
    try {
        const res = await fetch(`${API_BASE}/hosts/`);
        const data = await res.json();
        hosts = data.hosts;
        displayHosts();
        populateJumphostSelect();
    } catch (error) {
        console.error('Error loading hosts:', error);
        showError('Failed to load hosts');
    }
}

function displayHosts() {
    const container = document.getElementById('hosts-list');
    
    if (hosts.length === 0) {
        container.innerHTML = '<p class="loading">No hosts configured yet.</p>';
        return;
    }

    container.innerHTML = hosts.map(host => `
        <div class="list-item">
            <div class="list-item-info">
                <div class="list-item-title">${escapeHtml(host.name)}</div>
                <div class="list-item-subtitle">
                    ${escapeHtml(host.host)}:${host.port}
                    ${host.mac_address ? ` • MAC: ${host.mac_address}` : ''}
                    ${host.jumphost_id ? ' • 🔗 Jumphost' : ''}
                </div>
                ${host.description ? `<div class="list-item-subtitle">${escapeHtml(host.description)}</div>` : ''}
            </div>
            <div class="list-item-actions">
                <button class="btn btn-small btn-primary" onclick="testConnection('${host.id}')">Test</button>
                <button class="btn btn-small btn-warning" onclick="editHost('${host.id}')">Edit</button>
                <button class="btn btn-small btn-danger" onclick="deleteHost('${host.id}')">Delete</button>
            </div>
        </div>
    `).join('');
}

function populateJumphostSelect() {
    const select = document.getElementById('hostJumphost');
    select.innerHTML = '<option value="">None</option>';
    hosts.forEach(host => {
        select.innerHTML += `<option value="${host.id}">${escapeHtml(host.name)}</option>`;
    });
}

function showAddHostModal() {
    currentEditingHost = null;
    document.getElementById('addHostForm').reset();
    document.getElementById('addHostModal').classList.add('active');
}

function hideAddHostModal() {
    document.getElementById('addHostModal').classList.remove('active');
}

async function addHost(event) {
    event.preventDefault();
    
    const hostData = {
        name: document.getElementById('hostName').value,
        host: document.getElementById('hostAddress').value,
        port: parseInt(document.getElementById('hostPort').value) || 22,
        mac_address: document.getElementById('hostMac').value || null,
        jumphost_id: document.getElementById('hostJumphost').value || null,
        description: document.getElementById('hostDescription').value || null
    };

    try {
        const res = await fetch(`${API_BASE}/hosts/`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(hostData)
        });
        
        if (res.ok) {
            hideAddHostModal();
            await loadHosts();
            showSuccess('Host added successfully');
        } else {
            throw new Error('Failed to add host');
        }
    } catch (error) {
        console.error('Error adding host:', error);
        showError('Failed to add host');
    }
}

async function testConnection(hostId) {
    const host = hosts.find(h => h.id === hostId);
    if (!host) return;

    const username = prompt('Enter username for connection test:', 'pi');
    if (!username) return;

    const keyPath = prompt('Enter SSH key path:', '~/.ssh/id_rsa');
    if (!keyPath) return;

    try {
        const res = await fetch(`${API_BASE}/hosts/${hostId}/test?username=${encodeURIComponent(username)}&key_path=${encodeURIComponent(keyPath)}`, {
            method: 'POST'
        });
        const result = await res.json();
        
        if (result.success) {
            showSuccess(`Connection to ${host.name} successful!`);
        } else {
            showError(`Connection to ${host.name} failed: ${result.error}`);
        }
    } catch (error) {
        showError(`Connection test failed: ${error.message}`);
    }
}

async function deleteHost(hostId) {
    const host = hosts.find(h => h.id === hostId);
    if (!host) return;

    if (!confirm(`Are you sure you want to delete host "${host.name}"?`)) {
        return;
    }

    try {
        const res = await fetch(`${API_BASE}/hosts/${hostId}`, {
            method: 'DELETE'
        });
        
        if (res.ok) {
            await loadHosts();
            showSuccess('Host deleted successfully');
        } else {
            throw new Error('Failed to delete host');
        }
    } catch (error) {
        console.error('Error deleting host:', error);
        showError('Failed to delete host');
    }
}

function refreshHosts() {
    loadHosts();
    showSuccess('Hosts refreshed');
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

// Load hosts on page load
document.addEventListener('DOMContentLoaded', loadHosts);