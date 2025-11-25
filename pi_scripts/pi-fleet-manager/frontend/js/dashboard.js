// Dashboard functionality
const API_BASE = '/api';

// Load dashboard data
async function loadDashboard() {
    try {
        // Load stats
        const [hostsRes, usersRes, keysRes] = await Promise.all([
            fetch(`${API_BASE}/hosts/`),
            fetch(`${API_BASE}/users/`),
            fetch(`${API_BASE}/keys/`)
        ]);

        const hosts = await hostsRes.json();
        const users = await usersRes.json();
        const keys = await keysRes.json();

        // Update stats
        document.getElementById('total-hosts').textContent = hosts.hosts.length;
        document.getElementById('total-users').textContent = users.users.length;
        document.getElementById('total-keys').textContent = keys.keys.length;

        // Display hosts
        displayHosts(hosts.hosts);

    } catch (error) {
        console.error('Error loading dashboard:', error);
        showError('Failed to load dashboard data');
    }
}

function displayHosts(hosts) {
    const container = document.getElementById('hosts-list');
    
    if (hosts.length === 0) {
        container.innerHTML = '<p class="loading">No hosts configured yet. <a href="/static/pages/hosts.html">Add your first host</a></p>';
        return;
    }

    container.innerHTML = hosts.slice(0, 5).map(host => `
        <div class="list-item">
            <div class="list-item-info">
                <div class="list-item-title">${escapeHtml(host.name)}</div>
                <div class="list-item-subtitle">${escapeHtml(host.host)}:${host.port}</div>
            </div>
            <div class="list-item-actions">
                <button class="btn btn-small btn-primary" onclick="openTerminal('${host.id}')">Terminal</button>
                <a href="/static/pages/hosts.html" class="btn btn-small btn-primary">Manage</a>
            </div>
        </div>
    `).join('');
}

function openTerminal(hostId) {
    // Open terminal in new tab
    window.open(`/static/pages/terminal.html?host=${hostId}`, '_blank');
}

function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

function showError(message) {
    const container = document.querySelector('.container');
    const alert = document.createElement('div');
    alert.className = 'alert alert-danger';
    alert.textContent = message;
    container.insertBefore(alert, container.firstChild);
    
    setTimeout(() => alert.remove(), 5000);
}

// Load dashboard on page load
document.addEventListener('DOMContentLoaded', loadDashboard);