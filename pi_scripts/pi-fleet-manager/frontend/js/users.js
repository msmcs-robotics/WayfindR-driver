const API_BASE = '/api';

let users = [];
let hosts = [];

async function loadUsers() {
    try {
        const [usersRes, hostsRes] = await Promise.all([
            fetch(`${API_BASE}/users/`),
            fetch(`${API_BASE}/hosts/`)
        ]);
        
        const usersData = await usersRes.json();
        const hostsData = await hostsRes.json();
        
        users = usersData.users;
        hosts = hostsData.hosts;
        
        displayUsers();
    } catch (error) {
        console.error('Error loading users:', error);
        showError('Failed to load users');
    }
}

function displayUsers() {
    const container = document.getElementById('users-list');
    
    if (users.length === 0) {
        container.innerHTML = '<p class="loading">No users configured yet.</p>';
        return;
    }

    container.innerHTML = users.map(user => `
        <div class="list-item">
            <div class="list-item-info">
                <div class="list-item-title">${escapeHtml(user.username)}</div>
                ${user.description ? `<div class="list-item-subtitle">${escapeHtml(user.description)}</div>` : ''}
            </div>
            <div class="list-item-actions">
                <button class="btn btn-small btn-primary" onclick="mapUserToHosts('${user.id}')">Map to Hosts</button>
                <button class="btn btn-small btn-warning" onclick="editUser('${user.id}')">Edit</button>
                <button class="btn btn-small btn-danger" onclick="deleteUser('${user.id}')">Delete</button>
            </div>
        </div>
    `).join('');
}

function showAddUserModal() {
    document.getElementById('addUserForm').reset();
    document.getElementById('addUserModal').classList.add('active');
}

function hideAddUserModal() {
    document.getElementById('addUserModal').classList.remove('active');
}

async function addUser(event) {
    event.preventDefault();
    
    const userData = {
        username: document.getElementById('userUsername').value,
        description: document.getElementById('userDescription').value || null
    };

    try {
        const res = await fetch(`${API_BASE}/users/`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(userData)
        });
        
        if (res.ok) {
            hideAddUserModal();
            await loadUsers();
            showSuccess('User added successfully');
        } else {
            throw new Error('Failed to add user');
        }
    } catch (error) {
        console.error('Error adding user:', error);
        showError('Failed to add user');
    }
}

async function mapUserToHosts(userId) {
    const user = users.find(u => u.id === userId);
    if (!user) return;

    try {
        // Get current user hosts
        const res = await fetch(`${API_BASE}/users/${userId}/hosts`);
        const data = await res.json();
        const currentHostIds = data.hosts.map(h => h.id);

        const content = `
            <h4>Map "${escapeHtml(user.username)}" to Hosts</h4>
            <div class="checkbox-group">
                ${hosts.map(host => `
                    <label class="checkbox-label">
                        <input type="checkbox" value="${host.id}" 
                               ${currentHostIds.includes(host.id) ? 'checked' : ''}>
                        ${escapeHtml(host.name)} (${escapeHtml(host.host)}:${host.port})
                    </label>
                `).join('')}
            </div>
            <div class="button-group" style="margin-top: 1rem;">
                <button class="btn btn-primary" onclick="saveUserHostMapping('${userId}')">Save Mapping</button>
                <button class="btn btn-danger" onclick="hideMapUserModal()">Cancel</button>
            </div>
        `;

        document.getElementById('mapUserContent').innerHTML = content;
        document.getElementById('mapUserModal').classList.add('active');
    } catch (error) {
        console.error('Error loading user hosts:', error);
        showError('Failed to load user hosts');
    }
}

async function saveUserHostMapping(userId) {
    const checkboxes = document.querySelectorAll('#mapUserContent input[type="checkbox"]:checked');
    const hostIds = Array.from(checkboxes).map(cb => cb.value);

    try {
        const res = await fetch(`${API_BASE}/users/${userId}/hosts`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({ user_id: userId, host_ids: hostIds })
        });
        
        if (res.ok) {
            hideMapUserModal();
            showSuccess('User mapping updated successfully');
        } else {
            throw new Error('Failed to update user mapping');
        }
    } catch (error) {
        console.error('Error updating user mapping:', error);
        showError('Failed to update user mapping');
    }
}

function hideMapUserModal() {
    document.getElementById('mapUserModal').classList.remove('active');
}

async function deleteUser(userId) {
    const user = users.find(u => u.id === userId);
    if (!user) return;

    if (!confirm(`Are you sure you want to delete user "${user.username}"?`)) {
        return;
    }

    try {
        const res = await fetch(`${API_BASE}/users/${userId}`, {
            method: 'DELETE'
        });
        
        if (res.ok) {
            await loadUsers();
            showSuccess('User deleted successfully');
        } else {
            throw new Error('Failed to delete user');
        }
    } catch (error) {
        console.error('Error deleting user:', error);
        showError('Failed to delete user');
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

// Load users on page load
document.addEventListener('DOMContentLoaded', loadUsers);