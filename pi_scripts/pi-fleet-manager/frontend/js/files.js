const API_BASE = '/api';

let hosts = [];
let keys = [];

async function loadData() {
    try {
        const [hostsRes, keysRes] = await Promise.all([
            fetch(`${API_BASE}/hosts/`),
            fetch(`${API_BASE}/keys/`)
        ]);
        
        const hostsData = await hostsRes.json();
        const keysData = await keysRes.json();
        
        hosts = hostsData.hosts;
        keys = keysData.keys;
        
        populateHostCheckboxes('setupHosts');
        populateHostCheckboxes('transferHosts');
        populateKeySelect('setupKey');
        populateKeySelect('transferKey');
    } catch (error) {
        console.error('Error loading data:', error);
        showError('Failed to load hosts and keys');
    }
}

function populateHostCheckboxes(containerId) {
    const container = document.getElementById(containerId);
    container.innerHTML = hosts.map(host => `
        <label class="checkbox-label">
            <input type="checkbox" value="${host.id}">
            ${escapeHtml(host.name)} (${escapeHtml(host.host)})
        </label>
    `).join('');
}

function populateKeySelect(selectId) {
    const select = document.getElementById(selectId);
    select.innerHTML = keys.map(key => `
        <option value="${key.id}">${escapeHtml(key.name)}</option>
    `).join('');
}

function getSelectedHosts(containerId) {
    const checkboxes = document.querySelectorAll(`#${containerId} input[type="checkbox"]:checked`);
    return Array.from(checkboxes).map(cb => cb.value);
}

async function setupBakery() {
    const hostIds = getSelectedHosts('setupHosts');
    const username = document.getElementById('setupUsername').value;
    const keyId = document.getElementById('setupKey').value;

    if (hostIds.length === 0) {
        showError('Please select at least one host');
        return;
    }

    if (!username) {
        showError('Please enter a username');
        return;
    }

    const setupData = {
        host_ids: hostIds,
        username: username,
        key_id: keyId
    };

    try {
        const res = await fetch(`${API_BASE}/files/setup-bakery`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(setupData)
        });
        
        const results = await res.json();
        displayTransferResults(results.results, 'Bakery Setup');
    } catch (error) {
        console.error('Error setting up bakery:', error);
        showError('Failed to setup bakery folders');
    }
}

async function uploadAndTransfer() {
    const hostIds = getSelectedHosts('transferHosts');
    const username = document.getElementById('transferUsername').value;
    const keyId = document.getElementById('transferKey').value;
    const remotePath = document.getElementById('remotePath').value;
    const fileInput = document.getElementById('fileUpload');
    
    if (hostIds.length === 0) {
        showError('Please select at least one host');
        return;
    }

    if (!username) {
        showError('Please enter a username');
        return;
    }

    if (!fileInput.files.length) {
        showError('Please select a file to upload');
        return;
    }

    const formData = new FormData();
    formData.append('file', fileInput.files[0]);
    formData.append('host_ids', hostIds.join(','));
    formData.append('username', username);
    formData.append('key_id', keyId);
    formData.append('remote_path', remotePath);

    try {
        const res = await fetch(`${API_BASE}/files/upload`, {
            method: 'POST',
            body: formData
        });
        
        const results = await res.json();
        displayTransferResults(results.results, 'File Transfer');
        fileInput.value = ''; // Clear file input
    } catch (error) {
        console.error('Error uploading file:', error);
        showError('Failed to upload and transfer file');
    }
}

function displayTransferResults(results, title) {
    const container = document.getElementById('transfer-results');
    
    let html = `<h3>${title} Results</h3>`;
    
    for (const [hostId, result] of Object.entries(results)) {
        const host = hosts.find(h => h.id === hostId) || { name: hostId };
        
        html += `
            <div class="list-item">
                <div class="list-item-info">
                    <div class="list-item-title">${escapeHtml(host.name)}</div>
                    <div class="list-item-subtitle">
                        ${result.success ? '✅ Success' : '❌ Failed'}
                        ${result.error ? `: ${escapeHtml(result.error)}` : ''}
                        ${result.remote_path ? ` → ${escapeHtml(result.remote_path)}` : ''}
                    </div>
                </div>
            </div>
        `;
    }
    
    container.innerHTML = html;
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

// Load data on page load
document.addEventListener('DOMContentLoaded', loadData);