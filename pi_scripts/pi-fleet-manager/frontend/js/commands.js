const API_BASE = '/api';

let hosts = [];
let keys = [];
let currentStreamConnection = null;

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
        
        populateHostCheckboxes('commandHosts');
        populateKeySelect('commandKey');
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

function getSelectedHosts() {
    const checkboxes = document.querySelectorAll('#commandHosts input[type="checkbox"]:checked');
    return Array.from(checkboxes).map(cb => cb.value);
}

async function executeCommand() {
    const hostIds = getSelectedHosts();
    const username = document.getElementById('commandUsername').value;
    const keyId = document.getElementById('commandKey').value;
    const command = document.getElementById('commandInput').value;

    if (hostIds.length === 0) {
        showError('Please select at least one host');
        return;
    }

    if (!username || !command) {
        showError('Please fill in all required fields');
        return;
    }

    const commandData = {
        command: command,
        host_ids: hostIds,
        username: username,
        key_id: keyId
    };

    try {
        const res = await fetch(`${API_BASE}/commands/execute`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(commandData)
        });
        
        const results = await res.json();
        displayCommandResults(results.results);
    } catch (error) {
        console.error('Error executing command:', error);
        showError('Failed to execute command');
    }
}

function executeCommandStream() {
    const hostIds = getSelectedHosts();
    const username = document.getElementById('commandUsername').value;
    const keyId = document.getElementById('commandKey').value;
    const command = document.getElementById('commandInput').value;

    if (hostIds.length === 0) {
        showError('Please select at least one host');
        return;
    }

    if (!username || !command) {
        showError('Please fill in all required fields');
        return;
    }

    // Create WebSocket connection for streaming
    const connectionId = 'cmd_' + Date.now();
    currentStreamConnection = new WebSocket(`ws://${window.location.host}/api/commands/stream/${connectionId}`);
    
    // Clear previous results
    const resultsContainer = document.getElementById('command-results');
    resultsContainer.innerHTML = '';
    
    // Create output boxes for each host
    hostIds.forEach(hostId => {
        const host = hosts.find(h => h.id === hostId);
        if (host) {
            const outputBox = document.createElement('div');
            outputBox.className = 'output-box';
            outputBox.id = `output-${hostId}`;
            outputBox.innerHTML = `
                <div class="output-box-header">${escapeHtml(host.name)}</div>
                <pre></pre>
            `;
            resultsContainer.appendChild(outputBox);
        }
    });

    currentStreamConnection.onopen = () => {
        // Send command data
        currentStreamConnection.send(JSON.stringify({
            command: command,
            host_ids: hostIds,
            username: username,
            key_id: keyId
        }));
    };

    currentStreamConnection.onmessage = (event) => {
        const data = JSON.parse(event.data);
        
        if (data.host_id) {
            const outputBox = document.getElementById(`output-${data.host_id}`);
            if (outputBox) {
                const pre = outputBox.querySelector('pre');
                
                if (data.status === 'connecting') {
                    pre.textContent += `\n🔗 Connecting to ${data.host_name}...\n`;
                } else if (data.status === 'executing') {
                    pre.textContent += `🚀 Executing command...\n`;
                } else if (data.status === 'complete') {
                    pre.textContent += `✅ Command completed (exit code: ${data.exit_code})\n`;
                    if (data.stdout) pre.textContent += `\nSTDOUT:\n${data.stdout}\n`;
                    if (data.stderr) pre.textContent += `\nSTDERR:\n${data.stderr}\n`;
                } else if (data.status === 'error') {
                    pre.textContent += `❌ Error: ${data.error}\n`;
                }
                
                outputBox.scrollTop = outputBox.scrollHeight;
            }
        }
        
        if (data.status === 'all_complete') {
            showSuccess('All commands completed');
            currentStreamConnection.close();
            currentStreamConnection = null;
        }
    };

    currentStreamConnection.onerror = (error) => {
        console.error('WebSocket error:', error);
        showError('WebSocket connection failed');
        currentStreamConnection = null;
    };
}

function displayCommandResults(results) {
    const container = document.getElementById('command-results');
    container.innerHTML = '';

    for (const [hostId, result] of Object.entries(results)) {
        const host = hosts.find(h => h.id === hostId) || { name: hostId };
        
        const outputBox = document.createElement('div');
        outputBox.className = 'output-box';
        
        let content = `Host: ${escapeHtml(result.host_name || host.name)}\n`;
        content += `Address: ${escapeHtml(result.host || 'Unknown')}\n\n`;
        
        if (result.error) {
            content += `❌ ERROR: ${escapeHtml(result.error)}\n`;
        } else {
            if (result.stdout) {
                content += `STDOUT:\n${escapeHtml(result.stdout)}\n`;
            }
            if (result.stderr) {
                content += `STDERR:\n${escapeHtml(result.stderr)}\n`;
            }
            content += `\nExit Code: ${result.exit_code}\n`;
        }
        
        outputBox.innerHTML = `
            <div class="output-box-header">${escapeHtml(result.host_name || host.name)}</div>
            <pre>${content}</pre>
        `;
        
        container.appendChild(outputBox);
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

// Load data on page load
document.addEventListener('DOMContentLoaded', loadData);