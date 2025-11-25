const API_BASE = '/api';

let terminal;
let fitAddon;
let ws;
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
        
        populateHostSelect();
        populateKeySelect();
        
        // Initialize terminal
        initializeTerminal();
    } catch (error) {
        console.error('Error loading data:', error);
        showError('Failed to load hosts and keys');
    }
}

function populateHostSelect() {
    const select = document.getElementById('terminalHost');
    select.innerHTML = hosts.map(host => `
        <option value="${host.id}">${escapeHtml(host.name)} (${escapeHtml(host.host)})</option>
    `).join('');
}

function populateKeySelect() {
    const select = document.getElementById('terminalKey');
    select.innerHTML = keys.map(key => `
        <option value="${key.id}">${escapeHtml(key.name)}</option>
    `).join('');
}

function initializeTerminal() {
    terminal = new Terminal({
        cursorBlink: true,
        theme: {
            background: '#1e1e1e',
            foreground: '#ffffff'
        }
    });
    
    fitAddon = new FitAddon.FitAddon();
    terminal.loadAddon(fitAddon);
    
    const terminalContainer = document.getElementById('terminal-container');
    terminal.open(terminalContainer);
    fitAddon.fit();
    
    // Handle terminal input
    terminal.onData(data => {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({
                type: 'input',
                data: data
            }));
        }
    });
    
    // Handle window resize
    window.addEventListener('resize', () => {
        fitAddon.fit();
        if (ws && ws.readyState === WebSocket.OPEN) {
            const dimensions = fitAddon.proposeDimensions();
            ws.send(JSON.stringify({
                type: 'resize',
                width: dimensions.cols,
                height: dimensions.rows
            }));
        }
    });
}

function connectTerminal() {
    const hostId = document.getElementById('terminalHost').value;
    const username = document.getElementById('terminalUsername').value;
    const keyId = document.getElementById('terminalKey').value;
    
    if (!hostId || !username || !keyId) {
        showError('Please fill in all connection parameters');
        return;
    }
    
    const sessionId = 'term_' + Date.now();
    
    // Clear terminal
    terminal.clear();
    terminal.write('🔗 Connecting to host...\r\n');
    
    // Create WebSocket connection
    ws = new WebSocket(`ws://${window.location.host}/api/terminal/ws/${sessionId}`);
    
    ws.onopen = () => {
        terminal.write('✅ WebSocket connected, establishing SSH session...\r\n');
        
        // Send connection parameters
        ws.send(JSON.stringify({
            host_id: hostId,
            username: username,
            key_id: keyId
        }));
    };
    
    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        
        if (data.type === 'output') {
            terminal.write(data.data);
        } else if (data.status === 'connected') {
            terminal.write('✅ SSH session established!\r\n');
            terminal.write('You can now interact with the remote host.\r\n\r\n');
        } else if (data.error) {
            terminal.write(`❌ Error: ${data.error}\r\n`);
        }
    };
    
    ws.onclose = () => {
        terminal.write('\r\n\r\n🔌 SSH session closed\r\n');
    };
    
    ws.onerror = (error) => {
        terminal.write(`❌ WebSocket error: ${error}\r\n`);
    };
}

function disconnectTerminal() {
    if (ws) {
        ws.close();
        ws = null;
    }
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