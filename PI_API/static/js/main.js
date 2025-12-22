/**
 * WayfindR Dashboard - Main JavaScript
 *
 * Handles:
 * - WebSocket connection for real-time control and telemetry
 * - Keyboard input with combination support
 * - Button controls for direction pad
 * - Pattern execution
 * - Navigation controls
 */

class RobotDashboard {
    constructor() {
        this.ws = null;
        this.connected = false;
        this.pressedKeys = new Set();
        this.speedMultiplier = 0.5;
        this.activeButtons = new Set();

        this.init();
    }

    init() {
        this.connectWebSocket();
        this.setupKeyboardControls();
        this.setupButtonControls();
        this.setupSpeedSlider();
        this.setupPatternControls();
        this.setupNavigationControls();
        this.setupEmergencyStop();
        this.loadWaypoints();
    }

    // ==========================================
    // WebSocket Connection
    // ==========================================

    connectWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws`;

        console.log('Connecting to WebSocket:', wsUrl);
        this.ws = new WebSocket(wsUrl);

        this.ws.onopen = () => {
            console.log('WebSocket connected');
            this.connected = true;
            this.updateConnectionStatus(true);
        };

        this.ws.onclose = () => {
            console.log('WebSocket disconnected');
            this.connected = false;
            this.updateConnectionStatus(false);

            // Reconnect after 2 seconds
            setTimeout(() => this.connectWebSocket(), 2000);
        };

        this.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
        };

        this.ws.onmessage = (event) => {
            const msg = JSON.parse(event.data);
            this.handleMessage(msg);
        };
    }

    handleMessage(msg) {
        switch (msg.type) {
            case 'telemetry':
                this.updateTelemetry(msg.data);
                break;
            case 'ack':
                // Command acknowledged
                break;
            case 'state':
                this.updateTelemetry(msg.data);
                break;
            case 'error':
                console.error('Server error:', msg.message);
                break;
        }
    }

    send(data) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(data));
        }
    }

    updateConnectionStatus(connected) {
        const statusEl = document.getElementById('connection-status');
        if (statusEl) {
            statusEl.textContent = connected ? 'Connected' : 'Disconnected';
            statusEl.className = 'status ' + (connected ? 'connected' : 'disconnected');
        }
    }

    // ==========================================
    // Telemetry Display
    // ==========================================

    updateTelemetry(data) {
        // Mode
        const modeEl = document.getElementById('robot-mode');
        if (modeEl && data.mode) {
            modeEl.textContent = data.mode.toUpperCase();
        }

        // Battery
        const batteryEl = document.getElementById('battery-status');
        if (batteryEl && data.sensors) {
            batteryEl.textContent = `Battery: ${data.sensors.battery_percentage.toFixed(0)}%`;
        }

        // Position
        const posEl = document.getElementById('position-display');
        if (posEl && data.position) {
            posEl.textContent = `(${data.position.x.toFixed(2)}, ${data.position.y.toFixed(2)})`;
        }

        // Heading
        const headingEl = document.getElementById('heading-display');
        if (headingEl && data.position) {
            headingEl.textContent = `${data.position.yaw.toFixed(1)}°`;
        }

        // Speed
        const speedEl = document.getElementById('speed-display');
        if (speedEl && data.velocity) {
            const speed = Math.sqrt(
                data.velocity.linear_x ** 2 +
                data.velocity.linear_y ** 2
            );
            speedEl.textContent = `${speed.toFixed(2)} m/s`;
        }

        // Throttle & Steering
        if (data.drive) {
            const throttleEl = document.getElementById('throttle-display');
            if (throttleEl) {
                throttleEl.textContent = `${(data.drive.throttle * 100).toFixed(0)}%`;
            }

            const steeringEl = document.getElementById('steering-display');
            if (steeringEl) {
                steeringEl.textContent = `${(data.drive.steering * 100).toFixed(0)}%`;
            }

            // Motor bars
            this.updateMotorBar('motor-lf', data.drive.left_front?.speed || 0);
            this.updateMotorBar('motor-rf', data.drive.right_front?.speed || 0);
            this.updateMotorBar('motor-lr', data.drive.left_rear?.speed || 0);
            this.updateMotorBar('motor-rr', data.drive.right_rear?.speed || 0);
        }

        // Uptime
        const uptimeEl = document.getElementById('uptime-display');
        if (uptimeEl && data.uptime !== undefined) {
            uptimeEl.textContent = this.formatUptime(data.uptime);
        }
    }

    updateMotorBar(id, speed) {
        const motor = document.getElementById(id);
        if (motor) {
            const fill = motor.querySelector('.motor-fill');
            if (fill) {
                fill.style.width = `${Math.abs(speed) * 100}%`;
                fill.style.backgroundColor = speed >= 0 ? '#4ecca3' : '#e74c3c';
            }
        }
    }

    formatUptime(seconds) {
        if (seconds < 60) return `${Math.floor(seconds)}s`;
        if (seconds < 3600) {
            const m = Math.floor(seconds / 60);
            const s = Math.floor(seconds % 60);
            return `${m}m ${s}s`;
        }
        const h = Math.floor(seconds / 3600);
        const m = Math.floor((seconds % 3600) / 60);
        return `${h}h ${m}m`;
    }

    // ==========================================
    // Keyboard Controls
    // ==========================================

    setupKeyboardControls() {
        document.addEventListener('keydown', (e) => {
            // Ignore if typing in input
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') {
                return;
            }

            const key = this.normalizeKey(e.key);
            if (!this.pressedKeys.has(key)) {
                this.pressedKeys.add(key);
                this.send({ type: 'keydown', key: key });
                this.updateMovement();
            }

            // Prevent default for arrow keys and space
            if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(e.key)) {
                e.preventDefault();
            }
        });

        document.addEventListener('keyup', (e) => {
            const key = this.normalizeKey(e.key);
            this.pressedKeys.delete(key);
            this.send({ type: 'keyup', key: key });
            this.updateMovement();
        });

        // Clear keys on focus loss
        window.addEventListener('blur', () => {
            this.pressedKeys.clear();
            this.send({ type: 'stop' });
        });
    }

    normalizeKey(key) {
        const keyMap = {
            'ArrowUp': 'w',
            'ArrowDown': 's',
            'ArrowLeft': 'a',
            'ArrowRight': 'd',
            ' ': 'space'
        };
        return keyMap[key] || key.toLowerCase();
    }

    updateMovement() {
        let throttle = 0;
        let steering = 0;

        // Forward/backward
        if (this.pressedKeys.has('w')) throttle = 1;
        else if (this.pressedKeys.has('s')) throttle = -1;

        // Left/right
        if (this.pressedKeys.has('a')) steering = -1;
        else if (this.pressedKeys.has('d')) steering = 1;

        // Apply speed multiplier
        throttle *= this.speedMultiplier;
        steering *= this.speedMultiplier;

        // Space = stop
        if (this.pressedKeys.has('space')) {
            this.send({ type: 'stop' });
            return;
        }

        // Rotate in place (q/e)
        if (this.pressedKeys.has('q') && throttle === 0) {
            this.send({ type: 'move', throttle: 0, steering: -this.speedMultiplier });
            return;
        }
        if (this.pressedKeys.has('e') && throttle === 0) {
            this.send({ type: 'move', throttle: 0, steering: this.speedMultiplier });
            return;
        }

        if (throttle !== 0 || steering !== 0) {
            this.send({ type: 'move', throttle: throttle, steering: steering });
        } else {
            this.send({ type: 'stop' });
        }

        // Update button visual states
        this.updateButtonStates();
    }

    updateButtonStates() {
        // Clear all active states
        document.querySelectorAll('.dir-btn').forEach(btn => btn.classList.remove('active'));

        // Highlight based on pressed keys
        if (this.pressedKeys.has('w') && this.pressedKeys.has('a')) {
            document.getElementById('btn-forward-left')?.classList.add('active');
        } else if (this.pressedKeys.has('w') && this.pressedKeys.has('d')) {
            document.getElementById('btn-forward-right')?.classList.add('active');
        } else if (this.pressedKeys.has('w')) {
            document.getElementById('btn-forward')?.classList.add('active');
        } else if (this.pressedKeys.has('s') && this.pressedKeys.has('a')) {
            document.getElementById('btn-backward-left')?.classList.add('active');
        } else if (this.pressedKeys.has('s') && this.pressedKeys.has('d')) {
            document.getElementById('btn-backward-right')?.classList.add('active');
        } else if (this.pressedKeys.has('s')) {
            document.getElementById('btn-backward')?.classList.add('active');
        } else if (this.pressedKeys.has('a')) {
            document.getElementById('btn-rotate-left')?.classList.add('active');
        } else if (this.pressedKeys.has('d')) {
            document.getElementById('btn-rotate-right')?.classList.add('active');
        }
    }

    // ==========================================
    // Button Controls
    // ==========================================

    setupButtonControls() {
        // Direction buttons
        document.querySelectorAll('.dir-btn[data-throttle]').forEach(btn => {
            const throttle = parseFloat(btn.dataset.throttle);
            const steering = parseFloat(btn.dataset.steering);

            btn.addEventListener('mousedown', () => {
                this.send({
                    type: 'move',
                    throttle: throttle * this.speedMultiplier,
                    steering: steering * this.speedMultiplier
                });
                btn.classList.add('active');
            });

            btn.addEventListener('mouseup', () => {
                this.send({ type: 'stop' });
                btn.classList.remove('active');
            });

            btn.addEventListener('mouseleave', () => {
                if (btn.classList.contains('active')) {
                    this.send({ type: 'stop' });
                    btn.classList.remove('active');
                }
            });

            // Touch support
            btn.addEventListener('touchstart', (e) => {
                e.preventDefault();
                this.send({
                    type: 'move',
                    throttle: throttle * this.speedMultiplier,
                    steering: steering * this.speedMultiplier
                });
                btn.classList.add('active');
            });

            btn.addEventListener('touchend', () => {
                this.send({ type: 'stop' });
                btn.classList.remove('active');
            });
        });

        // Rotate buttons
        document.querySelectorAll('.dir-btn[data-rotate]').forEach(btn => {
            const rotate = parseFloat(btn.dataset.rotate);

            btn.addEventListener('mousedown', () => {
                this.send({
                    type: 'move',
                    throttle: 0,
                    steering: rotate * this.speedMultiplier
                });
                btn.classList.add('active');
            });

            btn.addEventListener('mouseup', () => {
                this.send({ type: 'stop' });
                btn.classList.remove('active');
            });

            btn.addEventListener('mouseleave', () => {
                if (btn.classList.contains('active')) {
                    this.send({ type: 'stop' });
                    btn.classList.remove('active');
                }
            });
        });

        // Stop button
        const stopBtn = document.getElementById('btn-stop');
        if (stopBtn) {
            stopBtn.addEventListener('click', () => {
                this.send({ type: 'stop' });
            });
        }
    }

    // ==========================================
    // Speed Slider
    // ==========================================

    setupSpeedSlider() {
        const slider = document.getElementById('speed-slider');
        const valueDisplay = document.getElementById('speed-value');

        if (slider) {
            slider.addEventListener('input', () => {
                this.speedMultiplier = slider.value / 100;
                if (valueDisplay) {
                    valueDisplay.textContent = slider.value;
                }
            });
        }
    }

    // ==========================================
    // Pattern Controls
    // ==========================================

    setupPatternControls() {
        document.querySelectorAll('.pattern-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const pattern = btn.dataset.pattern;
                const size = parseFloat(document.getElementById('pattern-size')?.value || 1.0);

                this.executePattern(pattern, size);
            });
        });

        const stopPatternBtn = document.getElementById('stop-pattern');
        if (stopPatternBtn) {
            stopPatternBtn.addEventListener('click', () => {
                this.stopPattern();
            });
        }
    }

    async executePattern(pattern, size) {
        try {
            const response = await fetch('/api/patterns/execute', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    pattern: pattern,
                    size: size,
                    speed: this.speedMultiplier,
                    repetitions: 1
                })
            });

            const result = await response.json();
            console.log('Pattern started:', result);
        } catch (error) {
            console.error('Pattern error:', error);
        }
    }

    async stopPattern() {
        try {
            await fetch('/api/patterns/stop', { method: 'POST' });
            console.log('Pattern stopped');
        } catch (error) {
            console.error('Stop pattern error:', error);
        }
    }

    // ==========================================
    // Navigation Controls
    // ==========================================

    setupNavigationControls() {
        const saveBtn = document.getElementById('save-waypoint');
        if (saveBtn) {
            saveBtn.addEventListener('click', () => {
                const name = prompt('Enter waypoint name:');
                if (name) {
                    this.saveWaypoint(name);
                }
            });
        }

        const cancelBtn = document.getElementById('cancel-nav');
        if (cancelBtn) {
            cancelBtn.addEventListener('click', () => {
                this.cancelNavigation();
            });
        }
    }

    async loadWaypoints() {
        try {
            const response = await fetch('/api/navigation/waypoints');
            const data = await response.json();
            this.displayWaypoints(data.waypoints);
        } catch (error) {
            console.error('Load waypoints error:', error);
        }
    }

    displayWaypoints(waypoints) {
        const list = document.getElementById('waypoint-list');
        if (!list) return;

        if (waypoints.length === 0) {
            list.innerHTML = '<p class="empty-message">No waypoints defined</p>';
            return;
        }

        list.innerHTML = waypoints.map(wp => `
            <div class="waypoint-item">
                <span>${wp.name} (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})</span>
                <button onclick="dashboard.navigateToWaypoint('${wp.name}')">Go</button>
            </div>
        `).join('');
    }

    async saveWaypoint(name) {
        try {
            const response = await fetch(`/api/navigation/waypoints/save_current?name=${encodeURIComponent(name)}`, {
                method: 'POST'
            });
            const result = await response.json();
            console.log('Waypoint saved:', result);
            this.loadWaypoints();
        } catch (error) {
            console.error('Save waypoint error:', error);
        }
    }

    async navigateToWaypoint(name) {
        try {
            const response = await fetch('/api/navigation/waypoint/goto', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ waypoint_name: name })
            });
            const result = await response.json();
            console.log('Navigation started:', result);
        } catch (error) {
            console.error('Navigation error:', error);
        }
    }

    async cancelNavigation() {
        try {
            await fetch('/api/navigation/cancel', { method: 'POST' });
            console.log('Navigation cancelled');
        } catch (error) {
            console.error('Cancel navigation error:', error);
        }
    }

    // ==========================================
    // Emergency Stop
    // ==========================================

    setupEmergencyStop() {
        const btn = document.getElementById('emergency-stop');
        if (btn) {
            btn.addEventListener('click', async () => {
                try {
                    await fetch('/api/emergency_stop', { method: 'POST' });
                    this.send({ type: 'stop' });
                    console.log('EMERGENCY STOP');
                } catch (error) {
                    console.error('Emergency stop error:', error);
                }
            });
        }
    }
}

// Initialize dashboard when DOM is ready
let dashboard;
document.addEventListener('DOMContentLoaded', () => {
    dashboard = new RobotDashboard();
});
