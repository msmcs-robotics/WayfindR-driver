/**
 * AMBOT Web Control — Core application.
 *
 * Manages SocketIO connection, emergency stop, and telemetry updates.
 */

const App = {
    socket: null,
    connected: false,

    init() {
        this.connectSocket();
        this.setupEmergencyStop();
        this.startTelemetryPolling();
    },

    // ── SocketIO ────────────────────────────────────────────

    connectSocket() {
        this.socket = io({
            transports: ['websocket', 'polling'],
            reconnection: true,
            reconnectionDelay: 1000,
            reconnectionDelayMax: 5000,
        });

        this.socket.on('connect', () => {
            console.log('SocketIO connected');
            this.connected = true;
            this.updateConnectionStatus(true);
        });

        this.socket.on('disconnect', () => {
            console.log('SocketIO disconnected');
            this.connected = false;
            this.updateConnectionStatus(false);
        });

        this.socket.on('motor_state', (data) => {
            this.updateMotorBars(data);
        });

        this.socket.on('status', (data) => {
            console.log('Server status:', data);
        });
    },

    updateConnectionStatus(connected) {
        const el = document.getElementById('connection-status');
        if (el) {
            el.textContent = connected ? 'Connected' : 'Disconnected';
            el.className = 'status ' + (connected ? 'connected' : 'disconnected');
        }
    },

    // ── Emergency Stop ──────────────────────────────────────

    setupEmergencyStop() {
        const btn = document.getElementById('emergency-stop');
        if (btn) {
            btn.addEventListener('click', () => this.emergencyStop());
        }
        // Spacebar emergency stop (when not in input)
        document.addEventListener('keydown', (e) => {
            if (e.key === ' ' &&
                e.target.tagName !== 'INPUT' &&
                e.target.tagName !== 'TEXTAREA') {
                e.preventDefault();
                this.emergencyStop();
            }
        });
    },

    emergencyStop() {
        if (this.socket) {
            this.socket.emit('emergency_stop');
        }
        // Also hit the REST endpoint as backup
        fetch('/api/motors/emergency', { method: 'POST' }).catch(() => {});
        console.log('EMERGENCY STOP');
    },

    // ── Motor Bars ──────────────────────────────────────────

    updateMotorBars(data) {
        this._updateBar('motor-left', data.left || 0);
        this._updateBar('motor-right', data.right || 0);

        const cmdEl = document.getElementById('cmd-count');
        if (cmdEl && data.cmd_count !== undefined) {
            cmdEl.textContent = data.cmd_count;
        }
    },

    _updateBar(id, value) {
        const motor = document.getElementById(id);
        if (!motor) return;
        const fill = motor.querySelector('.motor-fill');
        const valEl = motor.querySelector('.motor-value');
        if (fill) {
            fill.style.width = Math.abs(value) + '%';
            fill.style.backgroundColor = value >= 0 ? 'var(--accent)' : 'var(--danger)';
        }
        if (valEl) {
            valEl.textContent = value;
        }
    },

    // ── Telemetry Polling ───────────────────────────────────

    startTelemetryPolling() {
        // Poll diagnostics every 2 seconds
        setInterval(() => this.fetchTelemetry(), 2000);
        this.fetchTelemetry();
    },

    async fetchTelemetry() {
        try {
            const resp = await fetch('/api/diagnostics/system');
            if (!resp.ok) return;
            const data = await resp.json();
            this.updateTelemetry(data);
        } catch (e) {
            // Silently fail — connection might be down
        }
    },

    updateTelemetry(data) {
        // CPU
        const cpuBar = document.getElementById('cpu-bar');
        const cpuVal = document.getElementById('cpu-value');
        if (cpuBar && data.cpu_load !== undefined) {
            // CPU load as percentage (assume 4 cores, load 4.0 = 100%)
            const pct = Math.min(100, (data.cpu_load / 4) * 100);
            cpuBar.style.width = pct + '%';
            cpuBar.style.backgroundColor = pct > 80 ? 'var(--danger)' : 'var(--accent)';
        }
        if (cpuVal) cpuVal.textContent = (data.cpu_load || 0).toFixed(2);

        // Memory
        const memBar = document.getElementById('mem-bar');
        const memVal = document.getElementById('mem-value');
        if (memBar && data.mem_pct !== undefined) {
            memBar.style.width = data.mem_pct + '%';
            memBar.style.backgroundColor = data.mem_pct > 80 ? 'var(--danger)' : 'var(--accent)';
        }
        if (memVal) memVal.textContent = (data.mem_pct || 0).toFixed(1) + '%';

        // Uptime
        const uptimeEl = document.getElementById('uptime-display');
        if (uptimeEl && data.uptime !== undefined) {
            uptimeEl.textContent = this.formatUptime(data.uptime);
        }

        // Motor cmd count
        if (data.motor) {
            this.updateMotorBars(data.motor);
        }

        // Sensor lights
        if (data.sensors) {
            this._setSensorDot('sensor-lidar', data.sensors.lidar);
            this._setSensorDot('sensor-camera', data.sensors.camera);
            this._setSensorDot('sensor-imu', data.sensors.imu);
        }

        // LiDAR scan rate
        if (data.lidar) {
            const rateEl = document.getElementById('scan-rate-display');
            if (rateEl) rateEl.textContent = data.lidar.scan_rate + ' Hz';
            const lrEl = document.getElementById('lidar-rate');
            if (lrEl) lrEl.textContent = 'Scan rate: ' + data.lidar.scan_rate + ' Hz';
        }

        // Camera FPS
        if (data.camera) {
            const fpsEl = document.getElementById('cam-fps-display');
            if (fpsEl) fpsEl.textContent = data.camera.fps;
            const cfEl = document.getElementById('camera-fps');
            if (cfEl) cfEl.textContent = 'FPS: ' + data.camera.fps;
        }
    },

    _setSensorDot(id, isOn) {
        const el = document.getElementById(id);
        if (el) {
            el.className = 'sensor-dot' + (isOn ? ' on' : '');
        }
    },

    formatUptime(seconds) {
        if (seconds < 60) return Math.floor(seconds) + 's';
        if (seconds < 3600) {
            const m = Math.floor(seconds / 60);
            const s = Math.floor(seconds % 60);
            return m + 'm ' + s + 's';
        }
        const h = Math.floor(seconds / 3600);
        const m = Math.floor((seconds % 3600) / 60);
        return h + 'h ' + m + 'm';
    },
};

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => App.init());
