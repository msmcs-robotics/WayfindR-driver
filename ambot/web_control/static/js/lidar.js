/**
 * AMBOT Web Control — LiDAR polar plot renderer.
 *
 * Fetches scan data and renders on a canvas with safety zone rings.
 * Shows a "not connected" placeholder when no LiDAR hardware is available.
 */

const Lidar = {
    canvas: null,
    ctx: null,
    pollTimer: null,
    maxRange: 6000,  // mm
    _connected: false,

    init() {
        this.canvas = document.getElementById('lidar-canvas');
        if (!this.canvas) return;
        this.ctx = this.canvas.getContext('2d');

        // Check sensor status first, then start polling only if connected
        this.checkSensorStatus();
    },

    async checkSensorStatus() {
        try {
            const resp = await fetch('/api/diagnostics/sensors');
            if (resp.ok) {
                const data = await resp.json();
                this._connected = data.lidar === true;
            }
        } catch (e) {
            this._connected = false;
        }

        if (this._connected) {
            this.pollTimer = setInterval(() => this.fetchAndDraw(), 250);
            this.draw([]);
        } else {
            this.showPlaceholder();
        }
    },

    showPlaceholder() {
        // Hide canvas, show placeholder
        if (this.canvas) this.canvas.style.display = 'none';
        const placeholder = document.getElementById('lidar-placeholder');
        if (placeholder) placeholder.style.display = 'flex';
    },

    async fetchAndDraw() {
        try {
            const resp = await fetch('/api/diagnostics/lidar_scan');
            if (resp.ok) {
                const data = await resp.json();
                this.draw(data.scan || []);
                this.updateClosest(data.scan || []);
            }
        } catch (e) {
            // silent
        }
    },

    draw(points) {
        const ctx = this.ctx;
        const w = this.canvas.width;
        const h = this.canvas.height;
        const cx = w / 2;
        const cy = h / 2;
        const radius = Math.min(cx, cy) - 10;

        ctx.clearRect(0, 0, w, h);

        // Background
        ctx.fillStyle = '#0f0f23';
        ctx.fillRect(0, 0, w, h);

        // Safety zone rings
        const zones = [500, 1000, 2000, 4000];  // mm
        zones.forEach((range, i) => {
            const r = (range / this.maxRange) * radius;
            ctx.beginPath();
            ctx.arc(cx, cy, r, 0, Math.PI * 2);
            ctx.strokeStyle = i === 0 ? 'rgba(231,76,60,0.3)' : 'rgba(78,204,163,0.15)';
            ctx.lineWidth = 1;
            ctx.stroke();
        });

        // Cross-hairs
        ctx.beginPath();
        ctx.moveTo(cx, 10); ctx.lineTo(cx, h - 10);
        ctx.moveTo(10, cy); ctx.lineTo(w - 10, cy);
        ctx.strokeStyle = 'rgba(78,204,163,0.1)';
        ctx.stroke();

        // Robot marker
        ctx.beginPath();
        ctx.arc(cx, cy, 4, 0, Math.PI * 2);
        ctx.fillStyle = '#4ecca3';
        ctx.fill();

        // Forward direction indicator
        ctx.beginPath();
        ctx.moveTo(cx, cy - 6);
        ctx.lineTo(cx - 3, cy);
        ctx.lineTo(cx + 3, cy);
        ctx.closePath();
        ctx.fillStyle = '#4ecca3';
        ctx.fill();

        // Draw scan points
        if (points.length === 0) {
            ctx.fillStyle = '#8892b0';
            ctx.font = '14px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('Waiting for scan data...', cx, cy + 30);
            return;
        }

        ctx.fillStyle = '#4ecca3';
        points.forEach(p => {
            const angle = p.angle;
            const dist = p.distance;
            if (dist <= 0 || dist > this.maxRange) return;

            const r = (dist / this.maxRange) * radius;
            // Angle 0 = forward (up), clockwise
            const rad = (angle - 90) * Math.PI / 180;
            const px = cx + r * Math.cos(rad);
            const py = cy + r * Math.sin(rad);

            ctx.beginPath();
            ctx.arc(px, py, 1.5, 0, Math.PI * 2);
            ctx.fill();
        });

        // Labels
        ctx.fillStyle = '#8892b0';
        ctx.font = '10px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('0.5m', cx, cy - (500 / this.maxRange) * radius - 3);
        ctx.fillText('1m', cx, cy - (1000 / this.maxRange) * radius - 3);
        ctx.fillText('2m', cx, cy - (2000 / this.maxRange) * radius - 3);
    },

    updateClosest(points) {
        const el = document.getElementById('lidar-closest');
        if (!el || points.length === 0) return;

        let min = Infinity;
        points.forEach(p => {
            if (p.distance > 0 && p.distance < min) min = p.distance;
        });

        if (min < Infinity) {
            el.textContent = 'Closest: ' + Math.round(min) + ' mm';
            el.style.color = min < 500 ? 'var(--danger)' : 'var(--text-dim)';
        }
    },
};

document.addEventListener('DOMContentLoaded', () => Lidar.init());
