/**
 * AMBOT Web Control — Camera feed module.
 *
 * Manages MJPEG display and face detection overlay.
 */

const Camera = {
    pollTimer: null,

    init() {
        // Poll face data at ~3 Hz
        this.pollTimer = setInterval(() => this.fetchFaces(), 333);
    },

    async fetchFaces() {
        try {
            const resp = await fetch('/api/diagnostics/faces');
            if (!resp.ok) return;
            const data = await resp.json();
            this.updateFaceCount(data.count || 0);
        } catch (e) {
            // silent
        }
    },

    updateFaceCount(count) {
        const el = document.getElementById('face-count');
        if (el) {
            el.textContent = 'Faces: ' + count;
            el.style.color = count > 0 ? 'var(--accent)' : 'var(--text-dim)';
        }
    },
};

document.addEventListener('DOMContentLoaded', () => Camera.init());
