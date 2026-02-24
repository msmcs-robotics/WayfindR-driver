/**
 * AMBOT Web Control — Motor control module.
 *
 * Direction pad (mouse/touch), keyboard (WASD + arrows), speed slider.
 * Ported from PI_API/static/js/main.js, adapted for SocketIO.
 */

const Motors = {
    pressedKeys: new Set(),
    speedPercent: 50,
    _sendInterval: null,

    init() {
        this.setupKeyboard();
        this.setupDirectionPad();
        this.setupSpeedSlider();

        // Send motor commands at 10 Hz while keys are held
        this._sendInterval = setInterval(() => this._tickMovement(), 100);
    },

    // ── Keyboard ────────────────────────────────────────────

    setupKeyboard() {
        document.addEventListener('keydown', (e) => {
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
            if (e.repeat) return;

            const key = this._normalizeKey(e.key);
            if (key && !this.pressedKeys.has(key)) {
                this.pressedKeys.add(key);
                this._updateButtonHighlights();
            }

            if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) {
                e.preventDefault();
            }
        });

        document.addEventListener('keyup', (e) => {
            const key = this._normalizeKey(e.key);
            if (key) {
                this.pressedKeys.delete(key);
                this._updateButtonHighlights();
            }
        });

        // Stop on focus loss
        window.addEventListener('blur', () => {
            this.pressedKeys.clear();
            this._sendStop();
            this._updateButtonHighlights();
        });
    },

    _normalizeKey(key) {
        const map = {
            'ArrowUp': 'w', 'ArrowDown': 's',
            'ArrowLeft': 'a', 'ArrowRight': 'd',
        };
        const k = map[key] || key.toLowerCase();
        if ('wasdeq'.includes(k)) return k;
        return null;
    },

    // ── Direction Pad ───────────────────────────────────────

    setupDirectionPad() {
        // Throttle + steering buttons
        document.querySelectorAll('.dir-btn[data-throttle]').forEach(btn => {
            const throttle = parseFloat(btn.dataset.throttle);
            const steering = parseFloat(btn.dataset.steering);

            const start = () => {
                this._sendDrive(throttle, steering);
                btn.classList.add('active');
            };
            const stop = () => {
                this._sendStop();
                btn.classList.remove('active');
            };

            btn.addEventListener('mousedown', start);
            btn.addEventListener('mouseup', stop);
            btn.addEventListener('mouseleave', () => {
                if (btn.classList.contains('active')) stop();
            });
            btn.addEventListener('touchstart', (e) => { e.preventDefault(); start(); });
            btn.addEventListener('touchend', stop);
        });

        // Rotate buttons
        document.querySelectorAll('.dir-btn[data-rotate]').forEach(btn => {
            const rotate = parseFloat(btn.dataset.rotate);

            const start = () => {
                this._sendDrive(0, rotate);
                btn.classList.add('active');
            };
            const stop = () => {
                this._sendStop();
                btn.classList.remove('active');
            };

            btn.addEventListener('mousedown', start);
            btn.addEventListener('mouseup', stop);
            btn.addEventListener('mouseleave', () => {
                if (btn.classList.contains('active')) stop();
            });
            btn.addEventListener('touchstart', (e) => { e.preventDefault(); start(); });
            btn.addEventListener('touchend', stop);
        });

        // Center STOP button
        const stopBtn = document.getElementById('btn-stop');
        if (stopBtn) {
            stopBtn.addEventListener('click', () => this._sendStop());
        }
    },

    // ── Speed Slider ────────────────────────────────────────

    setupSpeedSlider() {
        const slider = document.getElementById('speed-slider');
        const display = document.getElementById('speed-value');
        if (slider) {
            slider.addEventListener('input', () => {
                this.speedPercent = parseInt(slider.value);
                if (display) display.textContent = slider.value;
            });
        }
    },

    // ── Movement Logic ──────────────────────────────────────

    _tickMovement() {
        if (this.pressedKeys.size === 0) return;

        let throttle = 0;
        let steering = 0;

        if (this.pressedKeys.has('w')) throttle = 1;
        else if (this.pressedKeys.has('s')) throttle = -1;

        if (this.pressedKeys.has('a')) steering = -1;
        else if (this.pressedKeys.has('d')) steering = 1;

        // Q/E for rotate-in-place
        if (this.pressedKeys.has('q') && throttle === 0) { steering = -1; }
        if (this.pressedKeys.has('e') && throttle === 0) { steering = 1; }

        if (throttle !== 0 || steering !== 0) {
            this._sendDrive(throttle, steering);
        } else {
            this._sendStop();
        }
    },

    _sendDrive(throttle, steering) {
        if (!App.socket) return;
        App.socket.emit('motor_command', {
            throttle: throttle,
            steering: steering * 0.5,  // Reduce turn sharpness
        });
    },

    _sendStop() {
        if (!App.socket) return;
        App.socket.emit('stop');
    },

    // ── Button Highlights ───────────────────────────────────

    _updateButtonHighlights() {
        document.querySelectorAll('.dir-btn').forEach(b => b.classList.remove('active'));

        const has = (k) => this.pressedKeys.has(k);

        if (has('w') && has('a'))      document.getElementById('btn-forward-left')?.classList.add('active');
        else if (has('w') && has('d')) document.getElementById('btn-forward-right')?.classList.add('active');
        else if (has('w'))             document.getElementById('btn-forward')?.classList.add('active');
        else if (has('s') && has('a')) document.getElementById('btn-backward-left')?.classList.add('active');
        else if (has('s') && has('d')) document.getElementById('btn-backward-right')?.classList.add('active');
        else if (has('s'))             document.getElementById('btn-backward')?.classList.add('active');
        else if (has('a') || has('q')) document.getElementById('btn-rotate-left')?.classList.add('active');
        else if (has('d') || has('e')) document.getElementById('btn-rotate-right')?.classList.add('active');
    },
};

document.addEventListener('DOMContentLoaded', () => Motors.init());
