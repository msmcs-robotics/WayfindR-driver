/**
 * AMBOT Web Control — Chat module.
 *
 * Sends questions via SocketIO for streaming token-by-token responses.
 * Falls back to REST /api/chat/ask if SocketIO is not connected.
 */

const Chat = {
    messagesEl: null,
    inputEl: null,
    sendBtn: null,
    _streaming: false,   // true while a stream is in progress
    _currentMsg: null,    // the DOM element being built during stream
    _stageEl: null,       // the loading stage indicator element

    init() {
        this.messagesEl = document.getElementById('chat-messages');
        this.inputEl = document.getElementById('chat-input');
        this.sendBtn = document.getElementById('chat-send');

        if (this.sendBtn) {
            this.sendBtn.addEventListener('click', () => this.send());
        }
        if (this.inputEl) {
            this.inputEl.addEventListener('keydown', (e) => {
                if (e.key === 'Enter') {
                    e.preventDefault();
                    this.send();
                }
            });
        }

        this.checkLLMStatus();
        this._registerSocketEvents();
    },

    // ── SocketIO streaming events ──────────────────────────────

    _registerSocketEvents() {
        // Wait for App.socket to be ready
        const tryRegister = () => {
            if (typeof App !== 'undefined' && App.socket) {
                this._bindSocket(App.socket);
            } else {
                setTimeout(tryRegister, 200);
            }
        };
        tryRegister();
    },

    _bindSocket(socket) {
        socket.on('chat:stage', (data) => {
            this._showStage(data.message);
        });

        socket.on('chat:sources', (data) => {
            // Sources arrived — search is done, generation starting
            this._removeStage();
            this._showStage('Generating response...');
            // Start the assistant message element
            this._currentMsg = this._createStreamMsg();
        });

        socket.on('chat:token', (data) => {
            if (this._currentMsg && data.token) {
                this._currentMsg.textContent += data.token;
                this.messagesEl.scrollTop = this.messagesEl.scrollHeight;
            }
        });

        socket.on('chat:done', (data) => {
            this._removeStage();
            this._streaming = false;
            this._currentMsg = null;
            this.sendBtn.disabled = false;
            this.inputEl?.focus();
        });

        socket.on('chat:error', (data) => {
            this._removeStage();
            this._streaming = false;
            this._currentMsg = null;
            this.addMessage(data.error || 'Chat error', 'error');
            this.sendBtn.disabled = false;
            this.inputEl?.focus();
        });
    },

    // ── Send a question ────────────────────────────────────────

    send() {
        const question = this.inputEl?.value?.trim();
        if (!question || this._streaming) return;

        this.inputEl.value = '';
        this.addMessage(question, 'user');
        this.sendBtn.disabled = true;

        // Prefer SocketIO streaming if connected
        if (typeof App !== 'undefined' && App.socket && App.connected) {
            this._streaming = true;
            App.socket.emit('chat:ask', { question });
        } else {
            // Fallback to REST
            this._sendREST(question);
        }
    },

    async _sendREST(question) {
        this._showStage('Waiting for response...');
        try {
            const resp = await fetch('/api/chat/ask', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ question }),
            });
            const data = await resp.json();
            this._removeStage();

            if (data.error) {
                this.addMessage(data.error, 'error');
            } else {
                this.addMessage(data.answer || 'No response', 'assistant');
            }
        } catch (e) {
            this._removeStage();
            this.addMessage('Failed to reach chat API', 'error');
        } finally {
            this.sendBtn.disabled = false;
            this.inputEl?.focus();
        }
    },

    // ── DOM helpers ────────────────────────────────────────────

    addMessage(text, type) {
        if (!this.messagesEl) return;
        const div = document.createElement('div');
        div.className = 'chat-msg ' + type;
        div.textContent = text;
        this.messagesEl.appendChild(div);
        this.messagesEl.scrollTop = this.messagesEl.scrollHeight;
    },

    _createStreamMsg() {
        if (!this.messagesEl) return null;
        const div = document.createElement('div');
        div.className = 'chat-msg assistant streaming';
        this.messagesEl.appendChild(div);
        this.messagesEl.scrollTop = this.messagesEl.scrollHeight;
        return div;
    },

    _showStage(message) {
        this._removeStage();
        if (!this.messagesEl) return;
        const el = document.createElement('div');
        el.className = 'chat-stage';
        el.innerHTML = '<span class="chat-spinner"></span> ' + message;
        this.messagesEl.appendChild(el);
        this.messagesEl.scrollTop = this.messagesEl.scrollHeight;
        this._stageEl = el;
    },

    _removeStage() {
        if (this._stageEl) {
            this._stageEl.remove();
            this._stageEl = null;
        }
    },

    async checkLLMStatus() {
        const dot = document.getElementById('llm-status');
        const text = document.getElementById('llm-status-text');
        try {
            const resp = await fetch('/api/chat/status');
            const data = await resp.json();
            if (data.available) {
                if (dot) dot.className = 'status-dot online';
                if (text) text.textContent = 'LLM: connected';
            } else {
                if (dot) dot.className = 'status-dot offline';
                if (text) text.textContent = 'LLM: unavailable';
            }
        } catch (e) {
            if (dot) dot.className = 'status-dot offline';
            if (text) text.textContent = 'LLM: unavailable';
        }
    },
};

document.addEventListener('DOMContentLoaded', () => Chat.init());
