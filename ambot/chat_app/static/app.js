/**
 * AMBOT Chat — Frontend with SSE streaming and state management.
 */

const Chat = {
    chatArea: null,
    input: null,
    sendBtn: null,
    statusDot: null,
    statusText: null,
    _busy: false,
    _model: 'unknown',

    init() {
        this.chatArea = document.getElementById('chat-area');
        this.input = document.getElementById('chat-input');
        this.sendBtn = document.getElementById('send-btn');
        this.statusDot = document.getElementById('status-dot');
        this.statusText = document.getElementById('status-text');

        this.sendBtn.addEventListener('click', () => this.send());
        this.input.addEventListener('keydown', (e) => {
            if (e.key === 'Enter' && !e.shiftKey) {
                e.preventDefault();
                this.send();
            }
        });

        this.checkHealth();
        // Re-check health periodically
        setInterval(() => this.checkHealth(), 30000);
    },

    // ── Health Check ──────────────────────────────

    async checkHealth() {
        this._setStatus('checking', 'Connecting...');
        try {
            const resp = await fetch('/api/health');
            const data = await resp.json();

            if (data.status === 'ready') {
                this._setStatus('online', 'Connected');
                this._fetchModel();
                return true;
            } else {
                this._setStatus('offline',
                    data.error || 'RAG API unavailable');
                return false;
            }
        } catch (e) {
            this._setStatus('offline', 'Server unreachable');
            return false;
        }
    },

    async _fetchModel() {
        try {
            const resp = await fetch('/api/models');
            const data = await resp.json();
            this._model = data.current || 'unknown';
        } catch (e) {
            // silent
        }
    },

    _setStatus(state, text) {
        if (this.statusDot) {
            this.statusDot.className = 'status-dot ' + state;
        }
        if (this.statusText) {
            this.statusText.textContent = text;
        }
    },

    // ── Send Message ──────────────────────────────

    async send() {
        const question = this.input.value.trim();
        if (!question || this._busy) return;

        this._busy = true;
        this.input.value = '';
        this.input.disabled = true;
        this.sendBtn.disabled = true;

        // Show user message
        this._addMessage('user', question);

        // Create stage indicator
        const stageEl = this._addStage('Connecting...');

        // Create assistant message (will be filled by streaming)
        const msgEl = this._addMessage('assistant', '');
        const textEl = msgEl.querySelector('.msg-text');

        let sources = [];
        let model = this._model;
        let gotTokens = false;

        try {
            const resp = await fetch('/api/chat', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ question }),
            });

            if (!resp.ok) {
                throw new Error(`Server returned ${resp.status}`);
            }

            const reader = resp.body.getReader();
            const decoder = new TextDecoder();
            let buffer = '';

            while (true) {
                const { done, value } = await reader.read();
                if (done) break;

                buffer += decoder.decode(value, { stream: true });
                const lines = buffer.split('\n');
                buffer = lines.pop(); // Keep incomplete line

                for (const line of lines) {
                    if (line.startsWith('event: ')) {
                        var eventType = line.slice(7).trim();
                    } else if (line.startsWith('data: ') && eventType) {
                        try {
                            const data = JSON.parse(line.slice(6));
                            this._handleEvent(eventType, data, stageEl,
                                              textEl, msgEl, { sources, model, gotTokens });

                            if (eventType === 'sources') {
                                sources = data.sources || [];
                                model = data.model || model;
                            }
                            if (eventType === 'token') {
                                gotTokens = true;
                            }
                        } catch (e) {
                            // skip malformed JSON
                        }
                        eventType = null;
                    }
                }
            }
        } catch (e) {
            stageEl.remove();
            if (!gotTokens) {
                textEl.textContent = '';
                msgEl.remove();
            }
            this._addMessage('error', 'Error: ' + e.message);
        }

        // Finalize
        stageEl.remove();

        // Add sources + model badge if we got a response
        if (gotTokens && sources.length > 0) {
            this._addSources(msgEl, sources);
        }
        if (gotTokens) {
            this._addModelBadge(msgEl, model);
        }

        this._busy = false;
        this.input.disabled = false;
        this.sendBtn.disabled = false;
        this.input.focus();
    },

    _handleEvent(type, data, stageEl, textEl, msgEl, ctx) {
        switch (type) {
            case 'state':
                stageEl.querySelector('.stage-text').textContent =
                    data.message || data.stage;
                break;

            case 'sources':
                // Sources received, stage transitions to "generating"
                break;

            case 'token':
                textEl.textContent += data.text;
                this._scrollToBottom();
                break;

            case 'error':
                stageEl.remove();
                this._addMessage('error', data.message || 'Unknown error');
                break;
        }
    },

    // ── DOM Helpers ───────────────────────────────

    _addMessage(role, text) {
        const div = document.createElement('div');
        div.className = 'message ' + role;

        const textSpan = document.createElement('span');
        textSpan.className = 'msg-text';
        textSpan.textContent = text;
        div.appendChild(textSpan);

        this.chatArea.appendChild(div);
        this._scrollToBottom();
        return div;
    },

    _addStage(text) {
        const div = document.createElement('div');
        div.className = 'stage-indicator';
        div.innerHTML =
            '<div class="spinner"></div>' +
            '<span class="stage-text">' + text + '</span>';
        this.chatArea.appendChild(div);
        this._scrollToBottom();
        return div;
    },

    _addSources(msgEl, sources) {
        const details = document.createElement('details');
        details.className = 'sources';

        const summary = document.createElement('summary');
        summary.textContent = sources.length + ' source' +
            (sources.length !== 1 ? 's' : '');
        details.appendChild(summary);

        sources.forEach(s => {
            const item = document.createElement('div');
            item.className = 'source-item';
            const name = s.document_filename || 'unknown';
            const score = s.score ? (s.score * 100).toFixed(0) + '%' : '';
            item.innerHTML =
                '<span class="source-filename">' + this._esc(name) + '</span>' +
                (score ? '<span class="source-score">' + score + '</span>' : '');
            details.appendChild(item);
        });

        msgEl.appendChild(details);
    },

    _addModelBadge(msgEl, model) {
        const badge = document.createElement('div');
        badge.className = 'model-badge';
        badge.textContent = model;
        msgEl.appendChild(badge);
    },

    _scrollToBottom() {
        this.chatArea.scrollTop = this.chatArea.scrollHeight;
    },

    _esc(str) {
        const div = document.createElement('div');
        div.textContent = str;
        return div.innerHTML;
    },
};

document.addEventListener('DOMContentLoaded', () => Chat.init());
