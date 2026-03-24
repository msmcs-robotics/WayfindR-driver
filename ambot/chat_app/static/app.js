/**
 * AMBOT Chat — Frontend with SSE streaming, timing, and conversation context.
 */

const Chat = {
    chatArea: null,
    input: null,
    sendBtn: null,
    statusDot: null,
    statusText: null,
    _busy: false,
    _model: 'unknown',
    _sessionId: null,
    _turnCount: 0,

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

        // New session button
        const newBtn = document.getElementById('new-session-btn');
        if (newBtn) newBtn.addEventListener('click', () => this.newSession());

        this.checkHealth();
        this.createSession();
        setInterval(() => this.checkHealth(), 30000);
    },

    // ── Session Management ────────────────────────

    async createSession() {
        try {
            const resp = await fetch('/api/session', { method: 'POST' });
            const data = await resp.json();
            this._sessionId = data.session_id;
            this._turnCount = 0;
            this._updateTurnCounter();
        } catch (e) {
            this._sessionId = 'default';
        }
    },

    async newSession() {
        if (this._sessionId && this._sessionId !== 'default') {
            await fetch('/api/session/' + this._sessionId, { method: 'DELETE' }).catch(() => {});
        }
        // Clear chat area
        this.chatArea.innerHTML = '';
        this._addMessage('system', 'New conversation started. Ask me anything!');
        this._turnCount = 0;
        this._updateTurnCounter();
        await this.createSession();
        this.input.focus();
    },

    _updateTurnCounter() {
        const el = document.getElementById('turn-counter');
        if (el) {
            el.textContent = this._turnCount > 0
                ? 'Turn ' + this._turnCount
                : '';
        }
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
            } else if (data.status === 'degraded') {
                this._setStatus('checking', 'Degraded');
                return true;
            } else {
                this._setStatus('offline', data.error || 'Offline');
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

        const sendTime = performance.now();

        // Show user message
        this._addMessage('user', question);

        // Create stage indicator
        const stageEl = this._addStage('Connecting...');

        // Create assistant message (will be filled by streaming)
        const msgEl = this._addMessage('assistant', '');
        const textEl = msgEl.querySelector('.msg-text');

        let sources = [];
        let mcpActions = [];
        let gotTokens = false;
        let timing = null;
        let queryType = null;
        let condensationCount = 0;

        try {
            const resp = await fetch('/api/chat', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    question,
                    session_id: this._sessionId,
                }),
            });

            if (!resp.ok) {
                throw new Error('Server returned ' + resp.status);
            }

            const reader = resp.body.getReader();
            const decoder = new TextDecoder();
            let buffer = '';

            while (true) {
                const { done, value } = await reader.read();
                if (done) break;

                buffer += decoder.decode(value, { stream: true });
                const lines = buffer.split('\n');
                buffer = lines.pop();

                let eventType = null;
                for (const line of lines) {
                    if (line.startsWith('event: ')) {
                        eventType = line.slice(7).trim();
                    } else if (line.startsWith('data: ') && eventType) {
                        try {
                            const data = JSON.parse(line.slice(6));

                            if (eventType === 'state') {
                                const stageText = stageEl.querySelector('.stage-text');
                                if (stageText) stageText.textContent = data.message || data.stage;

                                if (data.stage === 'done') {
                                    timing = data.timing;
                                    queryType = data.query_type;
                                    this._turnCount = data.turn || this._turnCount + 1;
                                    condensationCount = data.condensation_count || 0;
                                }
                            } else if (eventType === 'sources') {
                                sources = data.sources || [];
                            } else if (eventType === 'mcp_action') {
                                mcpActions.push(data);
                                this._addMcpAction(msgEl, data);
                            } else if (eventType === 'token') {
                                textEl.textContent += data.text;
                                gotTokens = true;
                                this._scrollToBottom();
                            } else if (eventType === 'error') {
                                stageEl.remove();
                                this._addMessage('error', data.message || 'Unknown error');
                            }
                        } catch (e) {
                            // skip
                        }
                        eventType = null;
                    }
                }
            }
        } catch (e) {
            stageEl.remove();
            if (!gotTokens) {
                msgEl.remove();
            }
            this._addMessage('error', 'Error: ' + e.message);
        }

        // Finalize
        stageEl.remove();

        if (gotTokens) {
            // Add metadata footer
            const footer = document.createElement('div');
            footer.className = 'msg-footer';

            // Timing
            const clientMs = Math.round(performance.now() - sendTime);
            const totalSec = timing
                ? (timing.total_ms / 1000).toFixed(1)
                : (clientMs / 1000).toFixed(1);
            let timingText = totalSec + 's';
            if (timing && timing.search_ms > 0) {
                timingText += ' (search: ' + (timing.search_ms / 1000).toFixed(1) + 's)';
            }
            footer.textContent = this._model + ' · ' + timingText;

            // Query type indicator
            if (queryType === 'casual') {
                footer.textContent += ' · direct';
            } else if (queryType === 'rag' && sources.length > 0) {
                footer.textContent += ' · ' + sources.length + ' sources';
            }

            // Condensation notice
            if (condensationCount > 0) {
                footer.textContent += ' · condensed x' + condensationCount;
            }

            msgEl.appendChild(footer);

            // Sources (collapsible)
            if (sources.length > 0) {
                this._addSources(msgEl, sources);
            }
        }

        this._updateTurnCounter();

        this._busy = false;
        this.input.disabled = false;
        this.sendBtn.disabled = false;
        this.input.focus();
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

    _addMcpAction(msgEl, data) {
        const action = document.createElement('div');
        action.className = 'mcp-action';
        action.textContent = '\u2192 MCP: ' + (data.text || data.tool);
        // Insert before the message text
        const textEl = msgEl.querySelector('.msg-text');
        if (textEl) {
            msgEl.insertBefore(action, textEl);
        } else {
            msgEl.appendChild(action);
        }
        this._scrollToBottom();
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
