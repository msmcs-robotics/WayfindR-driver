/**
 * AMBOT Web Control — Chat module.
 *
 * Sends questions to /api/chat/ask and displays responses.
 */

const Chat = {
    messagesEl: null,
    inputEl: null,
    sendBtn: null,

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
    },

    async send() {
        const question = this.inputEl?.value?.trim();
        if (!question) return;

        this.inputEl.value = '';
        this.addMessage(question, 'user');
        this.sendBtn.disabled = true;

        try {
            const resp = await fetch('/api/chat/ask', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ question }),
            });
            const data = await resp.json();

            if (data.error) {
                this.addMessage(data.error, 'error');
            } else {
                this.addMessage(data.answer || 'No response', 'assistant');
            }
        } catch (e) {
            this.addMessage('Failed to reach chat API', 'error');
        } finally {
            this.sendBtn.disabled = false;
            this.inputEl?.focus();
        }
    },

    addMessage(text, type) {
        if (!this.messagesEl) return;
        const div = document.createElement('div');
        div.className = 'chat-msg ' + type;
        div.textContent = text;
        this.messagesEl.appendChild(div);
        this.messagesEl.scrollTop = this.messagesEl.scrollHeight;
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
