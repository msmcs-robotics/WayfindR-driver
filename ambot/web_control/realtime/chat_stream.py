"""SocketIO chat streaming — proxies RAG API token stream to browser."""

import json
import logging
import time

import requests
from flask_socketio import emit

logger = logging.getLogger(__name__)


def register_chat_stream(socketio, app):
    """Register SocketIO event handlers for streaming chat."""

    @socketio.on('chat:ask')
    def on_chat_ask(data):
        """Handle a chat question — stream the response back token by token."""
        question = (data.get('question') or '').strip()
        if not question:
            emit('chat:error', {'error': 'No question provided'})
            return

        rag_url = app.config.get('RAG_API_URL', 'http://10.33.255.82:8000')

        # Stage 1: Searching
        emit('chat:stage', {'stage': 'searching', 'message': 'Searching knowledge base...'})

        try:
            resp = requests.post(
                f'{rag_url}/api/ask/stream',
                json={'question': question},
                stream=True,
                timeout=120,
            )
            resp.raise_for_status()
        except requests.ConnectionError:
            emit('chat:error', {'error': 'LLM service unavailable'})
            return
        except requests.Timeout:
            emit('chat:error', {'error': 'Request timed out'})
            return
        except Exception as e:
            logger.error('Chat stream error: %s', e)
            emit('chat:error', {'error': str(e)})
            return

        # Stage 2: Generating (we'll emit this after sources arrive)
        sources_sent = False
        token_count = 0
        t_start = time.monotonic()

        for line in resp.iter_lines(decode_unicode=True):
            if not line:
                continue
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue

            event = msg.get('event')

            if event == 'sources':
                sources_sent = True
                emit('chat:sources', {
                    'sources': msg.get('sources', []),
                    'model': msg.get('model', 'unknown'),
                })
                # Now generating
                emit('chat:stage', {'stage': 'generating', 'message': 'Generating response...'})

            elif event == 'token':
                token = msg.get('token', '')
                if token:
                    token_count += 1
                    emit('chat:token', {'token': token})

            elif event == 'error':
                emit('chat:error', {'error': msg.get('error', 'Unknown error')})
                return

            elif event == 'done':
                elapsed = time.monotonic() - t_start
                emit('chat:done', {
                    'token_count': token_count,
                    'elapsed': round(elapsed, 1),
                })
                return

        # If we get here without 'done', still signal completion
        elapsed = time.monotonic() - t_start
        emit('chat:done', {'token_count': token_count, 'elapsed': round(elapsed, 1)})
