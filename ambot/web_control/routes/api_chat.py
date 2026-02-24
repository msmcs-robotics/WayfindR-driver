"""LLM Chat API — proxies requests to Jetson RAG service."""

import logging

import requests
from flask import Blueprint, current_app, jsonify, request

logger = logging.getLogger(__name__)

chat_bp = Blueprint('chat', __name__)


@chat_bp.route('/ask', methods=['POST'])
def ask():
    """Send a question to the RAG API and return the response."""
    data = request.get_json(force=True)
    question = data.get('question', '').strip()

    if not question:
        return jsonify({'error': 'No question provided'}), 400

    rag_url = current_app.config.get('RAG_API_URL', 'http://10.33.255.82:8000')

    try:
        resp = requests.post(
            f'{rag_url}/api/query',
            json={'question': question},
            timeout=60,
        )
        resp.raise_for_status()
        result = resp.json()
        return jsonify({
            'answer': result.get('answer', ''),
            'sources': result.get('sources', []),
            'model': result.get('model', 'unknown'),
        })
    except requests.ConnectionError:
        logger.warning('RAG API unreachable at %s', rag_url)
        return jsonify({
            'error': 'LLM service unavailable',
            'detail': f'Cannot reach {rag_url}',
        }), 503
    except requests.Timeout:
        return jsonify({'error': 'LLM request timed out'}), 504
    except Exception as e:
        logger.error('Chat error: %s', e)
        return jsonify({'error': str(e)}), 500


@chat_bp.route('/status')
def chat_status():
    """Check if the RAG API is reachable."""
    rag_url = current_app.config.get('RAG_API_URL', 'http://10.33.255.82:8000')
    try:
        resp = requests.get(f'{rag_url}/api/health', timeout=5)
        resp.raise_for_status()
        return jsonify({'available': True, 'url': rag_url})
    except Exception:
        return jsonify({'available': False, 'url': rag_url})
