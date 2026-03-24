"""Conversation persistence via PostgreSQL (reuses the RAG Docker container).

Stores conversation history so sessions survive app restarts.
Connects to the existing ambot-rag-postgres container.
"""

import json
import logging
import os
import time
from contextlib import contextmanager

logger = logging.getLogger(__name__)

# Connection config — same DB as RAG, separate table
DB_HOST = os.environ.get("DB_HOST", "localhost")
DB_PORT = int(os.environ.get("DB_PORT", "5432"))
DB_NAME = os.environ.get("DB_NAME", "ambot_rag")
DB_USER = os.environ.get("DB_USER", "ambot")
DB_PASSWORD = os.environ.get("DB_PASSWORD", "ambot_secure_pass")

_conn = None


def _get_conn():
    """Get or create a database connection."""
    global _conn
    if _conn is not None:
        try:
            _conn.cursor().execute("SELECT 1")
            return _conn
        except Exception:
            _conn = None

    try:
        import psycopg2
        _conn = psycopg2.connect(
            host=DB_HOST, port=DB_PORT, dbname=DB_NAME,
            user=DB_USER, password=DB_PASSWORD,
            connect_timeout=5,
        )
        _conn.autocommit = True
        logger.info("Connected to PostgreSQL at %s:%d/%s", DB_HOST, DB_PORT, DB_NAME)
        return _conn
    except ImportError:
        logger.warning("psycopg2 not installed — conversation persistence disabled")
        return None
    except Exception as e:
        logger.warning("PostgreSQL connection failed: %s — persistence disabled", e)
        return None


def init_db():
    """Create the conversations table if it doesn't exist."""
    conn = _get_conn()
    if not conn:
        return False

    try:
        cur = conn.cursor()
        cur.execute("""
            CREATE TABLE IF NOT EXISTS conversations (
                id SERIAL PRIMARY KEY,
                session_id VARCHAR(32) NOT NULL,
                role VARCHAR(16) NOT NULL,
                content TEXT NOT NULL,
                turn_number INTEGER DEFAULT 0,
                is_condensation BOOLEAN DEFAULT FALSE,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        cur.execute("""
            CREATE INDEX IF NOT EXISTS idx_conversations_session
            ON conversations (session_id, created_at)
        """)
        cur.execute("""
            CREATE TABLE IF NOT EXISTS sessions (
                session_id VARCHAR(32) PRIMARY KEY,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                turn_count INTEGER DEFAULT 0,
                condensation_count INTEGER DEFAULT 0
            )
        """)
        logger.info("Conversation tables ready")
        return True
    except Exception as e:
        logger.warning("Failed to create tables: %s", e)
        return False


def save_message(session_id: str, role: str, content: str,
                 turn_number: int = 0, is_condensation: bool = False):
    """Save a conversation message."""
    conn = _get_conn()
    if not conn:
        return

    try:
        cur = conn.cursor()
        cur.execute(
            "INSERT INTO conversations (session_id, role, content, turn_number, is_condensation) "
            "VALUES (%s, %s, %s, %s, %s)",
            (session_id, role, content, turn_number, is_condensation),
        )
        cur.execute(
            "INSERT INTO sessions (session_id, turn_count) VALUES (%s, %s) "
            "ON CONFLICT (session_id) DO UPDATE SET "
            "updated_at = CURRENT_TIMESTAMP, turn_count = %s",
            (session_id, turn_number, turn_number),
        )
    except Exception as e:
        logger.warning("Failed to save message: %s", e)


def load_session(session_id: str) -> dict | None:
    """Load a session's conversation history from the database.

    Returns dict with 'messages', 'turn_count', 'condensation_count'
    or None if session not found.
    """
    conn = _get_conn()
    if not conn:
        return None

    try:
        cur = conn.cursor()
        cur.execute(
            "SELECT role, content, is_condensation FROM conversations "
            "WHERE session_id = %s ORDER BY created_at",
            (session_id,),
        )
        rows = cur.fetchall()
        if not rows:
            return None

        messages = []
        for role, content, is_condensation in rows:
            messages.append({"role": role, "content": content})

        cur.execute(
            "SELECT turn_count, condensation_count FROM sessions WHERE session_id = %s",
            (session_id,),
        )
        session_row = cur.fetchone()
        turn_count = session_row[0] if session_row else len(messages) // 2
        condensation_count = session_row[1] if session_row else 0

        return {
            "messages": messages,
            "turn_count": turn_count,
            "condensation_count": condensation_count,
            "created": time.time(),
        }
    except Exception as e:
        logger.warning("Failed to load session: %s", e)
        return None


def update_condensation(session_id: str, count: int):
    """Update the condensation count for a session."""
    conn = _get_conn()
    if not conn:
        return

    try:
        cur = conn.cursor()
        cur.execute(
            "UPDATE sessions SET condensation_count = %s WHERE session_id = %s",
            (count, session_id),
        )
    except Exception as e:
        logger.warning("Failed to update condensation: %s", e)


def clear_session(session_id: str):
    """Delete all messages for a session."""
    conn = _get_conn()
    if not conn:
        return

    try:
        cur = conn.cursor()
        cur.execute("DELETE FROM conversations WHERE session_id = %s", (session_id,))
        cur.execute("DELETE FROM sessions WHERE session_id = %s", (session_id,))
    except Exception as e:
        logger.warning("Failed to clear session: %s", e)


def list_sessions(limit: int = 20) -> list[dict]:
    """List recent sessions."""
    conn = _get_conn()
    if not conn:
        return []

    try:
        cur = conn.cursor()
        cur.execute(
            "SELECT session_id, created_at, updated_at, turn_count "
            "FROM sessions ORDER BY updated_at DESC LIMIT %s",
            (limit,),
        )
        return [
            {"session_id": r[0], "created_at": str(r[1]),
             "updated_at": str(r[2]), "turn_count": r[3]}
            for r in cur.fetchall()
        ]
    except Exception as e:
        logger.warning("Failed to list sessions: %s", e)
        return []
