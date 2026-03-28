#!/usr/bin/env python3
"""
Watch knowledge/ directory for changes and trigger RAG re-ingestion.

Uses os.scandir + mtime polling (portable, no inotify dependency).
Debounces rapid changes into a single ingestion call.

Usage:
    python3 watch_and_ingest.py              # Watch mode (default)
    python3 watch_and_ingest.py --once       # Single ingestion, then exit
    python3 watch_and_ingest.py --watch      # Explicit watch mode
    python3 watch_and_ingest.py --interval 3 # Custom poll interval (seconds)
    python3 watch_and_ingest.py --log /tmp/watch_ingest.log

Environment:
    RAG_API_URL   Base URL for the RAG API (default: http://localhost:8000)
    KNOWLEDGE_DIR Path to knowledge directory (default: auto-detect relative to script)
"""

import argparse
import json
import logging
import os
import sys
import time
import urllib.error
import urllib.request

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

WATCHED_EXTENSIONS = {".md", ".txt", ".pdf", ".json", ".yaml", ".yml", ".log"}
DEFAULT_API_URL = "http://localhost:8000"
DEFAULT_POLL_INTERVAL = 2  # seconds between polls
DEFAULT_DEBOUNCE = 5       # seconds to wait after last change before ingesting
INGEST_DOCKER_PATH = "/data/knowledge"
MAX_RETRIES = 1

# ---------------------------------------------------------------------------
# Logging setup
# ---------------------------------------------------------------------------

logger = logging.getLogger("watch_ingest")


def setup_logging(log_file=None):
    fmt = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    handler_stdout = logging.StreamHandler(sys.stdout)
    handler_stdout.setFormatter(fmt)
    logger.addHandler(handler_stdout)

    if log_file:
        handler_file = logging.FileHandler(log_file)
        handler_file.setFormatter(fmt)
        logger.addHandler(handler_file)

    logger.setLevel(logging.INFO)


# ---------------------------------------------------------------------------
# Directory snapshot
# ---------------------------------------------------------------------------

def scan_directory(directory):
    """Return dict of {relative_path: mtime} for all watched files."""
    snapshot = {}
    for root, _dirs, files in os.walk(directory):
        for name in files:
            ext = os.path.splitext(name)[1].lower()
            if ext not in WATCHED_EXTENSIONS:
                continue
            full = os.path.join(root, name)
            rel = os.path.relpath(full, directory)
            try:
                snapshot[rel] = os.stat(full).st_mtime
            except OSError:
                pass
    return snapshot


def diff_snapshots(old, new):
    """Compare two snapshots, return (added, modified, deleted) lists."""
    added = [f for f in new if f not in old]
    deleted = [f for f in old if f not in new]
    modified = [
        f for f in new
        if f in old and new[f] != old[f]
    ]
    return added, modified, deleted


# ---------------------------------------------------------------------------
# API interaction
# ---------------------------------------------------------------------------

def check_api_health(api_url):
    """Return True if the RAG API is reachable."""
    try:
        req = urllib.request.Request(f"{api_url}/api/health")
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status == 200
    except Exception:
        return False


def trigger_ingestion(api_url, retries=MAX_RETRIES):
    """POST to the directory ingest endpoint. Returns (success, message)."""
    url = f"{api_url}/api/ingest/directory"
    payload = json.dumps({"path": INGEST_DOCKER_PATH}).encode()

    for attempt in range(1 + retries):
        try:
            req = urllib.request.Request(
                url,
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=120) as resp:
                body = resp.read().decode()
                try:
                    data = json.loads(body)
                    if isinstance(data, list):
                        return True, f"Ingested {len(data)} new document(s)"
                    return True, str(data)
                except json.JSONDecodeError:
                    return True, body
        except urllib.error.HTTPError as e:
            msg = f"HTTP {e.code}: {e.read().decode()[:200]}"
            if attempt < retries:
                logger.warning("Ingestion failed (%s), retrying in 3s...", msg)
                time.sleep(3)
            else:
                return False, msg
        except Exception as e:
            msg = str(e)
            if attempt < retries:
                logger.warning("Ingestion failed (%s), retrying in 3s...", msg)
                time.sleep(3)
            else:
                return False, msg

    return False, "Exhausted retries"


# ---------------------------------------------------------------------------
# Watch loop
# ---------------------------------------------------------------------------

def watch_loop(knowledge_dir, api_url, poll_interval, debounce):
    """Poll the directory and trigger ingestion on changes."""
    logger.info("Watching: %s", knowledge_dir)
    logger.info("API: %s", api_url)
    logger.info("Poll interval: %ds, debounce: %ds", poll_interval, debounce)
    logger.info("Watched extensions: %s", ", ".join(sorted(WATCHED_EXTENSIONS)))

    # Wait for API to be reachable
    while not check_api_health(api_url):
        logger.warning("RAG API not reachable at %s, retrying in 10s...", api_url)
        time.sleep(10)
    logger.info("RAG API is healthy")

    snapshot = scan_directory(knowledge_dir)
    logger.info("Initial snapshot: %d files", len(snapshot))

    last_change_time = None
    pending_changes = {"added": [], "modified": [], "deleted": []}

    while True:
        try:
            time.sleep(poll_interval)
            new_snapshot = scan_directory(knowledge_dir)
            added, modified, deleted = diff_snapshots(snapshot, new_snapshot)

            if added or modified or deleted:
                # Accumulate changes for debouncing
                pending_changes["added"].extend(added)
                pending_changes["modified"].extend(modified)
                pending_changes["deleted"].extend(deleted)
                last_change_time = time.time()
                snapshot = new_snapshot

                for f in added:
                    logger.info("  + %s", f)
                for f in modified:
                    logger.info("  ~ %s", f)
                for f in deleted:
                    logger.info("  - %s", f)
                logger.info("Change detected, waiting %ds debounce...", debounce)

            # Fire ingestion after debounce period
            if last_change_time and (time.time() - last_change_time) >= debounce:
                total = (
                    len(pending_changes["added"])
                    + len(pending_changes["modified"])
                    + len(pending_changes["deleted"])
                )
                logger.info(
                    "Debounce elapsed. Triggering ingestion for %d change(s)...",
                    total,
                )

                success, msg = trigger_ingestion(api_url)
                if success:
                    logger.info("Ingestion OK: %s", msg)
                else:
                    logger.error("Ingestion FAILED: %s", msg)

                # Reset
                last_change_time = None
                pending_changes = {"added": [], "modified": [], "deleted": []}
                # Re-snapshot after ingestion
                snapshot = scan_directory(knowledge_dir)

        except KeyboardInterrupt:
            logger.info("Stopped by user")
            break
        except Exception as e:
            logger.error("Unexpected error: %s", e)
            time.sleep(5)


def run_once(knowledge_dir, api_url):
    """Single ingestion and exit."""
    logger.info("One-shot ingestion from: %s", knowledge_dir)

    if not check_api_health(api_url):
        logger.error("RAG API not reachable at %s", api_url)
        sys.exit(1)

    snapshot = scan_directory(knowledge_dir)
    logger.info("Found %d watched files", len(snapshot))

    success, msg = trigger_ingestion(api_url)
    if success:
        logger.info("Ingestion OK: %s", msg)
    else:
        logger.error("Ingestion FAILED: %s", msg)
        sys.exit(1)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_knowledge = os.path.join(script_dir, "knowledge")

    parser = argparse.ArgumentParser(
        description="Watch knowledge/ for changes and trigger RAG ingestion"
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--once", action="store_true",
        help="Run a single ingestion and exit",
    )
    mode.add_argument(
        "--watch", action="store_true", default=True,
        help="Continuously watch for changes (default)",
    )
    parser.add_argument(
        "--interval", type=int, default=DEFAULT_POLL_INTERVAL,
        help=f"Poll interval in seconds (default: {DEFAULT_POLL_INTERVAL})",
    )
    parser.add_argument(
        "--debounce", type=int, default=DEFAULT_DEBOUNCE,
        help=f"Debounce period in seconds (default: {DEFAULT_DEBOUNCE})",
    )
    parser.add_argument(
        "--api-url", default=None,
        help=f"RAG API URL (default: $RAG_API_URL or {DEFAULT_API_URL})",
    )
    parser.add_argument(
        "--knowledge-dir", default=None,
        help=f"Knowledge directory (default: {default_knowledge})",
    )
    parser.add_argument(
        "--log", default=None, metavar="FILE",
        help="Also log to this file",
    )
    args = parser.parse_args()

    setup_logging(args.log)

    api_url = args.api_url or os.environ.get("RAG_API_URL", DEFAULT_API_URL)
    knowledge_dir = args.knowledge_dir or os.environ.get(
        "KNOWLEDGE_DIR", default_knowledge
    )

    if not os.path.isdir(knowledge_dir):
        logger.error("Knowledge directory not found: %s", knowledge_dir)
        sys.exit(1)

    if args.once:
        run_once(knowledge_dir, api_url)
    else:
        watch_loop(knowledge_dir, api_url, args.interval, args.debounce)


if __name__ == "__main__":
    main()
