#!/usr/bin/env python3
"""AMBOT Conversation Loop — interactive RAG-powered chat.

Connects to the RAG API and provides a text-in/text-out conversation
interface.  Runs on the Jetson directly or from any machine that can
reach the API.

Usage:
    python3 chat.py                        # defaults to localhost:8000
    python3 chat.py --url http://10.33.255.82:8000
    python3 chat.py --no-sources           # hide source citations
    python3 chat.py --system "You are a helpful robot named AMBOT"
"""

from __future__ import annotations

import argparse
import json
import sys
import textwrap
from urllib.request import Request, urlopen
from urllib.error import URLError


def ask(api_url: str, question: str, system_prompt: str | None = None) -> dict:
    """Send a question to the RAG API and return the response dict."""
    payload = {"question": question}
    if system_prompt:
        payload["system_prompt"] = system_prompt

    data = json.dumps(payload).encode()
    req = Request(
        f"{api_url}/api/ask",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(req, timeout=120) as resp:
        return json.loads(resp.read().decode())


def health(api_url: str) -> dict:
    """Check API health."""
    req = Request(f"{api_url}/api/health")
    with urlopen(req, timeout=10) as resp:
        return json.loads(resp.read().decode())


def format_answer(response: dict, show_sources: bool = True) -> str:
    """Format the LLM response for terminal display."""
    lines = []
    lines.append(response["answer"])

    if show_sources and response.get("sources"):
        lines.append("")
        lines.append(f"  [{response['model']}] — {len(response['sources'])} source(s)")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="AMBOT conversation loop")
    parser.add_argument(
        "--url",
        default="http://localhost:8000",
        help="RAG API base URL (default: http://localhost:8000)",
    )
    parser.add_argument(
        "--system",
        default=None,
        help="System prompt for the LLM",
    )
    parser.add_argument(
        "--no-sources",
        action="store_true",
        help="Hide source citations in output",
    )
    args = parser.parse_args()

    api_url = args.url.rstrip("/")

    # Health check
    try:
        h = health(api_url)
        if h["status"] != "healthy":
            print(f"Warning: API status is '{h['status']}'")
    except (URLError, ConnectionError) as e:
        print(f"Cannot reach API at {api_url}: {e}")
        print("Start the RAG stack first, or use --url to specify the correct address.")
        sys.exit(1)

    print("AMBOT Chat — type your question, or 'quit' to exit.")
    if args.system:
        print(f"System prompt: {args.system}")
    print()

    while True:
        try:
            question = input("You: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nBye!")
            break

        if not question:
            continue
        if question.lower() in ("quit", "exit", "q"):
            print("Bye!")
            break

        try:
            response = ask(api_url, question, args.system)
            print()
            print(format_answer(response, show_sources=not args.no_sources))
            print()
        except URLError as e:
            print(f"\nAPI error: {e}\n")
        except json.JSONDecodeError:
            print("\nInvalid response from API\n")


if __name__ == "__main__":
    main()
