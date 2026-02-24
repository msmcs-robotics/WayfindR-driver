#!/usr/bin/env python3
"""Show ingested RAG documents."""
import json
import sys
import urllib.request

api_url = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:8000"

try:
    with urllib.request.urlopen(f"{api_url}/api/documents", timeout=10) as resp:
        docs = json.loads(resp.read().decode())
except Exception as e:
    print(f"  Error: {e}", file=sys.stderr)
    sys.exit(1)

if not docs:
    print("  (no documents)")
else:
    for d in docs:
        print(f"  [{d['id']}] {d['filename']} ({d['chunk_count']} chunks, {d['file_size']} bytes)")
    total_chunks = sum(d["chunk_count"] for d in docs)
    print(f"\n  Total: {len(docs)} documents, {total_chunks} chunks")
