#!/usr/bin/env python3
"""RAG System Test Suite — runs against the RAG API.

Usage:
    python3 rag-test.py                  # Test against localhost:8000
    python3 rag-test.py http://10.33.255.82:8000  # Test against remote
    python3 rag-test.py --json           # JSON output for automation
"""

import json
import sys
import time
import urllib.request
import urllib.error

API_URL = "http://localhost:8000"
JSON_OUTPUT = False

GREEN = "\033[0;32m"
RED = "\033[0;31m"
NC = "\033[0m"

results = []


def api_get(path: str, timeout: int = 15) -> dict:
    req = urllib.request.Request(f"{API_URL}{path}")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode())


def api_post(path: str, payload: dict, timeout: int = 120) -> dict:
    data = json.dumps(payload).encode()
    req = urllib.request.Request(
        f"{API_URL}{path}",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode())


def run_test(name: str, fn) -> bool:
    if not JSON_OUTPUT:
        print(f"  [{len(results) + 1}] {name}... ", end="", flush=True)
    try:
        fn()
        if not JSON_OUTPUT:
            print(f"{GREEN}PASS{NC}")
        results.append({"name": name, "status": "pass"})
        return True
    except Exception as e:
        if not JSON_OUTPUT:
            print(f"{RED}FAIL{NC}")
            msg = str(e)
            if msg:
                print(f"       {msg[:120]}")
        results.append({"name": name, "status": "fail", "error": str(e)[:200]})
        return False


def test_health():
    h = api_get("/api/health")
    assert h["status"] == "healthy", f"Status: {h['status']}"


def test_database():
    h = api_get("/api/health")
    assert h["database"] is True, "Database unhealthy"


def test_redis():
    h = api_get("/api/health")
    assert h["redis"] is True, "Redis unhealthy"


def test_embeddings():
    h = api_get("/api/health")
    assert h["embedding_service"] is True, "Embedding service unhealthy"


def test_llm():
    h = api_get("/api/health")
    assert h["llm"] is True, "LLM backend unhealthy"


def test_documents():
    docs = api_get("/api/documents")
    assert isinstance(docs, list), f"Expected list, got {type(docs)}"


def test_keyword_search():
    resp = api_post("/api/search", {"query": "robot", "mode": "keyword", "limit": 3})
    assert isinstance(resp, list), f"Expected list, got {type(resp)}"


def test_hybrid_search():
    resp = api_post("/api/search", {"query": "obstacle avoidance", "mode": "hybrid", "limit": 3})
    assert isinstance(resp, list), f"Expected list, got {type(resp)}"


def test_ask():
    resp = api_post("/api/ask", {"question": "What hardware does AMBOT use?"}, timeout=120)
    assert "answer" in resp, "Missing answer field"
    assert "model" in resp, "Missing model field"
    assert len(resp["answer"]) > 10, f"Answer too short: {len(resp['answer'])} chars"


def test_models():
    resp = api_get("/api/models")
    assert "models" in resp, "Missing models field"
    assert "current" in resp, "Missing current field"


def main():
    global API_URL, JSON_OUTPUT

    for arg in sys.argv[1:]:
        if arg.startswith("http"):
            API_URL = arg.rstrip("/")
        elif arg == "--json":
            JSON_OUTPUT = True

    if not JSON_OUTPUT:
        print(f"\n  RAG Test Suite — {API_URL}\n")

    tests = [
        ("API health check", test_health),
        ("Database connection", test_database),
        ("Redis connection", test_redis),
        ("Embedding service", test_embeddings),
        ("LLM backend (Ollama)", test_llm),
        ("Document listing", test_documents),
        ("Keyword search", test_keyword_search),
        ("Hybrid search", test_hybrid_search),
        ("RAG ask pipeline", test_ask),
        ("Models listing", test_models),
    ]

    start = time.time()
    for name, fn in tests:
        run_test(name, fn)
    elapsed = time.time() - start

    passed = sum(1 for r in results if r["status"] == "pass")
    failed = sum(1 for r in results if r["status"] == "fail")
    total = len(results)

    if JSON_OUTPUT:
        print(json.dumps({
            "passed": passed,
            "failed": failed,
            "total": total,
            "elapsed": round(elapsed, 1),
            "tests": results,
        }))
    else:
        print(f"\n  {'=' * 39}")
        if failed == 0:
            print(f"  {GREEN}All {total} tests passed{NC} ({elapsed:.1f}s)")
        else:
            print(f"  {GREEN}{passed} passed{NC}, {RED}{failed} failed{NC} (of {total}, {elapsed:.1f}s)")
        print(f"  {'=' * 39}\n")

    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
