"""Tests for query classification logic in chat_app.main.classify_query.

Run:  python -m pytest chat_app/test_classify.py -v
  or: python chat_app/test_classify.py
"""

import sys
import os

# Allow running from ambot/ directory without installing chat_app as a package
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from chat_app.main import classify_query


# ── Helpers ─────────────────────────────────────────────────────
def _assert_casual(query, **kwargs):
    result = classify_query(query, **kwargs)
    assert result == "casual", f"Expected 'casual' for {query!r}, got {result!r}"


def _assert_rag(query, **kwargs):
    result = classify_query(query, **kwargs)
    assert result == "rag", f"Expected 'rag' for {query!r}, got {result!r}"


# ── Greetings & Farewells (casual) ─────────────────────────────
def test_greetings():
    for q in ["hi", "Hi!", "hello", "Hello!", "hey", "Hey?", "yo", "greetings"]:
        _assert_casual(q)


def test_farewells():
    for q in ["bye", "goodbye", "see you", "Bye!"]:
        _assert_casual(q)


def test_good_time_of_day():
    for q in ["good morning", "Good Morning!", "good afternoon", "good evening"]:
        _assert_casual(q)


# ── Acknowledgments (casual) ──────────────────────────────────
def test_acknowledgments():
    for q in ["thanks", "thank you", "Thank you!", "ok", "okay", "cool",
              "nice", "great", "sure", "yes", "no"]:
        _assert_casual(q)


def test_reactions():
    for q in ["lol", "haha", "hmm", "wow", "Wow!"]:
        _assert_casual(q)


# ── Short casual with filler (casual) ─────────────────────────
def test_greeting_with_filler():
    """'hi there', 'hey!' etc. should stay casual."""
    for q in ["hi there", "hey!", "hello there", "thanks a lot",
              "good morning!", "thank you so much"]:
        _assert_casual(q)


# ── System / capability questions (casual) ─────────────────────
def test_capability_questions():
    for q in ["help", "what can you do", "what do you do", "What can you do?"]:
        _assert_casual(q)


# ── Out-of-scope questions (casual) ───────────────────────────
def test_out_of_scope():
    for q in ["what time is it", "What time is it?", "what day is it",
              "how's the weather", "what's the weather like"]:
        _assert_casual(q)


# ── Empty / whitespace (casual) ───────────────────────────────
def test_empty():
    _assert_casual("")
    _assert_casual("   ")


# ── Follow-ups with history (casual) ──────────────────────────
def test_followups_with_history():
    for q in ["tell me more", "why?", "elaborate", "go on", "what else",
              "how so", "can you explain"]:
        _assert_casual(q, has_history=True)


def test_followups_without_history_short():
    """Without history, very short follow-ups still classify as casual (<=2 words)."""
    _assert_casual("why?")
    _assert_casual("more")


# ── Greeting + real question (RAG) ────────────────────────────
def test_greeting_prefix_with_domain_query():
    """Greeting followed by a substantive question should trigger RAG."""
    for q in ["hi where is the robotics lab",
              "hey what courses are available",
              "hello who teaches robotics",
              "good morning where is the eecs department"]:
        _assert_rag(q)


def test_acknowledgment_prefix_with_domain_query():
    """'ok' / 'sure' followed by domain question should trigger RAG."""
    for q in ["ok what is the eecs department",
              "sure tell me about the robotics program"]:
        _assert_rag(q)


# ── Domain knowledge questions (RAG) ──────────────────────────
def test_faculty_questions():
    for q in ["who teaches robotics", "who is the professor for EECS",
              "tell me about the faculty"]:
        _assert_rag(q)


def test_location_questions():
    for q in ["where is the robotics lab", "where is the eecs department",
              "where is campus"]:
        _assert_rag(q)


def test_program_questions():
    for q in ["what programs does ERAU offer",
              "tell me about the engineering degree",
              "what courses are in the robotics program"]:
        _assert_rag(q)


def test_domain_keyword_short():
    """Even short queries with domain keywords should trigger RAG."""
    for q in ["robotics lab", "eecs department", "erau campus"]:
        _assert_rag(q)


def test_general_knowledge_questions():
    """Longer question-format queries without specific domain keywords."""
    for q in ["what are the admission requirements for this university",
              "how do I apply to the graduate school here"]:
        # These are 5+ words with question starters → RAG
        _assert_rag(q)


# ── How are you (casual, not RAG) ─────────────────────────────
def test_how_are_you():
    """'how are you' should be casual even though it starts with 'how'."""
    _assert_casual("how are you")
    _assert_casual("How are you?")


# ── Run directly ──────────────────────────────────────────────
if __name__ == "__main__":
    import inspect
    passed = 0
    failed = 0
    errors = []

    test_funcs = [
        (name, obj) for name, obj in sorted(globals().items())
        if name.startswith("test_") and callable(obj)
    ]

    for name, func in test_funcs:
        try:
            func()
            passed += 1
            print(f"  PASS  {name}")
        except AssertionError as e:
            failed += 1
            errors.append((name, str(e)))
            print(f"  FAIL  {name}: {e}")
        except Exception as e:
            failed += 1
            errors.append((name, str(e)))
            print(f"  ERROR {name}: {e}")

    print(f"\n{passed} passed, {failed} failed")
    if errors:
        print("\nFailures:")
        for name, msg in errors:
            print(f"  {name}: {msg}")
        sys.exit(1)
