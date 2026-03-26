#!/usr/bin/env python3
"""
AMBOT Motor Chat — Interactive CLI for LLM-controlled motors.

Talk to the robot in natural language. The LLM decides when to use
motor control tools.

Usage:
    python3 -m mcp_ability.chat                          # Simulate mode
    python3 -m mcp_ability.chat --no-simulate             # Real hardware
    python3 -m mcp_ability.chat --model llama3.2:3b       # Specific model
    python3 -m mcp_ability.chat --url http://localhost:11434  # Custom Ollama URL

Example conversation:
    You: Move forward
    AMBOT: Moving forward at 50% speed.
    You: Turn right for 2 seconds
    AMBOT: Turning right for 2 seconds.
    You: Stop
    AMBOT: Motors stopped.
"""

import argparse
import logging
import sys


def main():
    parser = argparse.ArgumentParser(description="AMBOT Motor Chat")
    parser.add_argument("--simulate", action="store_true", default=True,
                        help="Simulate motors (default)")
    parser.add_argument("--no-simulate", dest="simulate", action="store_false",
                        help="Use real hardware")
    parser.add_argument("--model", default="llama3.2:3b",
                        help="Ollama model (default: llama3.2:3b)")
    parser.add_argument("--url", default="http://localhost:11434",
                        help="Ollama API URL")
    parser.add_argument("--auto-stop", type=float, default=30.0,
                        help="Auto-stop timeout in seconds (default: 30)")
    parser.add_argument("--log-level", default="WARNING",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    # Add parent dir to path so imports work
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    from mcp_ability.ollama_bridge import AmbotAgent

    mode = "SIMULATE" if args.simulate else "HARDWARE"
    print(f"AMBOT Motor Chat ({mode} mode)")
    print(f"Model: {args.model} @ {args.url}")
    print(f"Auto-stop: {args.auto_stop}s")
    print("Type 'quit' to exit, 'reset' to clear history, 'status' for motor state")
    print("-" * 50)

    agent = AmbotAgent(
        simulate=args.simulate,
        model=args.model,
        ollama_url=args.url,
        auto_stop_timeout=args.auto_stop,
    )

    try:
        while True:
            try:
                user_input = input("\nYou: ").strip()
            except EOFError:
                break

            if not user_input:
                continue
            if user_input.lower() in ("quit", "exit", "q"):
                break
            if user_input.lower() == "reset":
                agent.reset()
                print("Conversation reset.")
                continue
            if user_input.lower() == "status":
                status = agent.motors.get_status()
                print(f"Motor state: {status['motor_state']}")
                continue

            response = agent.chat(user_input)
            print(f"\nAMBOT: {response}")

    except KeyboardInterrupt:
        print("\n\nInterrupted")
    finally:
        agent.cleanup()
        print("Motors stopped. Goodbye.")


if __name__ == "__main__":
    main()
