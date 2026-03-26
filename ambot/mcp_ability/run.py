#!/usr/bin/env python3
"""
AMBOT Motor Control MCP Server — Entry point.

Usage:
    # Simulate mode (no hardware), stdio transport (default for Claude Desktop)
    python -m mcp_ability.run

    # Simulate mode, SSE transport on port 8100
    python -m mcp_ability.run --transport sse --port 8100

    # Hardware mode on RPi
    python -m mcp_ability.run --no-simulate --transport sse --port 8100
"""

import argparse
import logging
import sys


def main():
    parser = argparse.ArgumentParser(description="AMBOT Motor Control MCP Server")
    parser.add_argument("--simulate", action="store_true", default=True,
                        help="Simulate motor commands (default: True)")
    parser.add_argument("--no-simulate", action="store_true",
                        help="Use real hardware motors")
    parser.add_argument("--host", default="0.0.0.0",
                        help="Host for SSE transport (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8100,
                        help="Port for SSE transport (default: 8100)")
    parser.add_argument("--transport", choices=["stdio", "sse"], default="stdio",
                        help="MCP transport (default: stdio)")
    parser.add_argument("--auto-stop", type=float, default=30.0,
                        help="Auto-stop timeout in seconds (default: 30, 0 to disable)")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args()

    # Configure logging
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        stream=sys.stderr,  # MCP stdio uses stdout for protocol
    )

    simulate = not args.no_simulate

    # Initialize motors before starting server
    from .server import init_motors, mcp
    init_motors(simulate=simulate, auto_stop_timeout=args.auto_stop)

    mode = "SIMULATE" if simulate else "HARDWARE"
    logging.getLogger(__name__).info(
        "Starting AMBOT Motor MCP — transport=%s, mode=%s", args.transport, mode
    )

    # Run the FastMCP server
    if args.transport == "sse":
        mcp.run(transport="sse", host=args.host, port=args.port)
    else:
        mcp.run(transport="stdio")


if __name__ == "__main__":
    main()
