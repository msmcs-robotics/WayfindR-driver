"""MCP tool layer for the AMBOT tour guide system.

Provides location-based tools that the chat system can invoke:
- check_location: Query current robot position
- move_to: Navigate to a named location
- list_locations: List all available locations
- find_location: Search for locations by name/description

Also handles intent classification to distinguish:
- Consultation: asking about a location ("tell me about the RASL lab")
- Execution: wanting to go there ("take me to the RASL lab")
"""

import logging
import re
from dataclasses import dataclass
from typing import Optional

from .locations import LocationManager

logger = logging.getLogger(__name__)

# ── Optional Motor Bridge ───────────────────────────────────────
# Module-level motor instance. None means motors not configured (simulation only).
_motor_interface = None  # type: Optional[object]

# Duration (seconds) for the proof-of-concept forward movement on move_to
_MOVE_DURATION = 2.0
_MOVE_SPEED = 30  # percent


def init_motors(simulate: bool = True) -> bool:
    """Initialize the motor interface for MCP tool use.

    Call from main.py at startup. Returns True if motors are ready.
    """
    global _motor_interface
    try:
        from mcp_ability.motor_interface import MotorInterface
        _motor_interface = MotorInterface(simulate=simulate)
        mode = "simulate" if _motor_interface.simulate else "hardware"
        logger.info("MCP motor bridge initialized (%s)", mode)
        return True
    except Exception as exc:
        logger.warning("MCP motor bridge init failed: %s — motor commands will be simulated via state dict only", exc)
        _motor_interface = None
        return False


def get_motor_interface():
    """Return the current motor interface (or None)."""
    return _motor_interface


def cleanup_motors():
    """Clean up motor resources on shutdown."""
    global _motor_interface
    if _motor_interface is not None:
        try:
            _motor_interface.cleanup()
        except Exception as exc:
            logger.warning("Motor cleanup error: %s", exc)
        _motor_interface = None

# ── Intent Classification ────────────────────────────────────────

_CONSULTATION_PATTERNS = [
    r"\b(tell me about|what is|what are|describe|explain|who works|what happens)\b",
    r"\b(information|info about|details|learn about)\b",
    r"\b(where is|which floor|which building|what room)\b",
    r"\b(is there|do they have|does it have)\b",
    r"\b(should i|is it worth|what do you recommend)\b",
]

_EXECUTION_PATTERNS = [
    r"\b(take me to|go to|move to|navigate to|head to)\b",
    r"\b(let'?s go|bring me|walk me|lead me)\b",
    r"\b(i want to go|i('d| would) like to go|i('d| would) like to visit)\b",
    r"\b(can you take me|can we go|show me the way)\b",
]

# Multi-stop route patterns: "take me to A then B", "go to A and then B"
_MULTI_STOP_PATTERNS = [
    r"\bthen\b",
    r"\band then\b",
    r"\bafter that\b",
    r"\bafterwards\b",
]

# Delimiters used to split a multi-stop message into individual location segments
_ROUTE_SPLITTERS = re.compile(
    r",?\s*(?:and then|then|after that|afterwards|next)\s+", re.IGNORECASE
)


@dataclass
class IntentResult:
    """Result of classifying user intent regarding locations."""
    intent: str  # "consultation", "execution", "multi_stop", "none"
    location_query: str  # extracted location name from the message
    confidence: float  # 0.0-1.0
    matched_locations: list[dict]  # locations that match the query
    route_queries: list[str] | None = None  # for multi-stop: ordered location names


def classify_location_intent(message: str, location_mgr: LocationManager) -> IntentResult:
    """Classify whether the user is asking about or wanting to go to a location.

    Priority: consultation overrides execution (safer for a robot).
    Multi-stop detection: "take me to A then B" returns intent="multi_stop".
    """
    lower = message.lower().strip()

    # Check for execution patterns
    is_execution = False
    for pattern in _EXECUTION_PATTERNS:
        if re.search(pattern, lower):
            is_execution = True
            break

    # Check for consultation patterns (overrides execution)
    is_consultation = False
    for pattern in _CONSULTATION_PATTERNS:
        if re.search(pattern, lower):
            is_consultation = True
            break

    # Multi-stop detection: only when execution intent is present
    if is_execution and not is_consultation:
        is_multi = any(re.search(p, lower) for p in _MULTI_STOP_PATTERNS)
        if is_multi:
            route_queries = _extract_multi_location_refs(lower, location_mgr)
            if len(route_queries) >= 2:
                # Resolve each stop to its best match
                all_matched = []
                for rq in route_queries:
                    matches = location_mgr.find(rq)
                    if matches:
                        loc, score = matches[0]
                        all_matched.append({
                            "name": loc.name, "score": round(score, 2),
                            "building": loc.building, "floor": loc.floor,
                            "room": loc.room,
                        })
                if len(all_matched) >= 2:
                    return IntentResult(
                        intent="multi_stop",
                        location_query=route_queries[0],
                        confidence=0.85,
                        matched_locations=all_matched,
                        route_queries=route_queries,
                    )

    # Single-location path (original logic)
    # Try to extract the location reference from the message
    location_query = _extract_location_ref(lower, location_mgr)

    if not location_query:
        return IntentResult(
            intent="none",
            location_query="",
            confidence=0.0,
            matched_locations=[],
        )

    # Find matching locations
    matches = location_mgr.find(location_query)
    matched_dicts = [
        {"name": loc.name, "score": round(score, 2),
         "building": loc.building, "floor": loc.floor, "room": loc.room}
        for loc, score in matches[:5]
    ]

    if is_consultation or (not is_execution):
        return IntentResult(
            intent="consultation",
            location_query=location_query,
            confidence=0.8 if is_consultation else 0.5,
            matched_locations=matched_dicts,
        )

    return IntentResult(
        intent="execution",
        location_query=location_query,
        confidence=0.9 if len(matched_dicts) == 1 and matched_dicts[0]["score"] > 0.8 else 0.6,
        matched_locations=matched_dicts,
    )


def _extract_location_ref(message: str, location_mgr: LocationManager) -> str:
    """Try to extract a location reference from a message.

    Checks against known location names, then falls back to
    extracting text after common prepositions.
    """
    # Check if any known location name appears in the message
    best_match = ""
    best_score = 0.0
    for loc in location_mgr.locations:
        for name in loc.all_names:
            nl = name.lower()
            if nl in message:
                score = len(nl) / len(message) if message else 0
                if score > best_score or len(nl) > len(best_match):
                    best_match = name
                    best_score = score

    if best_match:
        return best_match

    # Extract text after location prepositions
    for pattern in [
        r"(?:to|about|at|in|visit|see)\s+(?:the\s+)?(.{3,50}?)(?:\?|$|\.)",
        r"(?:where is|find)\s+(?:the\s+)?(.{3,50}?)(?:\?|$|\.)",
    ]:
        m = re.search(pattern, message)
        if m:
            return m.group(1).strip()

    return ""


def _extract_multi_location_refs(message: str, location_mgr: LocationManager) -> list[str]:
    """Extract multiple location references from a multi-stop message.

    Splits on route delimiters ("then", "and then", "after that", etc.)
    and extracts a location reference from each segment.
    """
    # Strip leading "first" if present (e.g. "first go to A, then B")
    cleaned = re.sub(r"^\s*first\s+", "", message, flags=re.IGNORECASE)

    # Split on route delimiters
    segments = _ROUTE_SPLITTERS.split(cleaned)

    refs = []
    for seg in segments:
        seg = seg.strip()
        if not seg:
            continue
        ref = _extract_location_ref(seg, location_mgr)
        if ref:
            refs.append(ref)

    return refs


# ── MCP Tool Execution ───────────────────────────────────────────

# Robot state (will be replaced with real data when locomotion is integrated)
_robot_state = {
    "current_location": None,
    "moving_to": None,
    "status": "idle",
}


@dataclass
class MCPAction:
    """An MCP action that was executed or queued."""
    tool: str  # "move_to", "check_location", etc.
    args: dict
    result: dict
    display_text: str  # Human-readable text for the chat UI


def execute_check_location() -> MCPAction:
    """Check the robot's current location."""
    loc = _robot_state["current_location"]
    status = _robot_state["status"]

    if loc:
        display = f"Current location: {loc}"
    else:
        display = "Location: unknown (no position data)"

    if status == "moving":
        display += f" → Moving to: {_robot_state['moving_to']}"

    return MCPAction(
        tool="check_location",
        args={},
        result={"location": loc, "status": status, "moving_to": _robot_state["moving_to"]},
        display_text=display,
    )


def execute_move_to(location_name: str, location_mgr: LocationManager) -> MCPAction:
    """Command the robot to navigate to a location."""
    loc = location_mgr.get_by_name(location_name)

    if not loc:
        # Try fuzzy match
        matches = location_mgr.find(location_name, threshold=0.5)
        if matches:
            loc = matches[0][0]

    if not loc:
        return MCPAction(
            tool="move_to",
            args={"location": location_name},
            result={"success": False, "error": f"Unknown location: {location_name}"},
            display_text=f"Unknown location: {location_name}",
        )

    # Update robot state
    _robot_state["moving_to"] = loc.name
    _robot_state["status"] = "moving"

    where = loc.building
    if loc.floor:
        where += f", Floor {loc.floor}"
    if loc.room:
        where += f", Room {loc.room}"

    # Attempt real motor movement if motors are available
    motor_result = None
    if _motor_interface is not None:
        try:
            motor_result = _motor_interface.drive(
                left_speed=_MOVE_SPEED,
                right_speed=_MOVE_SPEED,
                duration=_MOVE_DURATION,
                label=f"move_to:{loc.name}",
            )
            logger.info("Motor drive triggered for move_to '%s': %s", loc.name, motor_result)
        except Exception as exc:
            logger.warning("Motor drive failed for move_to '%s': %s — continuing with simulation", loc.name, exc)
            motor_result = None

    result = {
        "success": True,
        "destination": loc.name,
        "building": loc.building,
        "floor": loc.floor,
        "room": loc.room,
        "motors": "active" if motor_result else "simulated",
    }

    return MCPAction(
        tool="move_to",
        args={"location": loc.name},
        result=result,
        display_text=f"Navigating to: {loc.name} ({where})",
    )


def execute_list_locations(location_mgr: LocationManager) -> MCPAction:
    """List all available locations."""
    locs = location_mgr.list_all()
    names = [l["name"] for l in locs]
    return MCPAction(
        tool="list_locations",
        args={},
        result={"locations": locs, "count": len(locs)},
        display_text=f"Available locations: {', '.join(names)}",
    )


def execute_waypoint_route(
    location_names: list[str], location_mgr: LocationManager
) -> MCPAction:
    """Plan a multi-stop waypoint route.

    Resolves each location name to a Location object and returns an
    MCPAction with the full route plan. Does NOT begin navigation —
    actual movement between stops is future work.
    """
    stops = []
    errors = []

    for name in location_names:
        loc = location_mgr.get_by_name(name)
        if not loc:
            matches = location_mgr.find(name, threshold=0.5)
            if matches:
                loc = matches[0][0]

        if loc:
            where = loc.building
            if loc.floor:
                where += f", Floor {loc.floor}"
            if loc.room:
                where += f", Room {loc.room}"
            stops.append({
                "name": loc.name,
                "building": loc.building,
                "floor": loc.floor,
                "room": loc.room,
                "where": where,
            })
        else:
            errors.append(name)

    if not stops:
        return MCPAction(
            tool="waypoint_route",
            args={"locations": location_names},
            result={"success": False, "error": f"No valid locations found: {', '.join(errors)}"},
            display_text=f"Could not plan route — unknown locations: {', '.join(errors)}",
        )

    # Build display text
    route_str = " -> ".join(s["name"] for s in stops)
    display = f"Route planned ({len(stops)} stops): {route_str}"
    if errors:
        display += f"\n  (unknown: {', '.join(errors)})"

    return MCPAction(
        tool="waypoint_route",
        args={"locations": location_names},
        result={
            "success": True,
            "stops": stops,
            "stop_count": len(stops),
            "errors": errors,
        },
        display_text=display,
    )
