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

from .locations import LocationManager

logger = logging.getLogger(__name__)

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


@dataclass
class IntentResult:
    """Result of classifying user intent regarding locations."""
    intent: str  # "consultation", "execution", "none"
    location_query: str  # extracted location name from the message
    confidence: float  # 0.0-1.0
    matched_locations: list[dict]  # locations that match the query


def classify_location_intent(message: str, location_mgr: LocationManager) -> IntentResult:
    """Classify whether the user is asking about or wanting to go to a location.

    Priority: consultation overrides execution (safer for a robot).
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

    # If neither, check if any known location is mentioned
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

    # Update robot state (simulated — real integration with locomotion later)
    _robot_state["moving_to"] = loc.name
    _robot_state["status"] = "moving"

    where = loc.building
    if loc.floor:
        where += f", Floor {loc.floor}"
    if loc.room:
        where += f", Room {loc.room}"

    return MCPAction(
        tool="move_to",
        args={"location": loc.name},
        result={"success": True, "destination": loc.name, "building": loc.building,
                "floor": loc.floor, "room": loc.room},
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
