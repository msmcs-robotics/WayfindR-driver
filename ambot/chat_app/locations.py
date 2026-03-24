"""Location parser and manager for the AMBOT tour guide system.

Reads markdown files from the locations/ directory and provides
lookup, search, and matching functions for the MCP tool layer.
"""

import logging
import os
import re
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass
class Location:
    """A named location the robot can navigate to."""
    name: str
    aliases: list[str] = field(default_factory=list)
    building: str = ""
    floor: str = ""
    room: str = ""
    description: str = ""

    @property
    def all_names(self) -> list[str]:
        """All names this location is known by (primary + aliases)."""
        return [self.name] + self.aliases

    def matches(self, query: str) -> float:
        """Score how well this location matches a query string (0.0-1.0)."""
        q = query.lower().strip()
        # Exact match on any name
        for name in self.all_names:
            if q == name.lower():
                return 1.0
        # Partial match (query contained in a name or vice versa)
        best = 0.0
        for name in self.all_names:
            nl = name.lower()
            if q in nl or nl in q:
                score = min(len(q), len(nl)) / max(len(q), len(nl))
                best = max(best, score * 0.9)
        # Room number match
        if self.room and q == self.room.lower():
            best = max(best, 0.85)
        # Word overlap
        q_words = set(q.split())
        for name in self.all_names:
            n_words = set(name.lower().split())
            overlap = len(q_words & n_words)
            if overlap > 0:
                score = overlap / max(len(q_words), len(n_words))
                best = max(best, score * 0.7)
        return best


class LocationManager:
    """Manages all known locations, loaded from markdown files."""

    def __init__(self, locations_dir: str | None = None):
        self.locations_dir = locations_dir or os.path.join(
            os.path.dirname(__file__), "locations"
        )
        self.locations: list[Location] = []
        self.load()

    def load(self):
        """Load all location markdown files."""
        self.locations = []
        if not os.path.isdir(self.locations_dir):
            logger.warning("Locations directory not found: %s", self.locations_dir)
            return

        for fname in sorted(os.listdir(self.locations_dir)):
            if not fname.endswith(".md") or fname == "README.md":
                continue
            path = os.path.join(self.locations_dir, fname)
            try:
                locs = parse_locations_file(path)
                self.locations.extend(locs)
                logger.info("Loaded %d locations from %s", len(locs), fname)
            except Exception as e:
                logger.warning("Failed to parse %s: %s", fname, e)

        logger.info("Total locations loaded: %d", len(self.locations))

    def find(self, query: str, threshold: float = 0.4) -> list[tuple[Location, float]]:
        """Find locations matching a query, sorted by match score.

        Returns list of (location, score) tuples above threshold.
        """
        results = []
        for loc in self.locations:
            score = loc.matches(query)
            if score >= threshold:
                results.append((loc, score))
        results.sort(key=lambda x: x[1], reverse=True)
        return results

    def get_by_name(self, name: str) -> Location | None:
        """Get a location by exact name or alias match."""
        lower = name.lower().strip()
        for loc in self.locations:
            if lower == loc.name.lower():
                return loc
            for alias in loc.aliases:
                if lower == alias.lower():
                    return loc
        return None

    def list_all(self) -> list[dict]:
        """Return all locations as dicts for API responses."""
        return [
            {
                "name": loc.name,
                "aliases": loc.aliases,
                "building": loc.building,
                "floor": loc.floor,
                "room": loc.room,
                "description": loc.description[:200] + "..." if len(loc.description) > 200 else loc.description,
            }
            for loc in self.locations
        ]

    def format_for_llm(self) -> str:
        """Format all locations as a concise reference for LLM context."""
        lines = ["Available locations:"]
        for loc in self.locations:
            names = ", ".join(loc.all_names)
            where = loc.building
            if loc.floor:
                where += f" (Floor {loc.floor})"
            if loc.room:
                where += f" Room {loc.room}"
            lines.append(f"- {names} — {where}")
        return "\n".join(lines)


def parse_locations_file(path: str) -> list[Location]:
    """Parse a markdown file into Location objects."""
    with open(path) as f:
        content = f.read()

    locations = []
    # Split on ## headings (level 2)
    sections = re.split(r'^## ', content, flags=re.MULTILINE)

    for section in sections[1:]:  # Skip preamble before first ##
        lines = section.strip().split("\n")
        if not lines:
            continue

        name = lines[0].strip()
        aliases = []
        building = ""
        floor_val = ""
        room = ""
        desc_lines = []

        for line in lines[1:]:
            stripped = line.strip()
            if stripped.startswith("- **aliases**:"):
                alias_str = stripped.split(":", 1)[1].strip()
                aliases = [a.strip() for a in alias_str.split(",") if a.strip()]
            elif stripped.startswith("- **building**:"):
                building = stripped.split(":", 1)[1].strip()
            elif stripped.startswith("- **floor**:"):
                floor_val = stripped.split(":", 1)[1].strip()
            elif stripped.startswith("- **room**:"):
                room = stripped.split(":", 1)[1].strip()
            elif stripped and not stripped.startswith("- **"):
                desc_lines.append(stripped)

        description = " ".join(desc_lines).strip()

        if name:
            locations.append(Location(
                name=name,
                aliases=aliases,
                building=building,
                floor=floor_val,
                room=room,
                description=description,
            ))

    return locations
