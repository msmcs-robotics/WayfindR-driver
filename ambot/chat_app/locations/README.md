# AMBOT Locations

This directory contains location definitions for the AMBOT tour guide system.
Each markdown file defines rooms/locations that the robot can navigate to.

## Format

Each location is a section in a markdown file. The format is:

```markdown
## Location Name
- **aliases**: Other Name, Another Name
- **building**: Building Name
- **floor**: 3
- **room**: 340A

Description paragraph goes here. This can be multiple sentences describing
what the room is, what it's used for, who works there, etc. The description
is used by the LLM to answer questions about the location and to help match
user requests to the right place.
```

## Rules

- Each `##` heading starts a new location
- The heading text is the primary name
- **aliases** (optional): comma-separated alternative names people might use
- **building** (required): which building this is in
- **floor** (optional): floor number
- **room** (optional): room number or identifier
- Everything after the metadata lines is the description (plain text)
- Files can contain multiple locations
- Organize by building or area as you prefer
