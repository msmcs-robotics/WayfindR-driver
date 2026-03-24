# Location Files

Each markdown file in this directory defines a building and its rooms.
These files are:
1. **Parsed by the chat app** for MCP tour guide navigation
2. **Ingested into RAG** so the LLM can answer questions about locations

## File Format

Each file represents one building. Rooms are `##` sections.

```markdown
# Building Name

General description of the building.

## Room Name
- **also known as**: Alias 1, Alias 2, Short Name
- **floor**: 3
- **room**: 340A

Description paragraph. Can be multiple sentences describing what the
room is used for, who works there, what equipment is available, etc.
```

## Rules

- `#` heading = building name (one per file)
- `##` heading = room/location name
- `also known as` = comma-separated aliases (how people commonly refer to it)
- `floor` = floor number (integer or "G" for ground)
- `room` = room number or identifier (optional)
- Everything after the metadata lines is the description
- Keep descriptions concise but informative (2-5 sentences)
- One building per file, named after the building (lowercase, hyphens)
