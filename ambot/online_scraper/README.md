# ERAU Online Scraper

Scrapes public-facing ERAU web pages for ingestion into the AMBOT RAG knowledge base.

## Quick Start

```bash
cd ambot/online_scraper

# Full pipeline: scrape all departments + college pages, clean, show diff
./update_knowledge.sh

# Same, but copy results to the knowledge/ folder
./update_knowledge.sh --apply

# Full pipeline including Jetson re-ingestion
./update_knowledge.sh --apply --ingest
```

## Scripts

| Script | Purpose |
|--------|---------|
| `update_knowledge.sh` | Master pipeline: scrape, clean, diff, apply, ingest |
| `scrape_faculty_profiles.py` | Scrapes faculty.erau.edu individual profiles |
| `scraper.py` | General page crawler for daytonabeach.erau.edu |
| `clean_faculty_profiles.py` | Removes boilerplate from faculty profile markdown |
| `cleaner.py` | Removes boilerplate from general scraped pages |
| `scrape_config.py` | Shared config: departments, URLs, defaults |

## update_knowledge.sh Usage

```bash
./update_knowledge.sh                    # Scrape + clean + show diff (dry run)
./update_knowledge.sh --apply            # Scrape + clean + copy to knowledge/
./update_knowledge.sh --apply --ingest   # Full pipeline: scrape -> clean -> copy -> re-ingest
./update_knowledge.sh --resume           # Resume an interrupted scrape
./update_knowledge.sh --faculty-only     # Only scrape faculty profiles
./update_knowledge.sh --college-only     # Only scrape college pages
./update_knowledge.sh --skip-scrape      # Skip scraping, just clean + diff existing staging
```

### What it does

1. Clears `/tmp/erau-faculty-staging/` and `/tmp/erau-scrape-staging/`
2. Runs the faculty profile scraper (`--all-departments`)
3. Runs the college page crawler
4. Cleans both outputs (removes nav/footer boilerplate)
5. Shows a diff summary: new files, changed files, unchanged files
6. With `--apply`: copies cleaned files to `bootylicious/rag/knowledge/college/`
7. With `--ingest`: rsyncs to Jetson and runs `ingest.sh`

The script is idempotent. Running it again overwrites staging and produces a fresh diff.

## Faculty Scraper Features

- **Resume**: `--resume` skips profiles that already have output files (for interrupted scrapes)
- **Rate limiting**: `--delay N` (default 0.5s between requests)
- **Error logging**: Errors logged to `scrape.log` in the output directory
- **Dry run**: `--dry-run` lists URLs without fetching

```bash
python3 scrape_faculty_profiles.py --all-departments           # All departments
python3 scrape_faculty_profiles.py --department DCOEAE         # Aerospace only
python3 scrape_faculty_profiles.py --resume --all-departments  # Resume interrupted
python3 scrape_faculty_profiles.py --list-departments          # Show department codes
```

## Adding New Departments

Edit `scrape_config.py` and add the department code to the `DEPARTMENTS` dict:

```python
DEPARTMENTS = {
    "DCOEECSSE": "Electrical Engineering and Computer Science",
    "DCOEAE":    "Aerospace Engineering",
    # Add new ones here:
    "DCOASPHY":  "Physics",
}
```

Department codes are visible in faculty.erau.edu URLs (e.g., `/departments/DCOEECSSE`).

## Adding New Scraping Targets

For entirely new data sources (labs, course catalog, etc.):

1. Add an entry to `SCRAPE_TARGETS` in `scrape_config.py`:

```python
SCRAPE_TARGETS = {
    # ... existing entries ...
    "catalog": {
        "name": "catalog",
        "description": "Course catalog entries",
        "base_url": "https://catalog.erau.edu/",
        "scraper": "scrape_catalog.py",
        "output_dir": "/tmp/erau-catalog-staging",
        "knowledge_dest": "programs/",
        "enabled": True,
    },
}
```

2. Create the scraper script (or reuse `scraper.py` with a different base URL).
3. Add a scraping step to `update_knowledge.sh` if needed.

## Staging Workflow

All scrapers write to `/tmp/` staging directories, not directly to the knowledge base. This lets you review output before committing:

```
/tmp/erau-faculty-staging/     <- faculty profiles
/tmp/erau-scrape-staging/      <- college pages
```

The `--apply` flag copies cleaned staging to `bootylicious/rag/knowledge/`. Without it, nothing touches the knowledge base.

## Configuration Reference

All shared constants live in `scrape_config.py`:

| Setting | Default | Description |
|---------|---------|-------------|
| `FACULTY_BASE_URL` | `https://faculty.erau.edu` | Faculty directory root |
| `COLLEGE_BASE_URL` | `https://daytonabeach.erau.edu/college-engineering/` | College pages root |
| `DEPARTMENTS` | 5 CoE departments | Department codes and names |
| `DEFAULT_DELAY_SECONDS` | 0.5 | Faculty scraper request delay |
| `DEFAULT_CRAWL_DELAY_SECONDS` | 1.5 | General scraper request delay |
| `FACULTY_STAGING_DIR` | `/tmp/erau-faculty-staging` | Faculty output staging |
| `GENERAL_STAGING_DIR` | `/tmp/erau-scrape-staging` | General output staging |

## Dependencies

All scripts use Python stdlib only (no pip packages required).
