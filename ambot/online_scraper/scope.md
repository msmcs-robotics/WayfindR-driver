# Online Scraper — Scope & Configuration

## Purpose

Scrapes public-facing web pages from ERAU (Embry-Riddle Aeronautical University) College of Engineering website for ingestion into the AMBOT RAG knowledge base.

## Base URL Scope

```
https://daytonabeach.erau.edu/college-engineering/
```

**Strict scope enforcement:**
- All crawled URLs must match the base hostname (`daytonabeach.erau.edu`)
- All crawled URLs must have a path starting with `/college-engineering/`
- URLs outside this scope are never fetched (e.g., `/admissions/`, `/college-arts-sciences/`, other subdomains)

## What Gets Scraped

### Departments (depth 1)
- Aerospace Engineering (`/college-engineering/aerospace`)
- Civil Engineering (`/college-engineering/civil`)
- Engineering Fundamentals (`/college-engineering/fundamentals`)
- Mechanical Engineering (`/college-engineering/mechanical`)
- Electrical Engineering & Computer Science (`/college-engineering/electrical-engineering-computer-science`)

### Department Sub-Pages (depth 2+)
Each department typically has: accreditation, faculty, labs, research, student engagement, industry advisory, internships/co-ops, news

### College-Wide Pages (depth 1)
- Accreditation, Administration, Faculty, Industry Relations
- Labs & Facilities, Research & Innovation, Student Engagement
- Combined BS/MS Programs, Engineers Week

## What Gets Skipped

- **Login/SSO pages**: URLs containing `/login`, `/sso`, `/cas/`, `/auth`
- **Binary files**: PDFs, images, archives, Office documents
- **External links**: Any URL not within the base URL scope
- **Pages with < 100 chars of content**: Too little text to be useful
- **Timeout/error pages**: Logged but not saved

## Output

Scraped pages are saved as markdown files in `./output/`:
- One `.md` file per page
- Filename derived from URL path (e.g., `aerospace__labs.md`)
- Each file includes source URL and scrape timestamp
- Content is the main text of the page (navigation/footer stripped when possible)

## Usage

```bash
# Full crawl (default: depth 5, 1.5s delay)
python3 scraper.py

# List URLs only (no download)
python3 scraper.py --list-only

# Shallow crawl (faster)
python3 scraper.py --depth 2

# Custom output directory
python3 scraper.py --output ./my_output

# Slower crawl (more polite)
python3 scraper.py --delay 2.0
```

## Post-Scrape Content Cleaning

The `cleaner.py` script removes junk content from scraped files before RAG ingestion:

### What Gets Removed

- **Site-wide footer**: Contact info, social media links, legal boilerplate (~200 lines per page, identical across all pages)
- **Navigation link runs**: "Future Student Resources", "Current Student Resources", campus links, etc. (standalone hyperlink text that provides no content value)
- **Enrollment data tables**: Raw numbers (FT/PT counts, enrollment figures), semester labels ("Fall 2025"), table headers — these don't parse as useful text
- **Table footnotes**: Asterisk-prefixed explanation lines for enrollment tables
- **Contact blocks**: Address, phone, email, copyright notices

### What Gets Kept

- All educational content (program descriptions, objectives, outcomes)
- Faculty and research information
- Lab descriptions and facility details
- Accreditation statements and program details
- News articles and event descriptions

### Usage

```bash
# Clean all files in output/ (in-place)
python3 cleaner.py

# Preview changes without modifying files
python3 cleaner.py --dry-run

# Show per-file stats
python3 cleaner.py --stats

# Detailed breakdown of what was removed
python3 cleaner.py --verbose

# Custom input directory
python3 cleaner.py --input ./my_output
```

### Typical Results

- 49 files, ~27% content removed (100K chars of junk out of 371K total)
- Footer removal accounts for most savings (consistent ~1.8K chars/file)
- Accreditation pages with enrollment tables see 40-55% reduction

## Pipeline to RAG

```bash
# 1. Scrape
cd ambot/online_scraper
python3 scraper.py

# 2. Clean scraped content (removes footer, tables, nav links)
python3 cleaner.py --stats

# 3. Review output
ls -la output/

# 4. Copy to RAG knowledge folder
cp output/*.md ../bootylicious/rag/knowledge/

# 5. Sync to Jetson and ingest
rsync -avz ../bootylicious/rag/knowledge/ jetson:~/ambot/bootylicious/rag/knowledge/
ssh jetson "cd ~/ambot && ./bootylicious/ingest.sh"
```

## Configuration

### Scraper (`scraper.py`)

| Setting | Default | Description |
|---------|---------|-------------|
| `--base-url` | `https://daytonabeach.erau.edu/college-engineering/` | Root URL to crawl |
| `--depth` | 5 | Maximum link-following depth |
| `--delay` | 1.5s | Delay between requests (be polite) |
| `--output` | `./output/` | Directory for saved markdown files |
| `--list-only` | false | Only discover and list URLs |

### Cleaner (`cleaner.py`)

| Setting | Default | Description |
|---------|---------|-------------|
| `--input` | `./output/` | Directory with .md files to clean |
| `--dry-run` | false | Preview changes without writing |
| `--stats` | false | Show per-file cleaning statistics |
| `--verbose` | false | Show detailed removal breakdown |

## Notes

- Both scripts use only Python stdlib (no pip dependencies)
- Polite crawler: 1.5s default delay, identifies itself as educational project
- BFS crawl order (breadth-first)
- Deduplication via normalized URLs
- Does NOT handle JavaScript-rendered content (works because ERAU serves server-side HTML)
- Cleaner is idempotent — running it multiple times on already-cleaned files has no effect
- Output files are tracked in git (not gitignored)
