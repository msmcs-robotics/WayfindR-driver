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

## Pipeline to RAG

```bash
# 1. Scrape
cd ambot/online_scraper
python3 scraper.py

# 2. Review output
ls -la output/

# 3. Copy to RAG knowledge folder
cp output/*.md ../bootylicious/rag/knowledge/

# 4. Sync to Jetson and ingest
rsync -avz ../bootylicious/rag/knowledge/ jetson:~/ambot/bootylicious/rag/knowledge/
ssh jetson "cd ~/ambot && ./bootylicious/ingest.sh"
```

## Configuration

| Setting | Default | Description |
|---------|---------|-------------|
| `--base-url` | `https://daytonabeach.erau.edu/college-engineering/` | Root URL to crawl |
| `--depth` | 5 | Maximum link-following depth |
| `--delay` | 1.5s | Delay between requests (be polite) |
| `--output` | `./output/` | Directory for saved markdown files |
| `--list-only` | false | Only discover and list URLs |

## Notes

- Uses only Python stdlib (no pip dependencies)
- Polite crawler: 1.5s default delay, identifies itself as educational project
- BFS crawl order (breadth-first)
- Deduplication via normalized URLs
- Does NOT handle JavaScript-rendered content (works because ERAU serves server-side HTML)
