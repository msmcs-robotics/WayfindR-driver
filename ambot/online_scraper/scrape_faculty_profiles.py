#!/usr/bin/env python3
"""
ERAU Faculty Profile Scraper — Scrapes individual faculty profiles from faculty.erau.edu.

Outputs structured markdown files to a temp/staging directory for review
before moving to the RAG knowledge folder.

Data scraped per faculty member:
  - Name, title, department
  - Degrees with institutions
  - Areas of expertise
  - Courses taught
  - Email
  - Office info (if available)

Usage:
    # Scrape EECS department faculty profiles to staging folder
    python3 scrape_faculty_profiles.py

    # Scrape specific department
    python3 scrape_faculty_profiles.py --department DCOEECSSE

    # Scrape all departments
    python3 scrape_faculty_profiles.py --all-departments

    # Output to custom directory
    python3 scrape_faculty_profiles.py --output /tmp/faculty-staging

    # Resume an interrupted scrape (skip existing files)
    python3 scrape_faculty_profiles.py --resume

    # List departments only
    python3 scrape_faculty_profiles.py --list-departments

    # Dry run (just show URLs, don't scrape)
    python3 scrape_faculty_profiles.py --dry-run

Adding New URL Patterns:
    To scrape a new department, add its code to DEPARTMENTS in scrape_config.py.
    To add an entirely new scraping target (e.g., labs, course catalog), add an
    entry to SCRAPE_TARGETS in scrape_config.py, then create or reuse a scraper
    script. See scrape_config.py for the full schema and examples.
"""

import argparse
import logging
import os
import re
import time
import urllib.request
import urllib.error
from html.parser import HTMLParser
from pathlib import Path
from datetime import datetime

# Import shared config
try:
    from scrape_config import (
        FACULTY_BASE_URL as FACULTY_BASE,
        DEPARTMENTS,
        DEFAULT_FACULTY_DEPARTMENT as DEFAULT_DEPARTMENT,
        DEFAULT_DELAY_SECONDS as DELAY_BETWEEN_REQUESTS,
        DEFAULT_REQUEST_TIMEOUT,
        FACULTY_STAGING_DIR as DEFAULT_OUTPUT,
        USER_AGENT,
    )
except ImportError:
    # Fallback if run standalone without scrape_config on PYTHONPATH
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from scrape_config import (
        FACULTY_BASE_URL as FACULTY_BASE,
        DEPARTMENTS,
        DEFAULT_FACULTY_DEPARTMENT as DEFAULT_DEPARTMENT,
        DEFAULT_DELAY_SECONDS as DELAY_BETWEEN_REQUESTS,
        DEFAULT_REQUEST_TIMEOUT,
        FACULTY_STAGING_DIR as DEFAULT_OUTPUT,
        USER_AGENT,
    )


# ── Logging ───────────────────────────────────────────────────

log = logging.getLogger("faculty_scraper")


def setup_logging(output_dir):
    """Configure logging to both console and a log file in the output directory."""
    log.setLevel(logging.DEBUG)

    # Console handler (INFO level)
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    console.setFormatter(logging.Formatter("%(message)s"))
    log.addHandler(console)

    # File handler (DEBUG level, captures everything)
    log_dir = Path(output_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "scrape.log"
    fh = logging.FileHandler(log_file, mode="a", encoding="utf-8")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
    ))
    log.addHandler(fh)

    log.debug("=" * 60)
    log.debug("Scrape session started: %s", datetime.now().isoformat())
    log.debug("=" * 60)

    return log_file


# ── HTML Parsers ───────────────────────────────────────────

class FacultyListParser(HTMLParser):
    """Parse the department faculty listing page to extract profile URLs."""

    def __init__(self):
        super().__init__()
        self.profile_urls = []
        self._in_link = False
        self._current_href = ""

    def handle_starttag(self, tag, attrs):
        if tag == "a":
            attrs_dict = dict(attrs)
            href = attrs_dict.get("href", "")
            # Faculty profile links are like /FirstName.LastName
            if href and re.match(r'^/[A-Z][a-zA-Z]+\.[A-Z]', href):
                full_url = FACULTY_BASE + href
                if full_url not in self.profile_urls:
                    self.profile_urls.append(full_url)


class ProfileParser(HTMLParser):
    """Parse an individual faculty profile page to extract structured data."""

    def __init__(self):
        super().__init__()
        self.data = {
            "name": "",
            "title": "",
            "department": "",
            "email": "",
            "phone": "",
            "office": "",
            "degrees": [],
            "expertise": [],
            "courses": [],
            "bio": "",
        }
        self._current_section = ""
        self._text_buffer = []
        self._in_heading = False
        self._tag_stack = []
        self._capture = True

    def handle_starttag(self, tag, attrs):
        self._tag_stack.append(tag)
        attrs_dict = dict(attrs)

        if tag in ("h1", "h2", "h3", "h4"):
            self._in_heading = True
            self._text_buffer = []

        # Email links
        if tag == "a":
            href = attrs_dict.get("href", "")
            if href.startswith("mailto:"):
                self.data["email"] = href[7:]

    def handle_endtag(self, tag):
        if self._tag_stack and self._tag_stack[-1] == tag:
            self._tag_stack.pop()

        if tag in ("h1", "h2", "h3", "h4"):
            self._in_heading = False
            heading_text = " ".join(self._text_buffer).strip().lower()
            if "education" in heading_text or "degree" in heading_text:
                self._current_section = "education"
            elif "expertise" in heading_text or "research" in heading_text or "interest" in heading_text:
                self._current_section = "expertise"
            elif "course" in heading_text or "teaching" in heading_text:
                self._current_section = "courses"
            elif "bio" in heading_text or "about" in heading_text:
                self._current_section = "bio"
            elif "contact" in heading_text or "office" in heading_text:
                self._current_section = "contact"
            self._text_buffer = []

    def handle_data(self, data):
        text = data.strip()
        if not text:
            return

        if self._in_heading:
            self._text_buffer.append(text)
            return

        # First h1 is usually the name
        if not self.data["name"] and "h1" in self._tag_stack:
            self.data["name"] = text

        # Capture by section
        if self._current_section == "education":
            # Look for degree patterns: "Ph.D., Something University"
            if re.search(r'(Ph\.?D|M\.?S|B\.?S|M\.?A|B\.?A)', text):
                self.data["degrees"].append(text)
        elif self._current_section == "expertise":
            if len(text) > 3 and not text.startswith("\u2022"):
                self.data["expertise"].append(text.strip("\u2022- "))
        elif self._current_section == "courses":
            if len(text) > 3:
                self.data["courses"].append(text.strip("\u2022- "))
        elif self._current_section == "contact":
            if "room" in text.lower() or "building" in text.lower():
                self.data["office"] = text
            elif re.search(r'\d{3}[\-\.]\d{3}[\-\.]\d{4}', text):
                self.data["phone"] = text


# ── Fetch & Parse ──────────────────────────────────────────

def fetch_page(url, retries=2):
    """Fetch a URL and return HTML content."""
    for attempt in range(retries + 1):
        try:
            req = urllib.request.Request(url, headers={
                "User-Agent": USER_AGENT,
            })
            with urllib.request.urlopen(req, timeout=DEFAULT_REQUEST_TIMEOUT) as resp:
                return resp.read().decode("utf-8", errors="replace")
        except (urllib.error.URLError, urllib.error.HTTPError) as e:
            log.debug("Attempt %d/%d failed for %s: %s", attempt + 1, retries + 1, url, e)
            if attempt < retries:
                time.sleep(1)
            else:
                log.error("FAILED: %s -- %s", url, e)
                return None


def get_department_faculty_urls(dept_code):
    """Get all faculty profile URLs for a department."""
    all_urls = []
    page = 1
    while True:
        url = f"{FACULTY_BASE}/departments/{dept_code}"
        if page > 1:
            url += f"?p={page}"

        log.info("  Fetching listing page %d: %s", page, url)
        html = fetch_page(url)
        if not html:
            break

        parser = FacultyListParser()
        parser.feed(html)

        if not parser.profile_urls:
            break

        new_urls = [u for u in parser.profile_urls if u not in all_urls]
        if not new_urls:
            break

        all_urls.extend(new_urls)
        log.info("    Found %d profiles (total: %d)", len(new_urls), len(all_urls))

        # Check if there's a next page
        if f"?p={page + 1}" not in html:
            break
        page += 1
        time.sleep(DELAY_BETWEEN_REQUESTS)

    return all_urls


def scrape_profile(url):
    """Scrape a single faculty profile page."""
    html = fetch_page(url)
    if not html:
        return None

    parser = ProfileParser()
    parser.feed(html)

    # Extract name from URL as fallback
    if not parser.data["name"]:
        slug = url.rstrip("/").split("/")[-1]
        parts = slug.split(".")
        if len(parts) >= 2:
            parser.data["name"] = f"{parts[0]} {'.'.join(parts[1:])}"

    parser.data["profile_url"] = url
    return parser.data


# ── Output ─────────────────────────────────────────────────

def faculty_to_markdown(data):
    """Convert structured faculty data to RAG-friendly markdown."""
    lines = []

    name = data.get("name", "Unknown")
    lines.append(f"# {name}")
    lines.append("")

    # Core info
    if data.get("title"):
        lines.append(f"**Title:** {data['title']}")
    if data.get("department"):
        lines.append(f"**Department:** {data['department']}")
    if data.get("email"):
        lines.append(f"**Email:** {data['email']}")
    if data.get("phone"):
        lines.append(f"**Phone:** {data['phone']}")
    if data.get("office"):
        lines.append(f"**Office:** {data['office']}")
    if data.get("profile_url"):
        lines.append(f"**Profile:** {data['profile_url']}")
    lines.append("")

    # Education
    if data.get("degrees"):
        lines.append("## Education")
        lines.append("")
        for deg in data["degrees"]:
            lines.append(f"- {deg}")
        lines.append("")

    # Expertise / Research
    if data.get("expertise"):
        lines.append("## Areas of Expertise")
        lines.append("")
        for area in data["expertise"]:
            lines.append(f"- {area}")
        lines.append("")

    # Courses
    if data.get("courses"):
        lines.append("## Courses")
        lines.append("")
        for course in data["courses"]:
            lines.append(f"- {course}")
        lines.append("")

    # Bio
    if data.get("bio"):
        lines.append("## Biography")
        lines.append("")
        lines.append(data["bio"])
        lines.append("")

    # Metadata
    lines.append("---")
    lines.append(f"*Scraped from faculty.erau.edu on {datetime.now().strftime('%Y-%m-%d %H:%M')}*")
    lines.append("")

    return "\n".join(lines)


def name_to_filename(name):
    """Convert a faculty name to a safe filename."""
    safe = re.sub(r'[^a-zA-Z0-9\s]', '', name).strip()
    safe = re.sub(r'\s+', '-', safe).lower()
    return safe[:60] + ".md"


# ── Main ───────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="ERAU Faculty Profile Scraper",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Adding New Departments:
  Edit scrape_config.py and add the department code to the DEPARTMENTS dict.
  Department codes are visible in the faculty.erau.edu URL when browsing by
  department (e.g., /departments/DCOEECSSE).

Adding New Scraping Targets:
  For entirely new data sources (labs, course catalog, etc.), add an entry
  to SCRAPE_TARGETS in scrape_config.py. Each target specifies:
    - base_url:        root URL to scrape
    - scraper:         which Python script handles it
    - output_dir:      staging directory (under /tmp)
    - knowledge_dest:  subfolder in the RAG knowledge tree
  Then create or reuse a scraper script. The update_knowledge.sh master
  script will automatically pick up new enabled targets.

Examples:
  python3 scrape_faculty_profiles.py                         # EECS only
  python3 scrape_faculty_profiles.py --all-departments       # All departments
  python3 scrape_faculty_profiles.py --resume                # Skip existing files
  python3 scrape_faculty_profiles.py --delay 1.0             # Slower requests
  python3 scrape_faculty_profiles.py --department DCOEAE     # Aerospace only
  python3 scrape_faculty_profiles.py --list-departments      # Show codes
  python3 scrape_faculty_profiles.py --dry-run               # URLs only
        """,
    )
    parser.add_argument("--department", default=DEFAULT_DEPARTMENT,
                        help=f"Department code (default: {DEFAULT_DEPARTMENT})")
    parser.add_argument("--all-departments", action="store_true",
                        help="Scrape all engineering departments")
    parser.add_argument("--output", default=DEFAULT_OUTPUT,
                        help=f"Output directory (default: {DEFAULT_OUTPUT})")
    parser.add_argument("--list-departments", action="store_true",
                        help="List available department codes")
    parser.add_argument("--dry-run", action="store_true",
                        help="Show URLs without scraping")
    parser.add_argument("--resume", action="store_true",
                        help="Skip profiles that already have output files")
    parser.add_argument("--delay", type=float, default=DELAY_BETWEEN_REQUESTS,
                        help=f"Delay between requests in seconds (default: {DELAY_BETWEEN_REQUESTS})")
    args = parser.parse_args()

    if args.list_departments:
        print("Available departments:")
        for code, name in DEPARTMENTS.items():
            print(f"  {code}: {name}")
        return

    output_dir = Path(args.output)
    log_file = setup_logging(args.output)

    departments = DEPARTMENTS if args.all_departments else {args.department: DEPARTMENTS.get(args.department, args.department)}

    total_scraped = 0
    total_skipped = 0
    total_failed = 0

    for dept_code, dept_name in departments.items():
        log.info("")
        log.info("=" * 60)
        log.info("  Department: %s (%s)", dept_name, dept_code)
        log.info("=" * 60)

        # Create department subfolder
        dept_dir = output_dir / dept_code.lower()
        dept_dir.mkdir(parents=True, exist_ok=True)

        # Get all profile URLs
        urls = get_department_faculty_urls(dept_code)
        log.info("  Found %d faculty profiles", len(urls))

        if args.dry_run:
            for url in urls:
                log.info("    %s", url)
            continue

        # Scrape each profile
        for i, url in enumerate(urls):
            # Resume support: check if output file already exists
            if args.resume:
                # Guess the filename from the URL slug
                slug = url.rstrip("/").split("/")[-1]
                parts = slug.split(".")
                if len(parts) >= 2:
                    guessed_name = f"{parts[0]} {'.'.join(parts[1:])}"
                    guessed_file = dept_dir / name_to_filename(guessed_name)
                    if guessed_file.exists():
                        log.info("  [%d/%d] SKIP (exists): %s", i + 1, len(urls), guessed_file.name)
                        total_skipped += 1
                        continue

            log.info("  [%d/%d] %s ... ", i + 1, len(urls), url)

            data = scrape_profile(url)
            if data and data.get("name"):
                data["department"] = dept_name

                md = faculty_to_markdown(data)
                filename = name_to_filename(data["name"])
                filepath = dept_dir / filename

                # Double-check resume (in case name differs from URL slug)
                if args.resume and filepath.exists():
                    log.info("    SKIP (exists): %s", filename)
                    total_skipped += 1
                else:
                    filepath.write_text(md)
                    log.info("    OK (%s)", data["name"])
                    total_scraped += 1
            else:
                log.warning("    SKIP (no data)")
                total_failed += 1

            time.sleep(args.delay)

    log.info("")
    log.info("=" * 60)
    log.info("  Done! Scraped: %d  Skipped: %d  Failed: %d", total_scraped, total_skipped, total_failed)
    log.info("  Output: %s", output_dir)
    log.info("  Log:    %s", log_file)
    log.info("=" * 60)
    log.info("")
    log.info("  Next steps:")
    log.info("    1. Clean: python3 clean_faculty_profiles.py --input %s", output_dir)
    log.info("    2. Review, then copy to knowledge/")
    log.info("    3. Or use: ./update_knowledge.sh --apply")


if __name__ == "__main__":
    main()
