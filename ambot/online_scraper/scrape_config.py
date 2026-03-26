#!/usr/bin/env python3
"""
Shared configuration for all ERAU scrapers.

All department codes, base URLs, URL patterns, and output defaults live here.
Both scrape_faculty_profiles.py and scraper.py (and any future scrapers) import
from this module instead of defining their own constants.

To add a new scraping target:
  1. Add a new entry to SCRAPE_TARGETS below.
  2. If it needs a custom scraper, create a new script that imports from here.
  3. Register it in SCRAPE_TARGETS so update_knowledge.sh can orchestrate it.
"""

# ── Faculty Directory (faculty.erau.edu) ─────────────────────

FACULTY_BASE_URL = "https://faculty.erau.edu"

# Department codes as they appear on faculty.erau.edu.
# Key = code used in the URL, Value = human-readable name.
DEPARTMENTS = {
    # College of Engineering
    "DCOEECSSE": "Electrical Engineering and Computer Science",
    "DCOEAE":    "Aerospace Engineering",
    "DCOECIV":   "Civil Engineering",
    "DCOEMEC":   "Mechanical Engineering",
    "DCOEFE":    "Engineering Fundamentals",
    # College of Arts & Sciences (add as needed)
    # "DCOASPHY": "Physics",
    # "DCOASMATH": "Mathematics",
    # College of Aviation (add as needed)
    # "DCOAAVSCI": "Aviation Science",
}

DEFAULT_FACULTY_DEPARTMENT = "DCOEECSSE"


# ── General Page Scraper (daytonabeach.erau.edu) ─────────────

COLLEGE_BASE_URL = "https://daytonabeach.erau.edu/college-engineering/"


# ── Scraping Targets ─────────────────────────────────────────
# Each entry describes a scraping source that update_knowledge.sh can run.
# Future scrapers (labs, course catalog, etc.) get added here.
#
# Fields:
#   name        - short identifier (used in logs and --target flag)
#   description - what this target scrapes
#   base_url    - root URL for the scrape
#   scraper     - which Python script handles it
#   output_dir  - default staging directory (under /tmp)
#   knowledge_dest - where cleaned output goes in the knowledge tree
#   enabled     - whether update_knowledge.sh runs it by default

SCRAPE_TARGETS = {
    "faculty": {
        "name": "faculty",
        "description": "Individual faculty profiles from faculty.erau.edu",
        "base_url": FACULTY_BASE_URL,
        "scraper": "scrape_faculty_profiles.py",
        "output_dir": "/tmp/erau-faculty-staging",
        "knowledge_dest": "college/",
        "enabled": True,
    },
    "college": {
        "name": "college",
        "description": "College of Engineering pages from daytonabeach.erau.edu",
        "base_url": COLLEGE_BASE_URL,
        "scraper": "scraper.py",
        "output_dir": "/tmp/erau-scrape-staging",
        "knowledge_dest": "college/",
        "enabled": True,
    },
    # ── Future targets (uncomment and implement when ready) ──
    # "labs": {
    #     "name": "labs",
    #     "description": "Research lab pages",
    #     "base_url": "https://daytonabeach.erau.edu/college-engineering/research/",
    #     "scraper": "scraper.py",
    #     "output_dir": "/tmp/erau-labs-staging",
    #     "knowledge_dest": "college/",
    #     "enabled": False,
    # },
    # "catalog": {
    #     "name": "catalog",
    #     "description": "Course catalog entries",
    #     "base_url": "https://catalog.erau.edu/",
    #     "scraper": "scrape_catalog.py",
    #     "output_dir": "/tmp/erau-catalog-staging",
    #     "knowledge_dest": "programs/",
    #     "enabled": False,
    # },
}


# ── Rate Limiting ────────────────────────────────────────────

DEFAULT_DELAY_SECONDS = 0.5          # Faculty scraper (per-profile)
DEFAULT_CRAWL_DELAY_SECONDS = 1.5    # General page scraper (per-page)
DEFAULT_REQUEST_TIMEOUT = 15         # seconds


# ── Output Defaults ──────────────────────────────────────────

FACULTY_STAGING_DIR = "/tmp/erau-faculty-staging"
GENERAL_STAGING_DIR = "/tmp/erau-scrape-staging"

# Relative to the ambot/bootylicious/rag/ directory
KNOWLEDGE_BASE_DIR = "knowledge"


# ── User Agent ───────────────────────────────────────────────

USER_AGENT = (
    "Mozilla/5.0 (compatible; AMBOT-Scraper/1.1; "
    "educational research project; +https://github.com/msmcs-robotics/WayfindR-driver)"
)
