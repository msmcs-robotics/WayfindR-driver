#!/usr/bin/env python3
"""
Clean scraped faculty profiles — removes footer/navigation boilerplate.

Run after scrape_faculty_profiles.py to strip site-wide junk from profiles.

Usage:
    python3 clean_faculty_profiles.py                              # Clean default staging dir
    python3 clean_faculty_profiles.py --input /tmp/erau-faculty-staging
    python3 clean_faculty_profiles.py --dry-run                     # Preview without modifying
"""

import argparse
import re
from pathlib import Path

DEFAULT_INPUT = "/tmp/erau-faculty-staging"

# Lines that indicate the start of footer/nav boilerplate
BOILERPLATE_MARKERS = [
    "Personal Information",
    "University Links",
    "Libraries",
    "Scholarly Commons",
    "Research at Embry-Riddle",
    "Faculty Directory",
    "Course Catalog",
    "Daytona Beach, FL Campus",
    "Prescott, AZ Campus",
    "Worldwide Campus",
    "Online Campus",
    "Asia Campus",
    "Career Services",
    "Alumni Engagement",
    "Lift Magazine",
    "Giving to Embry-Riddle",
    "Crowdfunding",
    "Awards and Fellowships",
    "Professional Education",
    "K-12 / Dual Enrollment",
    "Athletics",
    "Speaker Series",
    "International Education",
    "Working at Embry-Riddle",
    "Board of Trustees",
    "Accreditation",
    "Consumer Information",
    "California Disclosures",
    "erau.edu",
    "Admin Offices:",
    "© Embry-Riddle",
    "All Rights Reserved",
    "Emergency Info",
    "Nondiscrimination",
    "Military Disclaimer",
    "Privacy Statement",
    "Terms of Use",
    "Accessibility",
]

# Navigation section headers that should be removed
NAV_SECTIONS = [
    "Biography",
    "Education",
    "Currently Teaching",
    "Courses Taught",
    "Research",
    "Publications",
]


def clean_profile(content):
    """Remove boilerplate from a faculty profile markdown file."""
    lines = content.split("\n")
    cleaned = []
    removed = 0
    in_boilerplate = False

    for line in lines:
        stripped = line.strip().lstrip("- ").strip()

        # Check if this line starts boilerplate
        if any(stripped == marker or stripped.startswith(marker) for marker in BOILERPLATE_MARKERS):
            in_boilerplate = True
            removed += 1
            continue

        # Check nav section headers (standalone lines like "- Biography")
        if stripped in NAV_SECTIONS and line.startswith("- "):
            removed += 1
            continue

        # If we hit the metadata footer (---), stop removing and include it
        if stripped == "---" and in_boilerplate:
            in_boilerplate = False
            cleaned.append(line)
            continue

        if in_boilerplate:
            # Check if we've hit real content again (e.g., ## Courses after nav)
            if stripped.startswith("## ") or stripped.startswith("# "):
                in_boilerplate = False
                cleaned.append(line)
            else:
                removed += 1
        else:
            cleaned.append(line)

    # Clean up excessive blank lines
    result = "\n".join(cleaned)
    result = re.sub(r'\n{3,}', '\n\n', result)

    return result, removed


def main():
    parser = argparse.ArgumentParser(description="Clean faculty profile boilerplate")
    parser.add_argument("--input", default=DEFAULT_INPUT, help="Input directory")
    parser.add_argument("--dry-run", action="store_true", help="Preview without writing")
    args = parser.parse_args()

    input_dir = Path(args.input)
    if not input_dir.exists():
        print(f"Directory not found: {input_dir}")
        return

    total_files = 0
    total_removed = 0

    for md_file in sorted(input_dir.rglob("*.md")):
        content = md_file.read_text()
        cleaned, removed = clean_profile(content)

        if removed > 0:
            total_files += 1
            total_removed += removed
            pct = (1 - len(cleaned) / len(content)) * 100 if content else 0
            print(f"  {md_file.name}: removed {removed} lines ({pct:.0f}% reduction)")

            if not args.dry_run:
                md_file.write_text(cleaned)

    print(f"\nCleaned {total_files} files, removed {total_removed} boilerplate lines")
    if args.dry_run:
        print("(dry-run mode — no files modified)")


if __name__ == "__main__":
    main()
