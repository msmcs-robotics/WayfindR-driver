#!/usr/bin/env python3
"""Post-scrape content cleaner for ERAU scraped markdown files.

Reads scraped .md files from the output/ directory (or specified directory),
removes junk content (site-wide footer, enrollment tables, navigation link
lists, boilerplate), and writes cleaned versions back in-place.

Usage:
    python3 cleaner.py                          # Clean all files in output/
    python3 cleaner.py --input ./output         # Specify input directory
    python3 cleaner.py --dry-run                # Preview changes without writing
    python3 cleaner.py --stats                  # Show per-file cleaning stats
    python3 cleaner.py --verbose                # Show what was removed

This is a post-processing step — run it AFTER scraper.py, BEFORE copying
files to the RAG knowledge folder.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

# Default paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_INPUT_DIR = os.path.join(SCRIPT_DIR, "output")


# =============================================================================
# Footer detection — the ERAU site has a standard footer on every page
# =============================================================================

# These lines signal the start of the site-wide footer. Once we see one of
# these as a standalone line, everything after it is footer/nav boilerplate.
FOOTER_MARKERS = [
    "Future Student Resources",
    "Current Student Resources",
    "University Links",
]

# These standalone lines appear just before the footer block and should also
# be removed (they're CTA buttons or contact headers, not content).
PRE_FOOTER_MARKERS = {
    "Apply Now",
    "Contact Us",
}

# Specific boilerplate lines that appear in the footer but might also appear
# earlier — we only strip these when they're in a footer context.
FOOTER_LINES = {
    "Facebook", "Instagram", "X", "YouTube", "Twitter", "LinkedIn",
    "Sitemap", "Directory", "Web Feedback", "Emergency Info",
    "Nondiscrimination & Title IX", "Military Disclaimer",
    "Privacy Statement & GDPR", "Terms of Use", "Accessibility",
    "|",
}


# =============================================================================
# Enrollment table detection
# =============================================================================

# Patterns that identify enrollment table data lines
ENROLLMENT_TABLE_PATTERNS = [
    r"^(FT|PT)$",                           # Full-Time / Part-Time markers
    r"^Fall \d{4}$",                         # "Fall 2025"
    r"^Spring \d{4}$",                       # "Spring 2025"
    r"^\d{1,4}$",                            # Bare numbers (enrollment counts)
    r"^-$",                                  # Dash (no data)
    r"^Available (?:in )?(?:August|January|May|December) \d{4}$",  # "Available August 2026"
]

# Table header lines that precede enrollment data
ENROLLMENT_HEADERS = {
    "Enrollment Data",
    "Full- or Part-Time",
    "Undergrad Students",
    "Master's Students",
    "Master's Students*",
    "Ph.D. Students",
    "Ph.D. Students*",
    "Ph.D. Students**",
    "Bachelor's Degrees",
    "Bachelor's Degrees*",
    "Bachelor's Degrees**",
    "Master's Degrees",
    "Master's Degrees*",
    "Master's Degrees**",
    "Doctoral Degrees",
    "Doctoral Degrees*",
    "Doctoral Degrees**",
    "Doctoral",
    "Degrees**",
}


# =============================================================================
# Navigation link list detection
# =============================================================================

# Known navigation link texts that appear as standalone lines. These are
# common across ERAU pages and are hyperlinks to other sections of the site.
NAV_LINK_LINES = {
    # Admissions / prospective student links
    "Admissions FAQs", "Campus Map and Directions", "Admissions Events",
    "Area Information", "Tuition and Estimated Costs", "Feature Stories",
    "Honors Program", "Housing", "Professional Programs", "ROTC",
    "Visit the Campus", "Student Blogs", "Clubs and Organizations",
    "Summer Camps", "Embry-Riddle Language Institute (ERLI)",
    # Current student links
    "Academic Calendar", "Bookstore", "Campus Directory", "Career Services",
    "Counseling", "Course Catalog", "Dean of Students",
    "Student Accessibility Services", "EagleCard", "ERNIE Central",
    "Financial Aid", "Health & Wellness", "Library",
    "Office of the Registrar", "Safety and Security",
    "Student Financial Services", "Academic Advancement Center (Tutoring)",
    # Other site-wide links
    "Parents & Family", "Future Students", "Veteran Student Services",
    "Alumni Engagement", "Athletics", "Libraries", "Scholarly Commons",
    "Research at Embry-Riddle", "Faculty Directory",
    "Daytona Beach, FL Campus", "Prescott, AZ Campus",
    "Worldwide Campus", "Online Campus", "Asia Campus",
    "Lift Magazine", "Giving to Embry-Riddle", "Crowdfunding",
    "Awards and Fellowships", "Professional Education",
    "K-12 / Dual Enrollment / Summer Camps", "Speaker Series",
    "International Education", "Working at Embry-Riddle",
    "Board of Trustees", "Accreditation", "Consumer Information",
    "California Disclosures",
    # CTA / action buttons
    "Apply Now", "Read more", "Read More",
    "Support Your Department",
}


# =============================================================================
# Contact/address block detection
# =============================================================================

CONTACT_PATTERNS = [
    r"^Daytona Beach Campus$",
    r"^1 Aerospace Boulevard$",
    r"^Daytona Beach,$",
    r"^Florida$",
    r"^32114$",
    r"^386-226-6100$",
    r"^800-862-2416$",
    r"^Call: 800-862-2416$",
    r"^[A-Za-z]+@erau\.edu$",
    r"^DaytonaBeach@erau\.edu$",
    r"^daytonabeach@erau\.edu$",
    r"^Admin Offices: 1 Aerospace Boulevard$",
    r"^Daytona Beach, FL 32114-3900$",
    r"^©.*Embry-Riddle.*$",
    r"^All Rights Reserved\.$",
    r"^erau\.edu$",
]


# =============================================================================
# Cleaning logic
# =============================================================================


def _is_enrollment_line(line: str) -> bool:
    """Check if a line is part of an enrollment data table."""
    stripped = line.strip()
    if stripped in ENROLLMENT_HEADERS:
        return True
    return any(re.match(pat, stripped) for pat in ENROLLMENT_TABLE_PATTERNS)


def _is_contact_line(line: str) -> bool:
    """Check if a line is part of a contact/address block."""
    stripped = line.strip()
    return any(re.match(pat, stripped) for pat in CONTACT_PATTERNS)


def _find_footer_start(lines: list[str]) -> int | None:
    """Find the line index where the site-wide footer begins.

    The footer is identified by the first occurrence of a FOOTER_MARKER
    as a standalone line. We then scan backwards to include any
    PRE_FOOTER_MARKERS (like "Apply Now", "Contact Us") that appear
    just before the footer.
    """
    footer_idx = None
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped in FOOTER_MARKERS:
            footer_idx = i
            break

    if footer_idx is None:
        return None

    # Scan backwards from footer to capture pre-footer markers and contact info
    start = footer_idx
    for j in range(footer_idx - 1, -1, -1):
        stripped = lines[j].strip()
        if not stripped:  # blank line, keep scanning
            continue
        if (stripped in PRE_FOOTER_MARKERS
                or stripped in FOOTER_LINES
                or stripped in NAV_LINK_LINES
                or _is_contact_line(stripped)):
            start = j
        else:
            break

    return start


def _remove_enrollment_tables(lines: list[str]) -> list[str]:
    """Remove blocks of enrollment table data from the content.

    Enrollment tables are sequences of lines that match enrollment
    patterns (bare numbers, FT/PT, Fall YYYY, etc.). We detect runs
    of 3+ consecutive enrollment-like lines and remove them.
    """
    result = []
    i = 0
    while i < len(lines):
        # Check if this starts an enrollment block
        if _is_enrollment_line(lines[i].strip()):
            # Look ahead to see if this is a sustained run
            run_start = i
            j = i
            consecutive_data = 0
            while j < len(lines):
                stripped = lines[j].strip()
                if stripped == "":
                    j += 1
                    continue
                if _is_enrollment_line(stripped):
                    consecutive_data += 1
                    j += 1
                else:
                    break

            # If we found a substantial enrollment block (5+ data lines),
            # skip it entirely. Short matches (1-2 lines) are kept since
            # they might be legitimate content.
            if consecutive_data >= 5:
                i = j
                continue

        result.append(lines[i])
        i += 1

    return result


def _remove_nav_link_runs(lines: list[str]) -> list[str]:
    """Remove runs of navigation link lines.

    Navigation links appear as standalone lines (one link per line).
    We detect runs of 4+ consecutive nav-link lines (with optional
    blank lines between them) and remove the whole block.
    """
    result = []
    i = 0
    while i < len(lines):
        stripped = lines[i].strip()
        if stripped in NAV_LINK_LINES:
            # Look ahead to see if this is a sustained run
            run_start = i
            j = i
            nav_count = 0
            while j < len(lines):
                s = lines[j].strip()
                if s == "":
                    j += 1
                    continue
                if s in NAV_LINK_LINES:
                    nav_count += 1
                    j += 1
                else:
                    break

            # Remove if 4+ nav links in a row (scattered single matches are kept)
            if nav_count >= 4:
                i = j
                continue

        result.append(lines[i])
        i += 1

    return result


def _remove_footnote_lines(lines: list[str]) -> list[str]:
    """Remove enrollment table footnote lines (start with * or **)."""
    result = []
    for line in lines:
        stripped = line.strip()
        # Footnotes like "* Master's Degrees include..." or "**Degrees Conferred..."
        if re.match(r"^\*{1,2}\s*(The |Degrees |Master)", stripped):
            continue
        # Standalone asterisks
        if stripped in ("*", "**"):
            continue
        result.append(line)
    return result


def clean_content(text: str) -> tuple[str, dict]:
    """Clean scraped content, returning (cleaned_text, stats).

    Stats dict contains counts of what was removed.
    """
    stats = {
        "original_lines": 0,
        "footer_lines_removed": 0,
        "table_lines_removed": 0,
        "nav_lines_removed": 0,
        "footnote_lines_removed": 0,
        "final_lines": 0,
    }

    lines = text.split("\n")
    stats["original_lines"] = len(lines)

    # Step 1: Remove the site-wide footer (everything from footer marker to end)
    footer_start = _find_footer_start(lines)
    if footer_start is not None:
        stats["footer_lines_removed"] = len(lines) - footer_start
        lines = lines[:footer_start]

    # Step 2: Remove enrollment table data blocks
    before_count = len(lines)
    lines = _remove_enrollment_tables(lines)
    stats["table_lines_removed"] = before_count - len(lines)

    # Step 3: Remove navigation link runs within the body
    before_count = len(lines)
    lines = _remove_nav_link_runs(lines)
    stats["nav_lines_removed"] = before_count - len(lines)

    # Step 4: Remove footnote lines
    before_count = len(lines)
    lines = _remove_footnote_lines(lines)
    stats["footnote_lines_removed"] = before_count - len(lines)

    # Step 5: Collapse 3+ consecutive blank lines into 2
    text = "\n".join(lines)
    text = re.sub(r"\n{3,}", "\n\n", text)
    text = text.strip() + "\n"

    stats["final_lines"] = len(text.split("\n"))

    return text, stats


def clean_file(filepath: str, dry_run: bool = False) -> dict:
    """Clean a single markdown file. Returns stats dict."""
    with open(filepath, "r", encoding="utf-8") as f:
        original = f.read()

    # Split into header (metadata) and body at the --- separator
    parts = original.split("---\n", 1)
    if len(parts) == 2:
        header = parts[0] + "---\n"
        body = parts[1]
    else:
        header = ""
        body = original

    cleaned_body, stats = clean_content(body)
    cleaned = header + cleaned_body

    stats["filename"] = os.path.basename(filepath)
    stats["original_chars"] = len(original)
    stats["final_chars"] = len(cleaned)
    stats["chars_removed"] = len(original) - len(cleaned)
    stats["changed"] = original != cleaned

    if not dry_run and stats["changed"]:
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(cleaned)

    return stats


# =============================================================================
# Main
# =============================================================================


def main():
    parser = argparse.ArgumentParser(
        description="Post-scrape content cleaner for ERAU markdown files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 cleaner.py                    # Clean all files in output/
  python3 cleaner.py --dry-run          # Preview without writing
  python3 cleaner.py --stats            # Show per-file stats
  python3 cleaner.py --verbose          # Detailed removal info
  python3 cleaner.py --input ./output   # Custom input directory
        """,
    )
    parser.add_argument(
        "--input",
        default=DEFAULT_INPUT_DIR,
        help=f"Input directory with .md files (default: {DEFAULT_INPUT_DIR})",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview changes without writing files",
    )
    parser.add_argument(
        "--stats",
        action="store_true",
        help="Show per-file cleaning statistics",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Show detailed removal information",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.input):
        print(f"Error: directory not found: {args.input}", file=sys.stderr)
        return 1

    md_files = sorted(Path(args.input).glob("*.md"))
    if not md_files:
        print(f"No .md files found in {args.input}")
        return 0

    total_original = 0
    total_final = 0
    files_changed = 0
    files_total = len(md_files)

    if args.dry_run:
        print("=== DRY RUN (no files will be modified) ===\n")

    for filepath in md_files:
        file_stats = clean_file(str(filepath), dry_run=args.dry_run)

        total_original += file_stats["original_chars"]
        total_final += file_stats["final_chars"]
        if file_stats["changed"]:
            files_changed += 1

        if args.stats or args.verbose:
            name = file_stats["filename"]
            removed = file_stats["chars_removed"]
            pct = (removed / file_stats["original_chars"] * 100) if file_stats["original_chars"] > 0 else 0

            if file_stats["changed"]:
                print(f"  {name}: {file_stats['original_chars']} -> {file_stats['final_chars']} chars "
                      f"(-{removed}, {pct:.0f}% removed)")
                if args.verbose:
                    print(f"    footer: {file_stats['footer_lines_removed']} lines, "
                          f"tables: {file_stats['table_lines_removed']} lines, "
                          f"nav: {file_stats['nav_lines_removed']} lines, "
                          f"footnotes: {file_stats['footnote_lines_removed']} lines")
            else:
                print(f"  {name}: no changes")

    # Summary
    total_removed = total_original - total_final
    pct = (total_removed / total_original * 100) if total_original > 0 else 0

    print()
    print("=" * 50)
    print(f"  Files processed: {files_total}")
    print(f"  Files changed:   {files_changed}")
    print(f"  Original size:   {total_original:,} chars")
    print(f"  Cleaned size:    {total_final:,} chars")
    print(f"  Removed:         {total_removed:,} chars ({pct:.1f}%)")
    print("=" * 50)

    if args.dry_run:
        print(f"\n  (dry run — no files were modified)")
    elif files_changed > 0:
        print(f"\n  {files_changed} files cleaned in {args.input}/")

    return 0


if __name__ == "__main__":
    sys.exit(main())
