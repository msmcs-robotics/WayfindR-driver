#!/usr/bin/env python3
"""ERAU College of Engineering Web Scraper.

Crawls public-facing pages under a base URL, extracts text content,
and saves each page as a markdown file. Stays within the specified
URL scope and skips login/SSO pages.

Usage:
    python3 scraper.py                          # Crawl with defaults
    python3 scraper.py --depth 3                # Limit crawl depth
    python3 scraper.py --list-only              # Just list discovered URLs
    python3 scraper.py --output ./my_output     # Custom output directory
    python3 scraper.py --delay 2.0              # Seconds between requests
    python3 scraper.py --base-url https://daytonabeach.erau.edu/college-engineering/

Outputs one .md file per page in the output/ directory.
"""

from __future__ import annotations

import argparse
import hashlib
import logging
import os
import re
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import urljoin, urlparse
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

# Default configuration
DEFAULT_BASE_URL = "https://daytonabeach.erau.edu/college-engineering/"
DEFAULT_OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
DEFAULT_DELAY = 1.5  # seconds between requests (be polite)
DEFAULT_TIMEOUT = 15  # seconds per request
DEFAULT_MAX_DEPTH = 5
USER_AGENT = (
    "Mozilla/5.0 (compatible; AMBOT-Scraper/1.0; "
    "educational research project; +https://github.com/msmcs-robotics/WayfindR-driver)"
)

# Skip patterns — URLs matching these are never fetched
SKIP_PATTERNS = [
    r"\.pdf$",         # PDFs (can't parse HTML)
    r"\.jpg$", r"\.jpeg$", r"\.png$", r"\.gif$", r"\.svg$",  # images
    r"\.mp4$", r"\.mp3$", r"\.wav$",  # media
    r"\.zip$", r"\.tar$", r"\.gz$",   # archives
    r"\.docx?$", r"\.xlsx?$", r"\.pptx?$",  # office
    r"/login", r"/sso", r"/cas/", r"/auth",  # authentication
    r"javascript:", r"mailto:", r"tel:",       # non-http
    r"#",  # fragment-only links
]


# =============================================================================
# HTML Parser — extracts links and text content
# =============================================================================


class PageParser(HTMLParser):
    """Extracts hyperlinks and visible text from HTML."""

    # Tags whose content is not visible text (paired tags only — have close tags)
    INVISIBLE_TAGS = {"script", "style", "noscript", "svg"}

    # Tags that represent block-level elements (insert newlines)
    BLOCK_TAGS = {
        "p", "div", "section", "article", "main", "header", "footer",
        "h1", "h2", "h3", "h4", "h5", "h6", "li", "tr", "blockquote",
        "figcaption", "nav", "aside",
    }

    def __init__(self):
        super().__init__()
        self.links: list[str] = []
        self.text_parts: list[str] = []
        self.title: str = ""
        self._tag_stack: list[str] = []
        self._in_invisible = 0
        self._in_title = False
        self._in_main = False
        self._main_text_parts: list[str] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        tag = tag.lower()
        self._tag_stack.append(tag)

        if tag in self.INVISIBLE_TAGS:
            self._in_invisible += 1

        if tag == "title":
            self._in_title = True

        # Track main content area
        attr_dict = dict(attrs)
        tag_id = attr_dict.get("id", "")
        tag_class = attr_dict.get("class", "")
        tag_role = attr_dict.get("role", "")
        if (tag == "main"
            or tag_role == "main"
            or tag_id in ("main-content", "content", "main")
            or "main-content" in tag_class
            or "page-content" in tag_class):
            self._in_main = True

        # Block-level tags insert a newline
        if tag in self.BLOCK_TAGS:
            self.text_parts.append("\n")
            if self._in_main:
                self._main_text_parts.append("\n")

        # Extract href from <a> tags
        if tag == "a":
            href = attr_dict.get("href")
            if href:
                self.links.append(href)

        # Insert newline for <br>
        if tag == "br":
            self.text_parts.append("\n")
            if self._in_main:
                self._main_text_parts.append("\n")

    def handle_endtag(self, tag: str) -> None:
        tag = tag.lower()

        if tag in self.INVISIBLE_TAGS:
            self._in_invisible = max(0, self._in_invisible - 1)

        if tag == "title":
            self._in_title = False

        if tag == "main":
            self._in_main = False

        if tag in self.BLOCK_TAGS:
            self.text_parts.append("\n")
            if self._in_main:
                self._main_text_parts.append("\n")

        if self._tag_stack and self._tag_stack[-1] == tag:
            self._tag_stack.pop()

    def handle_data(self, data: str) -> None:
        if self._in_title:
            self.title += data.strip()

        if self._in_invisible:
            return

        stripped = data.strip()
        if stripped:
            self.text_parts.append(stripped)
            if self._in_main:
                self._main_text_parts.append(stripped)

    def get_text(self) -> str:
        """Return extracted text, preferring main content area if found."""
        # Use main content if it has substance, otherwise fall back to full page
        main_text = _clean_text("\n".join(self._main_text_parts))
        if len(main_text) > 200:
            return main_text
        return _clean_text("\n".join(self.text_parts))


def _clean_text(raw: str) -> str:
    """Clean up extracted text: collapse whitespace, remove empty lines."""
    # Collapse multiple spaces within lines
    lines = raw.split("\n")
    cleaned = []
    for line in lines:
        line = re.sub(r"[ \t]+", " ", line).strip()
        cleaned.append(line)
    # Collapse 3+ consecutive blank lines into 2
    text = "\n".join(cleaned)
    text = re.sub(r"\n{3,}", "\n\n", text)
    return text.strip()


# =============================================================================
# URL helpers
# =============================================================================


def is_within_scope(url: str, base_url: str) -> bool:
    """Check if a URL is within the base URL scope."""
    parsed_base = urlparse(base_url)
    parsed_url = urlparse(url)

    # Must be same scheme and host
    if parsed_url.scheme not in ("http", "https"):
        return False
    if parsed_url.netloc != parsed_base.netloc:
        return False

    # Path must start with base path
    base_path = parsed_base.path.rstrip("/")
    url_path = parsed_url.path.rstrip("/")
    return url_path.startswith(base_path)


def should_skip(url: str) -> bool:
    """Check if a URL matches any skip pattern."""
    url_lower = url.lower()
    return any(re.search(pat, url_lower) for pat in SKIP_PATTERNS)


def normalize_url(url: str) -> str:
    """Normalize a URL for deduplication."""
    parsed = urlparse(url)
    # Remove fragment, normalize trailing slash
    path = parsed.path.rstrip("/") or "/"
    return f"{parsed.scheme}://{parsed.netloc}{path}"


def url_to_filename(url: str, base_url: str) -> str:
    """Convert a URL to a safe filename."""
    parsed = urlparse(url)
    base_parsed = urlparse(base_url)

    # Get the path relative to the base
    path = parsed.path
    base_path = base_parsed.path.rstrip("/")
    if path.startswith(base_path):
        path = path[len(base_path):]

    # Clean up
    path = path.strip("/")
    if not path:
        path = "index"

    # Replace slashes with double underscores
    safe = path.replace("/", "__")
    # Remove any non-alphanumeric (except underscores and hyphens)
    safe = re.sub(r"[^a-zA-Z0-9_\-]", "_", safe)
    # Truncate if too long
    if len(safe) > 120:
        safe = safe[:100] + "_" + hashlib.md5(safe.encode()).hexdigest()[:8]

    return safe + ".md"


# =============================================================================
# Fetcher
# =============================================================================


@dataclass
class FetchResult:
    url: str
    status: int = 0
    title: str = ""
    text: str = ""
    links: list[str] = field(default_factory=list)
    error: str = ""


def fetch_page(url: str, timeout: int = DEFAULT_TIMEOUT) -> FetchResult:
    """Fetch a URL and parse its HTML content."""
    result = FetchResult(url=url)
    try:
        req = Request(url, headers={"User-Agent": USER_AGENT})
        with urlopen(req, timeout=timeout) as resp:
            result.status = resp.status
            content_type = resp.headers.get("Content-Type", "")
            if "text/html" not in content_type:
                result.error = f"Not HTML: {content_type}"
                return result

            html = resp.read().decode("utf-8", errors="replace")

        parser = PageParser()
        parser.feed(html)

        result.title = parser.title
        result.text = parser.get_text()

        # Resolve relative links to absolute
        for href in parser.links:
            abs_url = urljoin(url, href)
            result.links.append(abs_url)

    except HTTPError as e:
        result.status = e.code
        result.error = f"HTTP {e.code}: {e.reason}"
    except URLError as e:
        result.error = f"URL error: {e.reason}"
    except TimeoutError:
        result.error = "Timeout"
    except Exception as e:
        result.error = f"Error: {e}"

    return result


# =============================================================================
# Crawler
# =============================================================================


@dataclass
class CrawlStats:
    pages_fetched: int = 0
    pages_saved: int = 0
    pages_skipped: int = 0
    pages_errored: int = 0
    links_discovered: int = 0


def crawl(
    base_url: str,
    output_dir: str,
    max_depth: int = DEFAULT_MAX_DEPTH,
    delay: float = DEFAULT_DELAY,
    list_only: bool = False,
) -> CrawlStats:
    """BFS crawl starting from base_url, staying within scope."""
    stats = CrawlStats()
    visited: set[str] = set()
    queue: deque[tuple[str, int]] = deque()  # (url, depth)

    base_url = base_url.rstrip("/") + "/"
    start_url = normalize_url(base_url)
    queue.append((start_url, 0))

    os.makedirs(output_dir, exist_ok=True)

    log.info("Starting crawl: %s", base_url)
    log.info("Output: %s", output_dir)
    log.info("Max depth: %d, Delay: %.1fs", max_depth, delay)
    log.info("")

    while queue:
        url, depth = queue.popleft()
        norm_url = normalize_url(url)

        if norm_url in visited:
            continue
        visited.add(norm_url)

        if not is_within_scope(norm_url, base_url):
            continue

        if should_skip(norm_url):
            stats.pages_skipped += 1
            continue

        if depth > max_depth:
            continue

        # Fetch
        log.info("[depth=%d] Fetching: %s", depth, norm_url)
        result = fetch_page(norm_url)
        stats.pages_fetched += 1

        if result.error:
            log.warning("  SKIP: %s", result.error)
            stats.pages_errored += 1
            # Don't follow links from errored pages
            continue

        # Discover new links
        for link in result.links:
            norm_link = normalize_url(link)
            if norm_link not in visited and is_within_scope(norm_link, base_url):
                if not should_skip(norm_link):
                    queue.append((norm_link, depth + 1))
                    stats.links_discovered += 1

        # Summary of what we found
        text_len = len(result.text)
        link_count = sum(
            1 for l in result.links
            if is_within_scope(normalize_url(l), base_url)
        )
        log.info("  Title: %s", result.title[:80] if result.title else "(no title)")
        log.info("  Content: %d chars, %d in-scope links", text_len, link_count)

        if list_only:
            continue

        # Save content
        if text_len < 100:
            log.info("  SKIP: too little content (%d chars)", text_len)
            stats.pages_skipped += 1
            continue

        filename = url_to_filename(norm_url, base_url)
        filepath = os.path.join(output_dir, filename)

        # Build markdown file
        md_lines = []
        md_lines.append(f"# {result.title}\n")
        md_lines.append(f"**Source**: {norm_url}\n")
        md_lines.append(f"**Scraped**: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        md_lines.append("---\n")
        md_lines.append(result.text)
        md_lines.append("")

        with open(filepath, "w", encoding="utf-8") as f:
            f.write("\n".join(md_lines))

        stats.pages_saved += 1
        log.info("  Saved: %s (%d chars)", filename, text_len)

        # Polite delay
        if queue:
            time.sleep(delay)

    return stats


# =============================================================================
# Main
# =============================================================================


def main():
    parser = argparse.ArgumentParser(
        description="ERAU College of Engineering Web Scraper",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 scraper.py                          # Full crawl with defaults
  python3 scraper.py --list-only              # Discover URLs without saving
  python3 scraper.py --depth 2 --delay 2.0    # Shallow crawl, slower
  python3 scraper.py --base-url https://daytonabeach.erau.edu/college-engineering/aerospace
        """,
    )
    parser.add_argument(
        "--base-url",
        default=DEFAULT_BASE_URL,
        help=f"Base URL to crawl (default: {DEFAULT_BASE_URL})",
    )
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT_DIR,
        help=f"Output directory (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--depth",
        type=int,
        default=DEFAULT_MAX_DEPTH,
        help=f"Maximum crawl depth (default: {DEFAULT_MAX_DEPTH})",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=DEFAULT_DELAY,
        help=f"Delay between requests in seconds (default: {DEFAULT_DELAY})",
    )
    parser.add_argument(
        "--list-only",
        action="store_true",
        help="Only list discovered URLs, don't save content",
    )
    args = parser.parse_args()

    stats = crawl(
        base_url=args.base_url,
        output_dir=args.output,
        max_depth=args.depth,
        delay=args.delay,
        list_only=args.list_only,
    )

    print()
    print("=" * 50)
    print(f"  Pages fetched:    {stats.pages_fetched}")
    print(f"  Pages saved:      {stats.pages_saved}")
    print(f"  Pages skipped:    {stats.pages_skipped}")
    print(f"  Pages errored:    {stats.pages_errored}")
    print(f"  Links discovered: {stats.links_discovered}")
    print("=" * 50)

    if not args.list_only and stats.pages_saved > 0:
        print(f"\n  Output: {args.output}/")
        print(f"  Files:  {stats.pages_saved} markdown files")

    return 0 if stats.pages_errored < stats.pages_fetched else 1


if __name__ == "__main__":
    sys.exit(main())
