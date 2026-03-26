#!/usr/bin/env bash
#
# update_knowledge.sh — One-command scrape + clean + diff + deploy pipeline.
#
# Scrapes ERAU faculty profiles and college pages, cleans them, and shows
# a diff against the current RAG knowledge base. Optionally copies to the
# knowledge folder and triggers re-ingestion on the Jetson.
#
# Safe to run repeatedly (idempotent). Uses /tmp staging directories that
# get wiped on each run (unless --resume is passed).
#
# Usage:
#   ./update_knowledge.sh                    # Scrape + clean + show diff (dry run)
#   ./update_knowledge.sh --apply            # Scrape + clean + copy to knowledge/
#   ./update_knowledge.sh --apply --ingest   # Full pipeline: scrape -> clean -> copy -> re-ingest
#   ./update_knowledge.sh --resume           # Resume interrupted scrape
#   ./update_knowledge.sh --faculty-only     # Only scrape faculty profiles
#   ./update_knowledge.sh --college-only     # Only scrape college pages
#   ./update_knowledge.sh --skip-scrape      # Skip scraping, just clean + diff existing staging
#
set -euo pipefail

# ── Paths ─────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AMBOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
KNOWLEDGE_DIR="$AMBOT_DIR/bootylicious/rag/knowledge"

FACULTY_STAGING="/tmp/erau-faculty-staging"
COLLEGE_STAGING="/tmp/erau-scrape-staging"

JETSON_HOST="jetson"  # SSH config alias

# ── Flags ─────────────────────────────────────────────────────

FLAG_APPLY=false
FLAG_INGEST=false
FLAG_RESUME=false
FLAG_FACULTY_ONLY=false
FLAG_COLLEGE_ONLY=false
FLAG_SKIP_SCRAPE=false

for arg in "$@"; do
    case "$arg" in
        --apply)        FLAG_APPLY=true ;;
        --ingest)       FLAG_INGEST=true ;;
        --resume)       FLAG_RESUME=true ;;
        --faculty-only) FLAG_FACULTY_ONLY=true ;;
        --college-only) FLAG_COLLEGE_ONLY=true ;;
        --skip-scrape)  FLAG_SKIP_SCRAPE=true ;;
        --help|-h)
            head -25 "$0" | tail -17
            exit 0
            ;;
        *)
            echo "Unknown flag: $arg"
            echo "Run with --help for usage."
            exit 1
            ;;
    esac
done

# ── Helpers ───────────────────────────────────────────────────

timestamp() { date "+%Y-%m-%d %H:%M:%S"; }

info()  { echo "[$(timestamp)] $*"; }
warn()  { echo "[$(timestamp)] WARN: $*" >&2; }
die()   { echo "[$(timestamp)] ERROR: $*" >&2; exit 1; }

section() {
    echo ""
    echo "============================================================"
    echo "  $*"
    echo "============================================================"
}

# Check Python is available
command -v python3 >/dev/null 2>&1 || die "python3 not found"

# ── Step 1: Clean staging directories ─────────────────────────

if [ "$FLAG_SKIP_SCRAPE" = false ] && [ "$FLAG_RESUME" = false ]; then
    section "Preparing staging directories"

    if [ "$FLAG_COLLEGE_ONLY" = false ]; then
        info "Cleaning $FACULTY_STAGING"
        rm -rf "$FACULTY_STAGING"
        mkdir -p "$FACULTY_STAGING"
    fi

    if [ "$FLAG_FACULTY_ONLY" = false ]; then
        info "Cleaning $COLLEGE_STAGING"
        rm -rf "$COLLEGE_STAGING"
        mkdir -p "$COLLEGE_STAGING"
    fi
fi

# ── Step 2: Scrape ────────────────────────────────────────────

if [ "$FLAG_SKIP_SCRAPE" = false ]; then
    RESUME_FLAG=""
    if [ "$FLAG_RESUME" = true ]; then
        RESUME_FLAG="--resume"
    fi

    # Faculty profiles
    if [ "$FLAG_COLLEGE_ONLY" = false ]; then
        section "Scraping faculty profiles (all departments)"
        python3 "$SCRIPT_DIR/scrape_faculty_profiles.py" \
            --all-departments \
            --output "$FACULTY_STAGING" \
            $RESUME_FLAG \
            || warn "Faculty scrape had errors (check $FACULTY_STAGING/scrape.log)"
    fi

    # College pages
    if [ "$FLAG_FACULTY_ONLY" = false ]; then
        section "Scraping college pages"
        python3 "$SCRIPT_DIR/scraper.py" \
            --output "$COLLEGE_STAGING" \
            || warn "College scrape had errors"
    fi
fi

# ── Step 3: Clean scraped content ─────────────────────────────

section "Cleaning scraped content"

if [ "$FLAG_COLLEGE_ONLY" = false ] && [ -d "$FACULTY_STAGING" ]; then
    info "Cleaning faculty profiles..."
    python3 "$SCRIPT_DIR/clean_faculty_profiles.py" --input "$FACULTY_STAGING"
fi

if [ "$FLAG_FACULTY_ONLY" = false ] && [ -d "$COLLEGE_STAGING" ]; then
    info "Cleaning college pages..."
    python3 "$SCRIPT_DIR/cleaner.py" --input "$COLLEGE_STAGING"
fi

# ── Step 4: Diff against current knowledge base ──────────────

section "Comparing against current knowledge base"

diff_summary() {
    local staging_dir="$1"
    local knowledge_subdir="$2"
    local label="$3"
    local target_dir="$KNOWLEDGE_DIR/$knowledge_subdir"

    if [ ! -d "$staging_dir" ]; then
        info "  $label: staging dir not found, skipping"
        return
    fi

    local new_files=0
    local changed_files=0
    local unchanged_files=0

    # Count staged files
    local staged_count
    staged_count=$(find "$staging_dir" -name "*.md" -type f 2>/dev/null | wc -l)

    if [ "$staged_count" -eq 0 ]; then
        info "  $label: no files in staging"
        return
    fi

    # Compare each staged file against knowledge base
    while IFS= read -r staged_file; do
        local basename
        basename=$(basename "$staged_file")

        # Check both at the knowledge_subdir level and in subdirectories
        local found=false
        while IFS= read -r existing; do
            found=true
            if diff -q "$staged_file" "$existing" >/dev/null 2>&1; then
                unchanged_files=$((unchanged_files + 1))
            else
                changed_files=$((changed_files + 1))
                info "  CHANGED: $basename"
            fi
            break
        done < <(find "$target_dir" -name "$basename" -type f 2>/dev/null)

        if [ "$found" = false ]; then
            new_files=$((new_files + 1))
            info "  NEW: $basename"
        fi
    done < <(find "$staging_dir" -name "*.md" -type f 2>/dev/null)

    echo ""
    info "  $label summary: $staged_count staged, $new_files new, $changed_files changed, $unchanged_files unchanged"
}

if [ "$FLAG_COLLEGE_ONLY" = false ]; then
    diff_summary "$FACULTY_STAGING" "college" "Faculty profiles"
fi

if [ "$FLAG_FACULTY_ONLY" = false ]; then
    diff_summary "$COLLEGE_STAGING" "college" "College pages"
fi

# ── Step 5: Apply (copy to knowledge/) ────────────────────────

if [ "$FLAG_APPLY" = true ]; then
    section "Applying: copying to knowledge base"

    if [ "$FLAG_COLLEGE_ONLY" = false ] && [ -d "$FACULTY_STAGING" ]; then
        info "Copying faculty profiles to $KNOWLEDGE_DIR/college/"
        mkdir -p "$KNOWLEDGE_DIR/college"
        # Copy .md files, preserving subdirectory structure flattened
        find "$FACULTY_STAGING" -name "*.md" -type f -exec cp {} "$KNOWLEDGE_DIR/college/" \;
        faculty_count=$(find "$FACULTY_STAGING" -name "*.md" -type f | wc -l)
        info "  Copied $faculty_count faculty profile files"
    fi

    if [ "$FLAG_FACULTY_ONLY" = false ] && [ -d "$COLLEGE_STAGING" ]; then
        info "Copying college pages to $KNOWLEDGE_DIR/college/"
        mkdir -p "$KNOWLEDGE_DIR/college"
        find "$COLLEGE_STAGING" -name "*.md" -type f -exec cp {} "$KNOWLEDGE_DIR/college/" \;
        college_count=$(find "$COLLEGE_STAGING" -name "*.md" -type f | wc -l)
        info "  Copied $college_count college page files"
    fi

    info "Knowledge base updated."
else
    echo ""
    info "Dry run complete. To apply changes, re-run with --apply"
    info "  Staging dirs:"
    [ "$FLAG_COLLEGE_ONLY" = false ] && info "    Faculty: $FACULTY_STAGING"
    [ "$FLAG_FACULTY_ONLY" = false ] && info "    College: $COLLEGE_STAGING"
fi

# ── Step 6: Re-ingest on Jetson ───────────────────────────────

if [ "$FLAG_INGEST" = true ]; then
    if [ "$FLAG_APPLY" = false ]; then
        die "--ingest requires --apply (must copy files first)"
    fi

    section "Syncing to Jetson and triggering re-ingestion"

    # Sync knowledge folder to Jetson
    info "Syncing knowledge/ to Jetson..."
    rsync -avz --delete \
        "$KNOWLEDGE_DIR/" \
        "$JETSON_HOST:~/ambot/bootylicious/rag/knowledge/" \
        || die "rsync to Jetson failed"

    # Trigger re-ingestion
    info "Triggering re-ingestion on Jetson..."
    ssh "$JETSON_HOST" "cd ~/ambot && ./bootylicious/ingest.sh" \
        || warn "Ingestion command failed (Jetson may be offline or ingest.sh missing)"

    info "Re-ingestion complete."
fi

# ── Done ──────────────────────────────────────────────────────

section "Done"
info "Finished at $(timestamp)"
