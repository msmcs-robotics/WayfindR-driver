#!/usr/bin/env bash
# AMBOT RAG Ingestion Helper
#
# Usage:
#   ./ingest.sh                     # Ingest all files in knowledge/
#   ./ingest.sh file.pdf            # Ingest a single file (copies to knowledge/ first)
#   ./ingest.sh --status            # Show ingested documents
#   ./ingest.sh --clear             # Clear all documents (requires confirmation)
#
# Supported formats: .pdf .md .txt .log .json .yaml .yml

set -euo pipefail

API_URL="${RAG_API_URL:-http://localhost:8000}"
KNOWLEDGE_DIR="$(cd "$(dirname "$0")/rag/knowledge" && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()  { echo -e "${GREEN}[ingest]${NC} $*"; }
warn() { echo -e "${YELLOW}[ingest]${NC} $*"; }
err()  { echo -e "${RED}[ingest]${NC} $*" >&2; }

# Health check
check_api() {
    if ! curl -sf "${API_URL}/api/health" > /dev/null 2>&1; then
        err "Cannot reach RAG API at ${API_URL}"
        err "Start the RAG stack: cd rag && sudo docker compose up -d"
        exit 1
    fi
}

# Show ingested documents
show_status() {
    check_api
    echo "=== Ingested Documents ==="
    curl -sf "${API_URL}/api/documents" | python3 -c "
import sys, json
docs = json.load(sys.stdin)
if not docs:
    print('  (no documents)')
else:
    for d in docs:
        print(f\"  [{d['id']}] {d['filename']} ({d['chunk_count']} chunks, {d['file_size']} bytes)\")
    print(f\"\\n  Total: {len(docs)} documents, {sum(d['chunk_count'] for d in docs)} chunks\")
"
}

# Ingest a single file
ingest_file() {
    local filepath="$1"
    local filename
    filename="$(basename "$filepath")"

    log "Ingesting ${filename}..."
    response=$(curl -sf -X POST "${API_URL}/api/ingest/file" \
        -F "file=@${filepath}" 2>&1) || {
        err "Failed to ingest ${filename}"
        echo "$response" >&2
        return 1
    }
    chunks=$(echo "$response" | python3 -c "import sys,json; print(json.load(sys.stdin)['chunk_count'])" 2>/dev/null || echo "?")
    log "${filename} → ${chunks} chunks"
}

# Ingest entire knowledge directory
ingest_directory() {
    check_api
    log "Ingesting from: ${KNOWLEDGE_DIR}"

    # Count supported files
    local count=0
    for ext in pdf md txt log json yaml yml; do
        count=$((count + $(find "$KNOWLEDGE_DIR" -name "*.${ext}" 2>/dev/null | wc -l)))
    done

    if [ "$count" -eq 0 ]; then
        warn "No supported files found in ${KNOWLEDGE_DIR}"
        warn "Supported: .pdf .md .txt .log .json .yaml .yml"
        exit 0
    fi

    log "Found ${count} files to ingest"

    # Use the directory ingest endpoint (handles resume/dedup automatically)
    local docker_path="/data/knowledge"
    response=$(curl -sf -X POST "${API_URL}/api/ingest/directory" \
        -H "Content-Type: application/json" \
        -d "{\"path\": \"${docker_path}\"}" 2>&1) || {
        err "Directory ingestion failed"
        echo "$response" >&2
        exit 1
    }

    local new_docs
    new_docs=$(echo "$response" | python3 -c "import sys,json; d=json.load(sys.stdin); print(len(d))" 2>/dev/null || echo "0")
    log "Ingested ${new_docs} new documents (already-ingested files auto-skipped)"
    echo
    show_status
}

# Clear all documents
clear_all() {
    check_api
    echo "This will delete ALL ingested documents and their embeddings."
    read -rp "Type 'yes' to confirm: " confirm
    if [ "$confirm" != "yes" ]; then
        echo "Aborted."
        exit 0
    fi

    # Get all doc IDs and delete them
    curl -sf "${API_URL}/api/documents" | python3 -c "
import sys, json, urllib.request
docs = json.load(sys.stdin)
for d in docs:
    req = urllib.request.Request('${API_URL}/api/documents/' + str(d['id']), method='DELETE')
    urllib.request.urlopen(req)
    print(f\"  Deleted: {d['filename']}\")
print(f'Cleared {len(docs)} documents.')
"
}

# Main
case "${1:-}" in
    --status|-s)
        show_status
        ;;
    --clear)
        clear_all
        ;;
    --help|-h)
        echo "Usage: ./ingest.sh [file|--status|--clear]"
        echo ""
        echo "  (no args)     Ingest all files in knowledge/"
        echo "  file.pdf      Copy file to knowledge/ and ingest"
        echo "  --status      Show ingested documents"
        echo "  --clear       Delete all ingested documents"
        ;;
    "")
        ingest_directory
        ;;
    *)
        # Ingest a specific file
        filepath="$1"
        if [ ! -f "$filepath" ]; then
            err "File not found: $filepath"
            exit 1
        fi
        check_api

        # Copy to knowledge directory if not already there
        filename="$(basename "$filepath")"
        dest="${KNOWLEDGE_DIR}/${filename}"
        if [ "$(realpath "$filepath")" != "$(realpath "$dest" 2>/dev/null)" ]; then
            cp "$filepath" "$dest"
            log "Copied ${filename} to knowledge/"
        fi

        ingest_file "$dest"
        ;;
esac
