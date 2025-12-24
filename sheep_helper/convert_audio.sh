#!/bin/bash
#
# Convert MP4 files to MP3 in case1/ and case2/ directories
#
# Usage:
#   ./convert_audio.sh           # Convert only, keep MP4 files
#   ./convert_audio.sh --delete  # Convert and delete original MP4 files
#   ./convert_audio.sh --dry-run # Show what would be done without doing it
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DELETE_ORIGINALS=false
DRY_RUN=false

# -----------------------------
# Parse arguments
# -----------------------------
for arg in "$@"; do
    case $arg in
        --delete)
            DELETE_ORIGINALS=true
            ;;
        --dry-run)
            DRY_RUN=true
            ;;
        --help|-h)
            echo "Usage: $0 [--delete] [--dry-run]"
            exit 0
            ;;
    esac
done

echo "========================================"
echo "Audio Converter - MP4 to MP3"
echo "========================================"
echo "Delete originals: $DELETE_ORIGINALS"
echo "Dry run: $DRY_RUN"
echo ""

# -----------------------------
# Check for ffmpeg
# -----------------------------
if ! command -v ffmpeg &> /dev/null; then
    echo "ERROR: ffmpeg not found"
    exit 1
fi

# -----------------------------
# Directories to process
# -----------------------------
DIRS=("$SCRIPT_DIR/case1" "$SCRIPT_DIR/case2")

converted=0
skipped=0
failed=0
deleted=0
total_saved=0

# -----------------------------
# Main loop
# -----------------------------
for dir in "${DIRS[@]}"; do
    [ -d "$dir" ] || continue

    echo "Processing: $dir"
    echo "----------------------------------------"

    shopt -s nullglob
    for mp4_file in "$dir"/*.mp4; do
        base="${mp4_file%.mp4}"
        mp3_file="${base}.mp3"

        # -----------------------------
        # Case 1: MP3 already exists
        # -----------------------------
        if [ -f "$mp3_file" ]; then
            echo "  MP3 exists → $(basename "$mp3_file")"
            ((skipped++))

            if [ "$DELETE_ORIGINALS" = true ]; then
                if [ "$DRY_RUN" = true ]; then
                    echo "  [DRY RUN] Would delete MP4: $(basename "$mp4_file")"
                else
                    rm -f "$mp4_file"
                    echo "  Deleted MP4: $(basename "$mp4_file")"
                    ((deleted++))
                fi
            fi
            continue
        fi

        # -----------------------------
        # Case 2: Convert MP4 → MP3
        # -----------------------------
        echo "  Converting: $(basename "$mp4_file") → $(basename "$mp3_file")"

        if [ "$DRY_RUN" = true ]; then
            ((converted++))
            continue
        fi

        if ffmpeg -i "$mp4_file" -vn -acodec libmp3lame -q:a 2 \
            "$mp3_file" -y -loglevel error; then

            if [ -f "$mp3_file" ]; then
                ((converted++))

                # Delete original after successful conversion
                if [ "$DELETE_ORIGINALS" = true ]; then
                    rm -f "$mp4_file"
                    echo "  Deleted MP4: $(basename "$mp4_file")"
                    ((deleted++))
                fi
            else
                echo "  FAILED: MP3 not created"
                ((failed++))
            fi
        else
            echo "  FAILED: ffmpeg error"
            ((failed++))
        fi
    done
    shopt -u nullglob
done

# -----------------------------
# Summary
# -----------------------------
echo ""
echo "========================================"
echo "Summary"
echo "========================================"
echo "  Converted: $converted"
echo "  Skipped (MP3 existed): $skipped"
echo "  Deleted MP4s: $deleted"
echo "  Failed: $failed"
echo ""
echo "Done!"
