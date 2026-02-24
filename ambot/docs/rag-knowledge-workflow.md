# RAG Knowledge Base Workflow

## Knowledge Folder

All documents for the RAG system go in:

```
ambot/bootylicious/rag/knowledge/
```

This folder is mounted into the Docker container at `/data/knowledge:ro`. Any files placed here are available for ingestion.

**Supported formats**: `.pdf .md .txt .log .json .yaml .yml`

## Adding Documents

### Option 1: Place files directly on the Jetson
```bash
ssh jetson
cp /path/to/documents/*.pdf ~/ambot/bootylicious/rag/knowledge/
```

### Option 2: Sync from dev machine
```bash
rsync -avz ambot/bootylicious/rag/knowledge/ jetson:~/ambot/bootylicious/rag/knowledge/
```

### Option 3: SCP individual files
```bash
scp my_document.pdf jetson:~/ambot/bootylicious/rag/knowledge/
```

**Note**: `./deploy.sh jetson bootylicious` intentionally **excludes** `knowledge/` from sync to prevent overwriting docs already on the Jetson. Use the commands above to sync knowledge files specifically.

## Ingesting Documents

After documents are in the knowledge folder on the Jetson:

```bash
# Ingest all files in knowledge/
ssh jetson "cd ~/ambot && ./bootylicious/ingest.sh"

# Ingest a single file
ssh jetson "cd ~/ambot && ./bootylicious/ingest.sh bootylicious/rag/knowledge/my_doc.pdf"

# Check what's already ingested
ssh jetson "cd ~/ambot && ./bootylicious/ingest.sh --status"

# Clear all ingested documents (requires confirmation)
ssh jetson "cd ~/ambot && ./bootylicious/ingest.sh --clear"
```

Or via the deploy script:
```bash
./deploy.sh jetson --test=rag-docs    # List ingested documents
```

## How Ingestion Works

1. Files are read and split into chunks (512 chars, 50 char overlap)
2. Junk chunks are filtered (TOC pages, index entries with >25% digits)
3. Each chunk is embedded using MiniLM (384-dim, CPU, ~50ms per chunk)
4. Embeddings are stored in PostgreSQL with pgvector
5. Content hashes prevent re-ingesting unchanged files (resume/dedup)
6. Batch commits (25 chunks) allow resuming if interrupted

## Online Scraper

The `ambot/online_scraper/` folder contains tools for scraping public university websites into text files. Once scraped, copy the output into the knowledge folder for ingestion.

```bash
# Scrape → knowledge → ingest pipeline:
cd ambot/online_scraper
python3 scraper.py                           # 1. Scrape content
cp output/*.md ../bootylicious/rag/knowledge/  # 2. Copy to knowledge
cd .. && ./bootylicious/ingest.sh            # 3. Ingest (on Jetson)
```

## Testing

```bash
./deploy.sh jetson --test=rag       # Full: sync + rebuild + 10-test suite
./deploy.sh jetson --test=rag-test  # Test suite only (no rebuild)
./deploy.sh jetson --test=rag-health # Quick API health check
```
