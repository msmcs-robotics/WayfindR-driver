# RAG Optimization Review — Adapting rag-atc-testing Patterns for AMBOT

> Date: 2026-02-17
> Context: Reviewing ~/exudeai/rag-atc-testing/ for optimization patterns applicable to AMBOT's Jetson Orin Nano RAG system (7.4 GiB RAM, Orin GPU)

## Current AMBOT RAG Stack

| Component | Current Setting | Notes |
|-----------|----------------|-------|
| Embedding model | all-MiniLM-L6-v2 (384-dim) | sentence-transformers, local |
| LLM model | llama3.2:3b (Q4_K_M) | Via Ollama, ~2 GB |
| Chunk size | 512 tokens | Whitespace-delimited |
| Chunk overlap | 50 tokens | Sentence-boundary aware |
| Search | Hybrid (70/30 semantic/keyword) | RRF fusion, k=60 |
| RAG Top-K | 3 chunks | Minimal context |
| DB pool | 20 connections, 10 overflow | May be oversized |
| Redis cache | 30-day TTL, allkeys-lru | Embedding cache |

## LLM Model Comparison (Tested 2026-02-17)

| Model | Size | Quality | Context Grounding |
|-------|------|---------|-------------------|
| tinyllama (1B) | 637 MB | Poor — hallucinated completely | Ignored retrieved context |
| llama3.2:3b (Q4_K_M) | 2.0 GB | Good — accurately identified components | Used retrieved context correctly |

**Recommendation**: Use llama3.2:3b as default. TinyLlama is only useful for pipeline smoke testing.

### Resource Usage with llama3.2:3b

| Component | Memory |
|-----------|--------|
| Docker containers | ~148 MiB (API 91 + PG 46 + Redis 11) |
| Ollama server | ~106 MiB |
| System total | ~5.5 GiB of 7.4 GiB |
| **Headroom** | **~1.9 GiB** |

## Optimization Opportunities (from rag-atc-testing)

### Priority 1: High Impact, Easy to Implement

#### 1.1 Embedding Model Upgrade — nomic-embed-text
- **Current**: all-MiniLM-L6-v2 (384-dim, 22 MB, 512-token context)
- **Recommended**: nomic-embed-text v1.5 (768-dim, 548 MB, 8192-token context)
- **Why**: 8x context window, better retrieval quality (MTEB 62.39 vs ~52)
- **Trade-off**: Larger vectors (768 vs 384), needs re-embedding all docs
- **Effort**: Config change + re-ingest

#### 1.2 Database Indexes
- **Current**: No indexes on vector column or FTS
- **Recommended**: Add after first real ingestion:
  ```sql
  CREATE INDEX ON chunks USING ivfflat (embedding vector_cosine_ops) WITH (lists = 100);
  CREATE INDEX ON chunks USING GIN (to_tsvector('english', content));
  ```
- **Impact**: 5-10x faster search for 1k+ chunks
- **Effort**: One SQL migration

#### 1.3 Connection Pool Reduction
- **Current**: pool_size=20, max_overflow=10
- **Recommended**: pool_size=5, max_overflow=5, pool_pre_ping=True
- **Why**: Jetson has few concurrent requests; 20 connections wastes memory
- **Effort**: Config change in database.py

#### 1.4 Redis Eviction Policy
- **Current**: allkeys-lru (least recently used)
- **Recommended**: allkeys-lfu (least frequently used)
- **Why**: Retains hot embeddings better during re-ingestion cycles
- **Effort**: One line in docker-compose.yml

### Priority 2: Medium Impact, Moderate Effort

#### 2.1 Junk Chunk Filtering
- **Current**: No filtering — all chunks embedded
- **Recommended**: Filter chunks with >25% digits or >10% dot leaders
- **Impact on FAA corpus**: Removed 10.4% junk chunks (552/5329)
- **Why**: Junk chunks waste embedding compute and pollute search results
- **Effort**: Add `is_junk_chunk()` to ingestion.py

#### 2.2 Text Normalization for Embeddings
- **Current**: Raw text passed to embedding model
- **Recommended**: Normalize unicode, cap at 800 chars, strip null bytes
- **Why**: Prevents Ollama model runner crashes on dense notation
- **Effort**: Add `_normalize_text()` to embeddings.py

#### 2.3 Embedding Retry with Progressive Truncation
- **Current**: Single attempt, no retry
- **Recommended**: Retry up to 8 times with exponential backoff; on failure, truncate to 600 chars and retry
- **Why**: Ollama model runner can crash on specific text patterns
- **Effort**: Wrap embed call with retry logic

#### 2.4 Sequential Embedding with Cooldown
- **Current**: embed_batch processes sequentially (already correct)
- **Recommended**: Add 1.5s inter-request delay for Ollama backend
- **Why**: Prevents Ollama model runner overload on constrained hardware
- **Effort**: Add asyncio.sleep() in embed loop

#### 2.5 Batch Ingestion with Resume
- **Current**: Per-file commits, no resume
- **Recommended**: Commit every 25 chunks, detect resume point from DB chunk count
- **Why**: Large ingestions survive interruptions without re-processing
- **Effort**: Modify ingestion loop

### Priority 3: Advanced, Higher Effort

#### 3.1 Dual Keyword Search (English + Simple)
- **Current**: Single `plainto_tsquery('english', query)`
- **Recommended**: Run both English (stemmed AND) and Simple (exact OR), merge with 20% boost for overlap
- **Why**: Preserves acronyms that English stemming mangles
- **Impact**: Acronym queries improved from 0% to 95% accuracy
- **Effort**: Extend search.py keyword function

#### 3.2 Adaptive Semantic Weight
- **Current**: Fixed 70/30 semantic/keyword
- **Recommended**: Adapt based on query characteristics:
  - Short acronym queries (≤2 words, >50% uppercase): 20/80
  - Contains acronym: 30/70
  - Short natural language (≤4 words): 50/50
  - Long natural language (>4 words): 70/30
- **Effort**: Add weight function in search.py

#### 3.3 Acronym Expansion Table
- **Current**: No acronym handling
- **Recommended**: Load domain-specific acronym table (JSON), expand acronyms in semantic queries
- **Why**: "What is a TAC?" → "What is a TAC? TRAJECTORY ALTERING CLEARANCE"
- **Effort**: Create acronyms.json + expand function

#### 3.4 Task-Specific Embedding Prefixes
- **Current**: No prefix differentiation
- **Recommended**: Add `search_document:` prefix for ingestion, `search_query:` for queries (nomic-embed-text specific)
- **Impact**: 2-5% embedding quality improvement
- **Effort**: Conditional prefix logic in embeddings.py

#### 3.5 Embedding Dimension Reduction (Matryoshka)
- **Current**: Full 384-dim (MiniLM) or 768-dim (nomic)
- **Recommended**: nomic supports 768→512→256 dimension reduction
- **Why**: Smaller vectors = faster search + less storage
- **Trade-off**: Slight quality loss below 512-dim
- **Effort**: Config change + re-embed

### Priority 4: Nice-to-Have

#### 4.1 5x Fetch Multiplier for Hybrid Search
- **Current**: fetch_limit = limit * 3
- **Recommended**: fetch_limit = limit * 5
- **Why**: Better RRF fusion quality (correct keyword results less likely to fall outside window)
- **Effort**: One constant change

#### 4.2 Stop Word Filtering for Keyword OR Queries
- **Current**: `plainto_tsquery` handles stop words natively
- **Recommended**: Custom stop word filtering for OR-based queries
- **Effort**: Add stop word set + filter function

#### 4.3 Content Hash Deduplication
- **Current**: Already implemented (SHA256 content_hash)
- **Status**: Working correctly

## Implementation Roadmap

### Phase 1: Quick Wins (< 1 hour each)
1. ~~Switch to llama3.2:3b~~ — **Done** (2026-02-17)
2. Reduce DB connection pool (5/5)
3. Switch Redis to allkeys-lfu
4. Add 5x fetch multiplier
5. Update .env.example with new defaults

### Phase 2: Resilience (1-2 hours each)
6. Add junk chunk filtering
7. Add text normalization for embeddings
8. Add embedding retry with progressive truncation
9. Add inter-request cooldown (1.5s)
10. Add batch commit + resume for ingestion

### Phase 3: Search Quality (2-4 hours each)
11. Switch embedding model to nomic-embed-text
12. Add database indexes (IVFFlat + GIN)
13. Implement dual keyword search (English + Simple)
14. Add adaptive semantic weight
15. Create domain acronym table

### Phase 4: Advanced (future)
16. Task-specific embedding prefixes
17. Matryoshka dimension reduction
18. Search quality test suite (port from rag-atc-testing)

## Files to Modify

| File | Changes |
|------|---------|
| `rag/.env.example` | New defaults: llama3.2:3b, nomic-embed-text, pool sizes |
| `rag/app/config.py` | Add EMBEDDING_CONCURRENCY, EMBED_COOLDOWN settings |
| `rag/app/database.py` | Reduce pool_size, add pool_pre_ping |
| `rag/app/embeddings.py` | Text normalization, retry logic, cooldown |
| `rag/app/ingestion.py` | Junk filtering, batch commit, resume detection |
| `rag/app/search.py` | Dual keyword, adaptive weight, acronym expansion |
| `rag/docker-compose.yml` | Redis LFU policy, memory tuning |
