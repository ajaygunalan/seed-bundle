---
description: Read-only drift report: does the seed bundle still match the repo (contracts, paths, entrypoints, interfaces, acceptance signals)?
---

## Input

```text
$ARGUMENTS
```

## READ-ONLY Process

1. Read `SPEC.md`. If missing, report and stop.
2. Read `README.md` and `CLAUDE.md` (if present) and note any direct contradictions with `SPEC.md` (surface-level check).
3. Validate (high signal only):
   - “System Invariants” is split into **External Contracts** and **Internal Conventions** (missing split = HIGH drift).
   - Every file path in the “Repo Map/Blueprint” exists.
   - Every link in “Algorithm Index” exists (if present).
   - Entrypoints/interfaces described exist and roughly match names/signatures.
   - Acceptance signals referenced exist (tests/scripts/commands), or are clearly labeled “manual”.
4. Output a compact drift table:
   - Severity: CRITICAL / HIGH / LOW
   - Location: SPEC section
   - Problem: what’s stale/false
   - Fix: run `/seed-align` or edit seed docs manually
