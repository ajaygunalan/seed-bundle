---
description: Shorten and de-duplicate seed docs (SPEC+algorithms) without changing meaning (keep them coherent and reconstructable).
---

## Input

```text
$ARGUMENTS
```

## Process

1. Read `SPEC.md`. If missing, suggest `/seed-align`.
2. Trim rules:
   - Replace prose with bullets.
   - Merge duplicates; remove contradictions.
   - Preserve the “External Contracts vs Internal Conventions” split (don’t trim away interop contracts).
   - Remove session-specific content (belongs in `RESUME.md`).
   - Remove runbook commands/setup (belongs in `CLAUDE.md`/`README.md`).
   - If `SPEC.md` contains derivations/math details or tuning constants, extract them into `algorithms/*.md`:
     - Keep only links + invariants in `SPEC.md`.
     - Put constants under “Defaults (tunable)” (not as hard contracts) unless explicitly stated otherwise.
3. Preserve the blueprint shape (keep headings).
4. Output what got removed/condensed and why.
