---
description: Align the seed bundle (README+CLAUDE+SPEC+algorithms) with repo reality (+ RESUME.md), keeping the repo reconstructable.
---

## Input

```text
$ARGUMENTS
```

## Rules

- `RESUME.md` is session continuity (working/broken, failures, repro, next step). Don’t copy its logs into `SPEC.md`.
- `SPEC.md` is the durable blueprint: architecture, contracts, invariants, acceptance.
- If the repo depends on math/controls/ML, put durable derivations/contracts in `algorithms/*.md` and only **index** them from `SPEC.md`.
- In `SPEC.md` → “System Invariants”, separate:
  - **External Contracts** (must match users/neighbor repos/libraries/hardware)
  - **Internal Conventions** (repo-local; can change if you update all internal call sites)
- Constants belong in `algorithms/*.md` as **Defaults (tunable)**, not as “contracts”, unless the user explicitly says they are fixed.
- If a mismatch implies a choice (change code vs change seed docs), ask the user; don’t guess.
- If uncertain, write an **assumption** or **open question**, not a fact.
- Prefer bullets. Keep it compact.
  - If a contract boundary is unclear, ask the user up to **3** short questions before writing it as “external”.

## Process

1. Read `README.md`, `CLAUDE.md`, and `RESUME.md` (if present).
2. Find any algorithm docs:
   - Prefer: `algorithms/**/*.md`
   - Also consider existing math docs (ex: `docs/*`, `src/*.md`) and index them instead of duplicating.
3. Scan the repo for:
   - Entrypoints (CLI/scripts/services) and their inputs/outputs
   - Major modules/components and responsibilities
   - Config files and their schema/meaning
   - Multi-repo neighbors (packages/services/hardware) and what must match them
   - Tests/smoke checks used as acceptance signals
4. Create `SPEC.md` if missing (use the standard blueprint headings).
5. Update `SPEC.md` so it matches reality:
   - North Star + non-goals
   - System invariants (External Contracts vs Internal Conventions)
   - Architecture + repo map (key directories/files)
   - Algorithm Index: link each core algorithm doc + “when to read”
   - Interfaces/contracts (what must remain stable)
   - Acceptance signals (how we know it’s correct)
   - Decisions + assumptions + open questions
6. Ensure `CLAUDE.md` has a minimal “Docs map” pointer (don’t rewrite it). If `CLAUDE.md` is missing, create a minimal one containing only these pointers:
   - “Blueprint: `SPEC.md`”
   - “Math contracts: `algorithms/` (see `SPEC.md` → Algorithm Index)”
   - “Session continuity: `RESUME.md` (Reheat), if present”
7. Output:
   - Sections changed (short list)
   - Any new assumptions/questions introduced
   - Suggested next command: `/seed-audit` or `/seed-prune`
