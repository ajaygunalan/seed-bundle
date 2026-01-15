# ALGORITHM.md (per-algorithm contract)

Put one of these in `algorithms/<name>.md` (or `algorithms/<package>/<name>.md` for monorepos).
This is the **durable math truth**: frames, units, equations, invariants, and how it maps to code.

Keep it short: if it doesn’t prevent a future mistake, omit it.

---

# <Algorithm Name>

## Purpose

- Problem it solves:
- Where it’s used (high level):

## Inputs (symbols, units, frames)

- Inputs:
  - `<symbol>` — `<meaning>` — `<units>` — `<frame>`

## Outputs (symbols, units, frames)

- Outputs:
  - `<symbol>` — `<meaning>` — `<units>` — `<frame>`

## Conventions (must not drift)

- Frames:
- Sign conventions:
- Coordinate conventions:

## Core Logic / Equations (minimal)

- Summary:
- Key equations / steps:

## Invariants / Guarantees

- What must remain true even after refactors:

## Implementation Mapping (code pointers)

- Primary implementation:
  - `<file path>` — `<function/class>` — `<responsibility>`
- Call sites / integration points:
  - `<file path>` — `<where it plugs in>`

## Defaults (tunable starting points)

These are **implementation aids**, not contracts. Include only values that materially affect behavior or prevent common mistakes.

- `<name>` = `<value>` — `<units>` — why it exists / what it tunes

## Acceptance Signals (how to validate)

- Fast checks:
  - `<command/test/manual check>` → `<expected result>`

## Known Pitfalls (avoid / gotchas)

- `<pitfall>` — `<why>` — `<how to avoid>`

## Open Questions

- [ ] `<question that could change the algorithm>`
