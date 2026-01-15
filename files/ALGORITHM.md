# ALGORITHM.md (per-algorithm contract + derivation)

Put one of these in `algorithms/<name>.md` (or `algorithms/<package>/<name>.md` for monorepos).
This is the **durable math truth**: derivations, equations, frames, units, invariants, and how it maps to code.

The math derivation is the primary narrative. Implementation details live in Obsidian callouts alongside the math they implement.

---

# <Algorithm Name>

## Purpose

- Problem it solves:
- Where it's used (high level):

## Inputs (symbols, units, frames)

| Symbol | Code name | Meaning | Units | Frame |
|--------|-----------|---------|-------|-------|
| $<symbol>$ | `<var>` | <meaning> | <units> | <frame> |

## Outputs (symbols, units, frames)

| Symbol | Code name | Meaning | Units | Frame |
|--------|-----------|---------|-------|-------|
| $<symbol>$ | `<var>` | <meaning> | <units> | <frame> |

## Conventions (must not drift)

- Frames:
- Sign conventions:
- Coordinate conventions:
- Ordering conventions (e.g., `[linear; angular]` vs `[ω; v]`):

---

## Derivation

This section contains the full mathematical derivation. Use Obsidian callouts to link math to implementation.

### Callout conventions

- `> [!implementation]` — function name, file, pseudo-code, output mapping
- `> [!note]` — code variable names, implementation choices
- `> [!warning]` — gotchas, assumptions that affect correctness
- `> [!todo]` — open questions, things to verify

### Model

<State the mathematical model / governing equations.>

### Given

<What is known / provided as input.>

### To Find

<What parameters or outputs we need to estimate / compute.>

### Method Overview

<Optional: high-level diagram or pipeline summary.>

```
[Step 1] ──► output_1
    │
    ▼
[Step 2] ──► output_2
    ...
```

---

### Step 1: <Step Name>

<Mathematical derivation for this step.>

$<key equation>$

> [!implementation] `<function_name>()`
> **File**: `<file_path>`
> **Inputs**: `<var1>`, `<var2>`
> **Output**: `<var>` → maps to $<symbol>$
> **Pseudo-code** (if non-trivial):
> ```
> H ← build_matrix(...)
> y* ← eigenvector of min eigenvalue of H'H
> return recover_x(y*)
> ```

<Continue derivation...>

> [!note] Symbol mapping
> $<math_symbol>$ → `<code_var>` in implementation

<If referencing theory:>
See [[<concept_note>]] for background.

---

### Step 2: <Step Name>

<Repeat pattern: math → callout → math...>

---

## Invariants / Guarantees

- What must remain true even after refactors:

## Implementation Mapping (summary)

| Step | Function | File | Responsibility |
|------|----------|------|----------------|
| 1 | `<func>()` | `<file>` | <what it does> |
| 2 | `<func>()` | `<file>` | <what it does> |

- Integration points:
  - `<file>` — where algorithm is called / orchestrated

## Defaults (tunable starting points)

These are **implementation aids**, not contracts.

| Name | Value | Units | Purpose |
|------|-------|-------|---------|
| `<name>` | `<value>` | `<units>` | <why it exists> |

## Acceptance Signals (how to validate)

- `<check>` → `<expected result>`

## Known Pitfalls (avoid / gotchas)

- `<pitfall>` — `<why>` — `<how to avoid>`

## Open Questions

- [ ] `<question that could change the algorithm>`
