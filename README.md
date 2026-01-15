# Seed Bundle (Blueprint SPEC + Reheat + Plan Mode)

You want the repo to be reconstructable from a small “seed”:
`README.md` + `CLAUDE.md` + `SPEC.md` (+ `algorithms/` for math-heavy repos), and `RESUME.md` for session continuity.

This system keeps cognitive load low by giving each file **one job** and using 3 seed commands to keep the blueprint current.

## Roles (one job per file)

- `README.md` — human-facing overview (install/usage)
- `CLAUDE.md` — agent runbook (how to run/build/test + conventions)
- `RESUME.md` (Reheat) — session continuity (working/broken, failures, repro, next step)
- `SPEC.md` — rebuildable blueprint (architecture, contracts, invariants, acceptance; split external vs internal so you know what breaks neighbors vs just yourself)
- `algorithms/*.md` (optional) — per-algorithm math contracts (frames/units/equations), linked from `SPEC.md`

## Quick Reference (what goes where)

| File | Primary Reader | Job | Put Here | Don’t Put Here |
|------|----------------|-----|----------|----------------|
| `README.md` | Humans | Explain what it is and how to use it | install, usage examples, high-level behavior | session status, deep implementation notes |
| `CLAUDE.md` | AI agent | Runbook for working in the repo | build/test/run commands, repo conventions, “where things live” | session handoff, long specs/blueprints |
| `RESUME.md` | Next session (any agent) | Session continuity | working/broken, failures, exact repro, next action | durable architecture/spec (it will rot here) |
| `SPEC.md` | Future agent + you | Rebuildable blueprint | architecture, interfaces/contracts, invariants, acceptance signals | logs, stack traces, “next steps right now” |
| `algorithms/*.md` | You + math agent | Durable algorithm truth | definitions, units/frames, equations, invariants, code mapping | session logs, runbook/setup, task lists |

Routing rule:
- “How do I run/build/test?” → `CLAUDE.md`
- “What’s broken/what next right now?” → `RESUME.md`
- “What must be true / how is it structured?” → `SPEC.md`
- “What is the actual math/algorithm?” → `algorithms/*.md` (indexed from `SPEC.md`)
- “How does a human install/use it?” → `README.md`
- Prefer diagrams when they replace lots of bullets; don’t delete them unless they’re wrong.

## How This Differs From Agent OS and Spec Kit

- **Agent OS**: Drives a spec-first pipeline (shape spec → write spec → create tasks → implement/verify) so agents can “ship on the first try”. It does this by installing multi-phase command chains plus optional subagents/standards injection to reduce ambiguity and enforce consistency.
- **Spec Kit**: Drives a constitution/spec/plan/tasks workflow (often with gates like clarification, task generation, and cross-artifact analysis). It does this by generating and keeping multiple structured artifacts, then checking alignment between them to prevent drift.
- **This (Seed Bundle + Reheat)**: Optimizes for evolutionary solo dev where requirements change as you build. It does this by separating **session continuity** (`RESUME.md`) from a **rebuildable blueprint** (`SPEC.md`) and keeping planning ephemeral (Plan Mode), with minimal commands to audit/align/prune the seed and reduce cognitive load.
  - If the repo is math/controls/ML-heavy, add per-algorithm contracts in `algorithms/*.md` and index them from `SPEC.md`.

## Commands (copy into your slash-command folder)

- `/seed-align` — write: align seed docs with repo reality (+ `RESUME.md`)
- `/seed-audit` — read-only: drift report (“is the seed still true?”)
- `/seed-prune` — write: compact/de-duplicate seed docs without changing meaning

## Workflow (expected)

Note: `/reheat:*` commands come from the Reheat plugin; `seed-*` commands are milestone maintenance for the seed bundle.

### Daily loop (session continuity)

1. Start: `/reheat:resume`
2. Plan: Plan Mode
3. Build/test
4. End: `/reheat:save-quick` (or `/reheat:save`)

### Milestones (make the repo rebuildable)

1. `/seed-align` after meaningful progress (new module, new interface, major refactor)
2. `/seed-audit` if you suspect drift or inconsistencies
3. `/seed-prune` when `SPEC.md` gets long
4. If you change math/controls/ML: update the relevant `algorithms/*.md` and re-run `/seed-align` to keep the index current

## Why this reduces context rot

- Reheat prevents *session* context loss.
- `SPEC.md` prevents *project* context loss by recording the blueprint the code now embodies.
- `algorithms/*.md` prevents *math drift* by keeping frames/units/equations stable and mapped to code.
- `seed-*` commands reduce manual writing by promoting what exists in code into a concise seed.

## Examples

- [`examples/rcm_qp_drake/SPEC.md`](examples/rcm_qp_drake/SPEC.md)
- [`examples/ur_admittance_controller/SPEC.md`](examples/ur_admittance_controller/SPEC.md)
