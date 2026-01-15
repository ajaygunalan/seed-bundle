# SPEC.md (Blueprint)

This file is the repo’s **rebuildable specification**.
If the code vanished tomorrow, `README.md` + `CLAUDE.md` + `SPEC.md` (+ `algorithms/` for math-heavy repos) should let an AI agent recreate most of the repository.

Scope:
- Don’t duplicate runbook/setup (belongs in `README.md`/`CLAUDE.md`) or session continuity (belongs in `RESUME.md`).
- If the repo depends on math/controls/ML, put durable derivations/contracts in `algorithms/*.md` and index them here.
- Prefer a small diagram when it replaces many bullets.

---

## North Star

- Problem:
- Success looks like:
- Non-goals (for now):

## System Invariants (must remain true)

### External Contracts (breaking these breaks interop)

Inbound constraints you must match (users/neighbor repos/libraries/hardware). Avoid duplicating what you expose—reference “Interfaces & Contracts” when needed.

- User-facing surface:
  - [What the user/CLI/UI relies on (canonical usage lives in `README.md`/`CLAUDE.md`).]
- Repo-to-repo:
  - [Topics/APIs/files/interfaces owned by *other* packages or services that you must match.]
- Library/Hardware:
  - [Assumptions dictated by external deps (message types, ordering, frame naming, protocol calls).]

### Internal Conventions (breaking these breaks ourselves)

- [Repo-local contracts (file names/paths, responsibilities, data formats). Changing these requires updating internal references.]

## Behavior (what it must do)

- Core scenarios:
  - [Scenario] → [Expected outcome]
- Failure modes:
  - [Failure] → [Expected handling]

## Architecture (what exists and why)

- Major components/modules:
  - [Component] — responsibility — key file paths (only where it prevents confusion)
- Data/control flow:
  - [High-level flow, 3–7 bullets]

## Algorithm Index (math contracts, optional)

- When the repo’s behavior depends on math/ML/controls, keep one file per algorithm in `algorithms/`.
- Each algorithm doc should define: inputs/outputs + units/frames, core equations/logic, invariants, code mapping, acceptance signals.
- Index (add links + “when to read”):
  - `algorithms/<algo>.md` — [what it covers] — read when changing [module/behavior]

## Interfaces & Contracts (things future code must preserve)

Outbound interfaces this repo defines (what others may depend on). If something here has external consumers, treat it as interop-critical.

- Entrypoints (CLI/services):
  - [Entrypoint] — where — inputs/flags — outputs/side effects
- Public APIs:
  - [Class/function] — contract (expects/guarantees)
- Configuration & data formats:
  - [File] — schema/fields — meaning

## Repo Blueprint (key file map)

Treat file paths listed here as repo-local contracts: renaming/moving requires updating internal references.

```text
# Keep this as the minimal “shape of the repo”.
```

## Acceptance Signals (how we know it’s correct)

- Smoke checks (manual or scripted):
  - [Check] → [Expected result]
- Tests (if any):
  - [Test name/file] → [What it proves] (how to run lives in `README.md`/`CLAUDE.md`)

## Decisions (durable tradeoffs)

- YYYY-MM-DD — Decision → Why → Tradeoff

## Assumptions (unverified / TBD)

- [Clearly marked hypothesis.]

## Open Questions

- [ ] [Question that could change architecture/spec.]
