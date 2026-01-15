# CLAUDE.md (example docs set)

## Docs Map (read in this order)

- @README.md — human install/usage
- @SPEC.md — rebuildable blueprint (what must remain true)
- @algorithms/ — math contracts (index in @SPEC.md → “Algorithm Index”)
- @RESUME.md (if present) — session continuity (Reheat)

## Project Overview

QP-based Remote Center of Motion (RCM) controller for UR5e surgical robotics.
A probe pivots through a trocar (fixed entry point) while controlling contact forces.
**Same code runs simulation AND hardware** (Digital Twin pattern).

## Architecture (conceptual)

```
┌─────────────────────────────────────────────────────────────┐
│                     src/demo.py                             │
│        Entry point: --sim, --hardware, --mode               │
└─────────────────────────────────────────────────────────────┘
                          │
          ┌───────────────┴───────────────┐
          ▼                               ▼
┌─────────────────────┐     ┌─────────────────────────────────┐
│  src/controller.py  │     │     driver/ur5e_driver.py       │
│    (Pure Math)      │     │       (Digital Twin)            │
│                     │     │                                 │
│  rcm_step()         │     │  Same API for sim & hardware    │
│  solve_rcm_qp()     │     │  get_state(), send_velocity()   │
│  hybrid_force_step()│     │  get_tool_wrench_W()            │
└─────────────────────┘     └─────────────────────────────────┘
```

## Code Style

- Minimalistic — less code is better
- Single source of truth — no duplication
- Fail cleanly — print message, exit
