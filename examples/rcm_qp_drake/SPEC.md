# SPEC.md (Blueprint) — `rcm_qp_drake` (example)

This `SPEC.md` is a **rebuildable blueprint** for the repo.
If the code vanished tomorrow, `README.md` + `CLAUDE.md` + `SPEC.md` + `algorithms/` should let an AI agent recreate most of it.

Scope: this file avoids run commands and session logs (see `README.md`/`CLAUDE.md` and `RESUME.md`).

---

## North Star

- Problem: Control a UR5e-held probe that must pivot through a fixed trocar (RCM constraint) while optionally regulating contact force with tissue.
- Success looks like:
  - Same “RCM demo” code runs in simulation and on hardware (Digital Twin driver interface).
  - RCM constraint is enforced via a convex QP on joint velocities.
  - Hybrid mode regulates insertion force while scanning tangentially.
- Non-goals (for now): full surgical workflow, autonomy, perception, path planning, formal safety certification.

## System Invariants (must remain true)

### External Contracts (breaking these breaks interop)

- Drake spatial Jacobian ordering is `[ω; v]` (dictated by Drake); controller math assumes this ordering.
- UR RTDE interface:
  - Velocity command uses `ur_rtde` `speedJ(qd, acceleration, time)` with `qd` in joint order (rad/s).
  - Feedback uses `getActualQ()` / `getActualQd()` for joint state.
- NetFT interface:
  - Raw measurement format is 6D `[fx, fy, fz, tx, ty, tz]` in `[N, N·m]` (before internal compensation).
- Model frames referenced by control must exist in the Drake model:
  - Shaft frame: `flange`
  - Tip frame: `tcp`

### Internal Conventions (breaking these breaks ourselves)

- Force mode requires calibration: wrench processing expects `config/wrench_calibration.yaml` with required keys (see “Configuration & data formats”).
- This repo treats stale/old calibration schemas as invalid input (fail fast; re-run calibration).

## Behavior (what it must do)

- Core scenarios:
  - Simulation RCM pivoting: probe pivots about the trocar while maintaining the RCM constraint, with visualization.
  - Simulation hybrid force/motion: tangential scan while regulating insertion force (demo-stable, not production).
  - Hardware mode: same entrypoint drives a real UR5e via RTDE velocity commands through the Digital Twin driver interface.
- Failure modes:
  - Hardware mode without an IP is rejected (fail fast).
  - Force mode without NetFT installed/configured errors explicitly (no silent fallback).
  - Missing/malformed calibration YAML errors explicitly.

## Architecture (what exists and why)

- Major components (see `CLAUDE.md` for the file-level diagram):
  - Entrypoint + loop: mode dispatch, sim scene build, visualization, control loop.
  - Controller math: RCM QP + optional hybrid force step (kept pure-math; I/O lives in the driver layer).
  - Digital Twin driver: same loop surface for sim/hardware; owns kinematics plant; RTDE velocity output; optional NetFT wrench.
  - Wrench pipeline: calibration + filtering/compensation shared across sim/hardware.
  - Assets/config: robot/probe/tissue models and calibration outputs.
- Data/control flow (RCM loop):
  - Read `q` → compute desired twist `V_des` → solve QP for `v_opt` → send velocity → step.

## Algorithm Index (math contracts, optional)

- [`algorithms/rcm_qp.md`](algorithms/rcm_qp.md) — RCM-only QP (constraints/cost/conventions) — read before changing `src/controller.py:solve_rcm_qp` or `compute_*`.
- [`algorithms/rcm_and_force.md`](algorithms/rcm_and_force.md) — hybrid force/motion + barrier logic — read before changing `src/controller.py:hybrid_force_step` or force integration in `src/demo.py`.

## Interfaces & Contracts (things future code must preserve)

- User-facing entrypoint (contract; canonical docs live in `README.md`):
  - `src/demo.py` requires exactly one mode (`--sim | --hardware | --check`); `--hardware` requires `--ip`.
- Internal seam (repo-local, but preserved to keep code navigable):
  - `driver/ur5e_driver.py:UR5eDriver` provides the loop boundary: `get_state()` → controller step → `send_velocity()` → `step()`.
- Internal configuration (repo-local):
  - `config/wrench_calibration.yaml` consumed by wrench processing:
    - Required keys: `sensor_rotation` (9 floats), `gravity_force` (3), `force_bias` (3), `torque_bias` (3), `center_of_mass` (3)

## Repo Blueprint (key file map)

```text
src/
driver/
models/
config/
algorithms/
```

## Acceptance Signals (how we know it’s correct)

- Sim smoke:
  - RCM demo runs without crashing; visualization shows UR5e/probe; RCM error remains bounded.
  - Hybrid mode runs; contact/force loop remains stable enough for a short demo (no runaway).
- Hardware smoke:
  - Wrench calibration produces a valid `config/wrench_calibration.yaml` with required keys.
  - Hardware mode sends RTDE velocity commands and the robot responds smoothly.

## Decisions (durable tradeoffs)

- RCM enforced as a hard equality constraint in a convex QP on joint velocities.
- Digital Twin driver always owns a Drake plant even in hardware mode (Jacobian computation + viz).
- Hybrid force uses admittance (velocity command from force error) rather than a force-mode API.

## Assumptions (unverified / TBD)

- World frame used for control is effectively base frame for gravity compensation/Jacobians.
- Default gains/targets are demo-tuned and may not generalize across probes/tissues.

## Open Questions

- [ ] Should force/velocity safety limits be enforced in the QP constraints for hardware?
- [ ] Do we want a minimal automated sim test harness (even just “runs for N seconds”)?
