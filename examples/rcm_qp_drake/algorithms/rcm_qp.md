# RCM QP (RCM-only)

## Purpose

- Enforce the trocar Remote Center of Motion (RCM) constraint while tracking a desired pivoting motion.
- Output joint velocities for a 6-DOF arm.

## Inputs (symbols, units, frames)

- Joint state:
  - `q ∈ R^6` — joint positions — rad
- Geometry:
  - `p_WC` — trocar position — m — world frame `W`
  - `X_WF = (R_WF, p_WF)` — shaft/flange pose — `W`
  - `n_hat = R_WF[:,2]` — shaft direction (unit) — `W`
- Desired motion:
  - `V_des = [ω_des; v_des]` — desired TCP twist — `[rad/s; m/s]` — expressed in `W`
- Kinematics:
  - `J_T(q)` — TCP spatial Jacobian — 6×6 — Drake ordering `[ω; v]`
  - `J_P(q)` — Jacobian of projection point on shaft — 3×6 (linear velocity)

## Outputs (symbols, units, frames)

- `v_opt ∈ R^6` — joint velocities — rad/s

## Conventions (must not drift)

- Drake spatial Jacobian ordering is `[ω; v]`.
- RCM constraint is 2D (lateral plane) — use a 2×3 basis `B_perp`, not a 3D projection matrix.

## Core Logic / Equations (minimal)

- Projection point on shaft closest to trocar:
  - `r = p_WC - p_WF`
  - `λ = r ⋅ n_hat`
  - `p_WP = p_WF + λ n_hat`
  - `e = p_WC - p_WP` (lateral error)
- Build a 2D orthonormal basis `B_perp` spanning the plane perpendicular to `n_hat`.
- Equality constraint (RCM correction in lateral plane):
  - `B_perp J_P v = K_rcm B_perp e`
  - Optional slack `s ∈ R^2`:
    - `[B_perp J_P  -I] [v; s] = K_rcm B_perp e`, penalize `||s||^2`
- Cost (track TCP linear velocity + roll):
  - Let `J_v = J_T[3:6,:]` and `J_ω = J_T[0:3,:]`
  - Linear tracking: `|| J_v v - v_des ||^2_{W_lin}`
  - Roll tracking: `ω_roll = n_hatᵀ J_ω v`, `ω_roll_des = n_hatᵀ ω_des`
  - Regularization: `ε ||v||^2`
- Bounds: `|v_i| ≤ v_max`

## Invariants / Guarantees

- RCM equality is always present (with optional slack, but never “removed silently”).
- `B_perp` must be well-defined for any `n_hat` (choose a non-parallel auxiliary axis).

## Implementation Mapping (code pointers)

- Primary implementation:
  - `src/controller.py` — `compute_rcm_error`, `compute_jacobians`, `solve_rcm_qp`, `rcm_step`
- Call sites:
  - `src/demo.py` — RCM mode loop (`run_rcm` / `_rcm_motion_loop`)

## Defaults (tunable starting points)

- `K_rcm = 20.0` — 1/s — lateral error correction gain
- `ε = 0.01` — unitless — joint-velocity regularization weight
- `v_max = 1.5` — rad/s — per-joint velocity bound
- `w_roll = 0.1` — unitless — roll tracking weight
- `w_slack = 1000.0` — unitless — slack penalty weight (if slack enabled)
- Typical control loop: `dt = 0.01` s (100 Hz)
- Model frames referenced by the controller:
  - `shaft_frame = "flange"`
  - `tcp_frame = "tcp"`

## Acceptance Signals (how to validate)

- `python src/demo.py --sim` runs and pivots about the trocar marker in Meshcat.
- `solve_rcm_qp(...)` returns non-zero velocities (not all zeros) for non-trivial `V_des`.

## Known Pitfalls (avoid / gotchas)

- Mixing twist conventions: this repo uses `[ω; v]` (Drake), not `[v; ω]` (KDL).
- Using the wrong shaft frame name breaks `n_hat` (must be the probe/shaft Z axis frame).

## Open Questions

- [ ] Should joint velocity limits be replaced with joint acceleration/position limits for hardware safety?
