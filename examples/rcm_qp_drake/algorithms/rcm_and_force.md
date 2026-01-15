# RCM + Hybrid Force/Motion

## Purpose

- Add axial force regulation (admittance) on top of the RCM QP, while keeping tangential scanning motion.
- Add a “force barrier” to prevent pushing further into large perpendicular contact.

## Inputs (symbols, units, frames)

- From RCM QP (see `algorithms/rcm_qp.md`):
  - `q`, `p_WC`, kinematics, `V_pivot`
- Force sensing:
  - `wrench_W = [f_W; τ_W]` — measured wrench — `[N; N·m]` — world `W`
  - `n_hat` — shaft direction (unit) — `W`
- Force control parameters (typical):
  - `f_target` (N), `k_a` (m/s per N), `v_max` (m/s), `f_perp_max` (N), hysteresis (N), LPF `α`

## Outputs (symbols, units, frames)

- `v_opt ∈ R^6` — joint velocity command — rad/s
- `v_ins` — insertion velocity scalar along `n_hat` — m/s
- Optional `u_f` — barrier direction (unit, perpendicular) — `W`

## Conventions (must not drift)

- Axial compression is **positive**:
  - `f_axial = -(f_W ⋅ n_hat)`
- The hybrid controller modifies the desired TCP linear velocity, not the RCM constraint.

## Core Logic / Equations (minimal)

1. Decompose force:
   - `f_perp = f_W - (f_W ⋅ n_hat) n_hat`
2. Low-pass filter `f_axial` and `f_perp`.
3. Axial admittance (P-control with saturation):
   - `v_ins = clip( k_a (f_target - f_axial_filt), [-v_max, v_max] )`
4. Modify desired twist:
   - `V_des = V_pivot`
   - `V_des.linear += v_ins n_hat`
5. Force barrier (perpendicular):
   - If `||f_perp_filt|| > f_perp_max`, freeze `u_f = f_perp_filt / ||f_perp_filt||` until it drops below `(f_perp_max - hysteresis)`
   - Add QP inequality (forbid motion further into contact):
     - `-u_fᵀ J_T_lin v ≤ 0`
6. Solve the same RCM QP (with optional slack) to get `v_opt`.

## Invariants / Guarantees

- If barrier is active, its direction is held constant (prevents “direction chatter”).
- Force mode must be explicit: never claim force regulation if the wrench pipeline is uncalibrated/unavailable.

## Implementation Mapping (code pointers)

- Force logic:
  - `src/controller.py` — `force_decompose`, `hybrid_force_step`, `ForceControlState`, `ForceControlParams`
- Barrier integration:
  - `src/controller.py` — `solve_rcm_qp(... force_barrier_dir=...)`
- Call sites:
  - `src/demo.py` — hardware loop uses `UR5eDriver.get_tool_wrench_W()`
  - `src/demo.py` — sim loop uses Drake contact + `WrenchProcessor` then the same `hybrid_force_step()`

## Defaults (tunable starting points)

- Force control:
  - `f_target = 5.0` N
  - `k_a = 0.005` (m/s)/N — axial admittance gain
  - `v_max = 0.01` m/s — insertion speed cap
  - `f_perp_max = 5.0` N — barrier threshold
  - `hysteresis = 0.5` N — barrier release hysteresis
  - `α = 0.3` — unitless — force LPF coefficient (if used)
- Wrench preprocessing (when using the built-in filter/deadband):
  - `ALPHA = 0.715` — unitless — wrench LPF coefficient
  - `DEADBAND_FORCE = 1.2` N
  - `DEADBAND_TORQUE = 0.02` N·m

## Acceptance Signals (how to validate)

- Sim:
  - `python src/demo.py --sim --mode rcm_and_force --tissue flat` runs; insertion responds to contact force directionally.
- Hardware:
  - `python driver/wrench_calibration_node.py --hardware --ip <ROBOT_IP>` produces a calibration YAML with required keys.
  - `python src/demo.py --hardware --ip <ROBOT_IP> --mode rcm_and_force` runs with NetFT.

## Known Pitfalls (avoid / gotchas)

- Wrong sign on axial force will cause the controller to “dig in” instead of retract.
- If `wrench_W` is not truly in world frame, `f_axial` and barrier direction are wrong (frame bug looks like “tuning bug”).
- Stale calibration YAML schema (old keys) makes the wrench pipeline invalid; treat as malformed and re-run calibration.

## Open Questions

- [ ] Should the barrier be implemented as a soft cost instead of a hard inequality?
