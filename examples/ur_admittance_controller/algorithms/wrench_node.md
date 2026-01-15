# Wrench Processing (LPF → gravity/bias comp → deadband → TF transforms)

## Purpose

- Convert raw NetFT readings into usable external wrench signals for control.
- Publish the same compensated wrench in three frames: sensor, probe/TCP, and base.

## Inputs (symbols, units, frames)

- ROS topics:
  - `/netft/raw_sensor` — raw wrench — `geometry_msgs/WrenchStamped` — frame `netft_link1`
- Calibration:
  - `config/wrench_calibration.yaml` with keys: `sensor_rotation`, `gravity_force`, `force_bias`, `torque_bias`, `center_of_mass`
- TF dependencies:
  - Static: `p42v_link1 ← netft_link1` (probe ← sensor)
  - Dynamic: `tool0 ← base_link` (tool ← base) for gravity mapping

## Outputs (symbols, units, frames)

- `/netft/proc_sensor` — compensated wrench in sensor frame `netft_link1`
- `/netft/proc_probe` — compensated wrench expressed in probe frame `p42v_link1`
- `/netft/proc_base` — compensated wrench expressed in base frame `base_link`

## Conventions (must not drift)

- Wrench vector ordering is `[fx, fy, fz, tx, ty, tz]`.
- Filtering happens on raw data before compensation.

## Core Logic / Equations (minimal)

Given `w_raw`:
1. LPF (IIR): `w_filt = α w_raw + (1-α) w_prev` (`α≈0.715` for `Δt≈0.002s`)
2. Sanitize tiny values (avoid floating noise)
3. Gravity + bias compensation (still in sensor frame):
   - `g_S = R_SE * R_TB * f_grav_b`
   - `τ_g_S = p_CoM_s × g_S`
   - `w_proc = w_sanitized - [g_S; τ_g_S] - [f_bias_s; t_bias_s]`
4. Deadband: zero small residual force/torque
5. Transform:
   - Sensor → Probe using a full dual-adjoint transform (`tf2_kdl::doTransform` on KDL::Wrench)
   - Probe → Base similarly

## Invariants / Guarantees

- If static TF `PROBE←SENSOR` is not available, the node must not publish (early return).
- If calibration YAML is missing/invalid, the node must fail fast.

## Implementation Mapping (code pointers)

- `include/wrench_node.hpp` — frame names, parameters, types
- `src/wrench_node.cpp` — the pipeline (`WrenchCallback`, `CompensateWrench`, YAML load)

## Defaults (tunable starting points)

- LPF:
  - Typical loop `Δt ≈ 0.002` s (500 Hz)
  - `f_c ≈ 200` Hz
  - `α = 1 - exp(-2π f_c Δt) ≈ 0.715`
- Deadband:
  - `DEADBAND_FORCE = 1.2` N
  - `DEADBAND_TORQUE = 0.02` N·m

## Acceptance Signals (how to validate)

- `ros2 topic echo /netft/raw_sensor --once` shows data.
- After running `wrench_node`:
  - `ros2 topic echo /netft/proc_sensor --once`
  - `ros2 topic echo /netft/proc_probe --once`
  - `ros2 topic echo /netft/proc_base --once`

## Known Pitfalls (avoid / gotchas)

- Wrong `frame_id` from NetFT driver (must match `netft_link1`) makes transforms silently wrong.
- Missing TF makes the node look “stuck” (it’s correctly gating until transforms exist).

## Open Questions

- [ ] Should `/netft/proc_base` be the primary control input to avoid repeating TF in downstream hybrid controllers?
