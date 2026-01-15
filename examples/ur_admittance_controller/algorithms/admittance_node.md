# Admittance Control (6D ODE + limiter + resolved-rates IK)

## Purpose

- Convert an external wrench at the probe into compliant motion using a 6D admittance law.
- Publish joint velocity commands for the UR forward velocity controller.

## Inputs (symbols, units, frames)

- Wrench input:
  - `/netft/proc_probe` — compensated wrench — `[N; N·m]` — probe frame `P` (`p42v_link1`)
- Robot state:
  - `/joint_states` — measured joints
- Desired pose (optional):
  - `/admittance_node/desired_pose` — PoseStamped (planner input)
- Configuration:
  - `config/equilibrium.yaml` — equilibrium pose (loaded at startup)
  - `config/admittance_config.yaml` — gains/limits (generate_parameter_library)

## Outputs (symbols, units, frames)

- `/forward_velocity_controller/commands` — `std_msgs/Float64MultiArray` joint velocities (rad/s)

## Conventions (must not drift)

- 6D vector ordering is `[linear; angular]` everywhere (matches KDL::Twist).
- Wrench is assumed already expressed in probe frame (no extra TF in admittance loop).

## Core Logic / Equations (minimal)

1. Admittance ODE in probe axes:
   - `M ẍ + D ẋ + K x = F_ext` (all 6D)
2. Integrate (semi-implicit Euler) to update `x` and `ẋ`.
3. Map offset and rate to world using adjoints built from the **desired** pose.
4. Compose commanded pose `X_cmd` and compute pose error vs FK `X_meas`.
5. World twist command:
   - `V_cmd = V_des + ẋ_world + Kp ⊙ error`
6. Apply a single world limiter:
   - acceleration-rate gating (memory)
   - workspace wall gating
   - velocity magnitude caps
7. Resolved-rates IK (damped) to joint velocities; publish.

## Invariants / Guarantees

- If equilibrium pose config is missing/malformed, the node must fail fast.
- Limiter is an engineering safety layer and must remain in world space (don’t scatter limits across frames).

## Implementation Mapping (code pointers)

- Control loop / ROS interfaces:
  - `src/admittance_node.cpp`
- Math helpers (ordering + adjoints + pose error + limits + IK wrappers):
  - `src/admittance_computations.cpp`
  - `include/admittance_computations.hpp` (note the ordering contract)
- Equilibrium pose generation:
  - `src/init_robot.cpp` → writes `equilibrium.yaml`

## Defaults (tunable starting points)

- Control rate:
  - Typical loop `Δt ≈ 0.002` s (500 Hz) to match the wrench pipeline timing.
- IK:
  - Resolved-rates IK uses KDL’s damped least squares (WDLS); damping is a tunable stability knob.

## Acceptance Signals (how to validate)

- With wrench pipeline running:
  - `ros2 topic echo /forward_velocity_controller/commands --once` shows messages.
- If the wrench is near zero, commanded velocities should be near zero (after deadband and limiter).

## Known Pitfalls (avoid / gotchas)

- Mixing 6D ordering with Drake-style `[ω; v]` breaks everything silently; keep `[linear; angular]`.
- Forgetting to switch controllers (or switching with active publishers) can cause a “jump”.

## Open Questions

- [ ] Should the desired pose topic be required (closed-loop pose tracking), or remain optional (equilibrium-only)?
