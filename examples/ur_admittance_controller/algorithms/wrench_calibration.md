# Wrench Calibration (LROM + Procrustes + LS)

## Purpose

- Estimate gravity/bias calibration parameters for a wrist-mounted F/T sensor so raw readings can be turned into contact wrench.
- Output a YAML that `wrench_node` can consume deterministically.

## Inputs (symbols, units, frames)

- Measurements:
  - `w_raw = [f; τ]` — raw wrench from `/netft/raw_sensor` — `[N; N·m]` — sensor frame `S`
  - `R_TB` — rotation `tool0 ← base_link` from TF (`lookupTransform("tool0","base_link")`)
- Procedure constants:
  - `NUM_POSES=32`, `SAMPLES_PER_POSE=10`, `SAMPLE_DELAY=0.1s`

## Outputs (symbols, units, frames)

- `config/wrench_calibration.yaml` with (required by wrench_node):
  - `sensor_rotation` (`R_SE`, 9 floats): `{S} ← {E}` rotation
  - `gravity_force` (3): gravity * force vector in base `{B}` (includes `mg`)
  - `force_bias` (3): sensor force bias in `{S}`
  - `torque_bias` (3): sensor torque bias in `{S}`
  - `center_of_mass` (3): CoM position in `{S}`
- Optional extras (stored, not required by wrench_node):
  - `tool_mass`, `installation_roll_*`, `installation_pitch_*`

## Conventions (must not drift)

- Samples are collected while the robot is static at each pose.
- The algorithm assumes gravity direction ambiguity is resolved by picking the solution with negative z in typical base coordinates.

## Core Logic / Equations (minimal)

Pipeline (implemented as pure functions):
1. LROM: estimate gravity force `F_b` in base from multiple orientations.
2. Procrustes (SVD): estimate `R_SE` and force bias `F0`.
3. Least squares: estimate CoM `p_CoM` and torque bias `T0`.
4. Decompose `F_b` magnitude into tool mass and installation angles.

## Invariants / Guarantees

- Output YAML schema must match what `wrench_node` loads (key names + sizes).

## Implementation Mapping (code pointers)

- Orchestration + I/O:
  - `src/wrench_calibration_node.cpp`
- Pure algorithm steps:
  - `src/wrench_calibration_algorithm.cpp`
    - `estimateGravitationalForceInBaseFrame`
    - `estimateSensorRotationAndForceBias`
    - `estimateCOMAndTorqueBias`
    - `decomposeGravityVector`
- Types/constants:
  - `include/wrench_calibration_node.hpp`

## Acceptance Signals (how to validate)

- `ros2 run ur_admittance_controller wrench_calibration_node` creates/overwrites `config/wrench_calibration.yaml`.
- File contains required keys with correct sizes (rotation has 9 elements, vectors have 3).

## Known Pitfalls (avoid / gotchas)

- If TF `tool0 ← base_link` is missing or wrong, calibration “works” but produces nonsense (downstream looks like a control instability).
- Calibration while moving (or while in contact) corrupts the estimation.

## Open Questions

- [ ] Should calibration write into the package share directory instead of the workspace `src/.../config/` path?

