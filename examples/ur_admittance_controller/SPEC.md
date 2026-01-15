# SPEC.md (Blueprint) — `ur_admittance_controller` (example)

This `SPEC.md` is a **rebuildable blueprint** for the ROS2 package.
If the code vanished tomorrow, `README.md` + `CLAUDE.md` + `SPEC.md` + `algorithms/` should let an AI agent recreate most of it.

Scope: this file avoids run commands and session logs (see `README.md`/`CLAUDE.md` and `RESUME.md`).

---

## North Star

- Problem: Turn wrist-mounted force/torque sensing into stable, force-compliant motion on UR robots (hardware + sim).
- Success looks like:
  - A calibrated wrench pipeline publishes reliable compensated wrenches in probe and base frames.
  - Admittance control produces bounded joint velocity commands from external wrench input.
  - Hardware bringup publishes a complete TF tree with robot + sensor frames from a single robot description.
- Non-goals (for now): full hybrid force/motion scanning controller; contact classification; formal safety certification.

## System Invariants (must remain true)

### External Contracts (breaking these breaks interop)

- ROS topics + types:
  - `/netft/raw_sensor` — `geometry_msgs/WrenchStamped` (from `netft_utils`)
  - `/joint_states` — `sensor_msgs/JointState` (from the UR driver)
  - `/forward_velocity_controller/commands` — `std_msgs/Float64MultiArray` (to `ros2_control`)
- QoS must match upstream/downstream:
  - Wrench/velocity topics use reliable QoS (mismatched QoS can cause silent drops).
- Controller names (UR driver / ros2_control procedures):
  - `forward_velocity_controller`
  - `scaled_joint_trajectory_controller`
- TF frames owned by neighbors must match:
  - `base_link`, `tool0` (UR ecosystem)
  - `netft_link1` (NetFT driver `frame_id`; must match the URDF)

### Internal Conventions (breaking these breaks ourselves)

- Internal pipeline topics:
  - `/netft/proc_sensor`, `/netft/proc_probe`, `/netft/proc_base`
- Internal probe/TCP frame:
  - `p42v_link1` (defined in our URDF)
- Internal data format:
  - `config/wrench_calibration.yaml` contains keys: `sensor_rotation` (9), `gravity_force` (3), `force_bias` (3), `torque_bias` (3), `center_of_mass` (3)
- 6D ordering convention:
  - `[linear; angular]` everywhere (matches KDL; do not mix with Drake `[ω; v]`)

## Behavior (what it must do)

- Core scenarios:
  - Hardware bringup publishes TF and UR driver runs.
  - Wrench calibration writes a valid YAML with required keys.
  - Wrench processing publishes compensated topics in probe and base frames.
  - Admittance publishes bounded velocity commands from the compensated wrench input.
- Failure modes:
  - Missing TF chain → wrench/admittance blocks and logs, not publish garbage.
  - Missing calibration YAML → wrench_node fails fast (don’t run uncalibrated).
  - Switching controllers with active publishers → documented as dangerous (avoid jumps).

## Architecture (what exists and why)

- Major components/modules:
  - `src/wrench_node.cpp` (+ `include/wrench_node.hpp`) — filter + sanitize + gravity/bias compensate; publish wrenches in sensor/probe/base frames.
  - `src/wrench_calibration_node.cpp` + `src/wrench_calibration_algorithm.cpp` — collect static samples across poses; estimate calibration parameters; write YAML.
  - `src/admittance_node.cpp` + `src/admittance_computations.cpp` — admittance ODE + world limiter + resolved-rates IK; publish joint velocities.
  - `src/init_robot.cpp` (+ `include/init_robot.hpp`) — move to equilibrium; write `equilibrium.yaml` used by admittance.
  - `urdf/ur_hardware_with_ft_sensor.urdf.xacro` (+ sensor xacros) — unified robot + sensor description for TF correctness.
  - `launch/ur_hardware_bringup.launch.py` — professional hardware bringup wrapper around `ur_robot_driver`.
- TF publishing pattern:
  - Hardware bringup uses a unified URDF so one robot_state_publisher publishes the full chain `base_link → tool0 → netft_* → p42v_*`.
- Data/control flow:
  - `/netft/raw_sensor` → `wrench_node` → `/netft/proc_probe` → `admittance_node` → `/forward_velocity_controller/commands`

## Algorithm Index (math contracts, optional)

- [`algorithms/wrench_calibration.md`](algorithms/wrench_calibration.md) — LROM + Procrustes + LS calibration — read before touching calibration code or YAML schema.
- [`algorithms/wrench_node.md`](algorithms/wrench_node.md) — filtering + gravity/bias comp + TF transforms — read before touching wrench processing topics/frames.
- [`algorithms/admittance_node.md`](algorithms/admittance_node.md) — 6D admittance + limiter + IK mapping — read before tuning gains or changing 6D ordering.

## Interfaces & Contracts (things future code must preserve)

- Entrypoints (ROS2 executables):
  - `admittance_node`, `wrench_node`, `wrench_calibration_node`, `init_robot` (see `CMakeLists.txt`)
- Configuration:
  - `config/admittance_config.yaml` → generated parameter library (`ur_admittance_controller_parameters`)
  - `config/equilibrium.yaml` → equilibrium pose loaded by `admittance_node`
    - Structure:
      - `admittance_node.ros__parameters.equilibrium.position = [x, y, z]` (m)
      - `admittance_node.ros__parameters.equilibrium.orientation = [x, y, z, w]` (quat)
  - `config/wrench_calibration.yaml` → calibration consumed by `wrench_node`
- TF dependencies:
  - `wrench_node` requires `PROBE_FRAME ← SENSOR_FRAME` (static) and `TOOL_FRAME ← BASE_FRAME` (dynamic) to be available.

## Repo Blueprint (key file map)

```text
package.xml
CMakeLists.txt
include/
src/
config/
urdf/
launch/
algorithms/
```

## Acceptance Signals (how we know it’s correct)

- Build:
  - Package builds successfully.
- TF chain:
  - `tool0 → p42v_link1` transform resolves (no hang).
- Wrench pipeline:
  - Raw sensor topic provides data.
  - Processed probe/base topics publish compensated data after `wrench_node` runs.
- Control:
  - Velocity command topic publishes messages while `admittance_node` runs.

## Decisions (durable tradeoffs)

- Unified URDF pattern: one robot description includes robot + sensor, so TF is published from one place.
- Wrench math is explicit and validated by schema checks; missing calibration is a hard error.
- 6D ordering matches KDL (`[linear; angular]`) end-to-end to avoid hidden reorder bugs.

## Assumptions (unverified / TBD)

- NetFT driver publishes `/netft/raw_sensor` with `frame_id=netft_link1`.
- Controller switching procedure is followed to avoid velocity “jumps”.

## Open Questions

- [ ] Do we need a minimal simulation test that exercises wrench_node + admittance_node end-to-end?
- [ ] Where should safety limits live: in admittance_node limiter, in ros2_control, or both?
