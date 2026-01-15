# ur_admittance_controller (example docs set)

6-DOF force-compliant motion control for Universal Robots with gravity compensation (ATI NetFT).

This folder is a **Seed Bundle** example for the real package at `/home/ajay/ros2_ws/src/ur_admittance_controller`.

## Documentation (where to look)

| File | Why you’d read it |
|------|-------------------|
| [`README.md`](README.md) | Human-facing setup + run overview (this file) |
| [`CLAUDE.md`](CLAUDE.md) | Agent runbook + “what to read when” |
| [`SPEC.md`](SPEC.md) | Rebuildable blueprint (architecture/contracts/acceptance) |
| [`algorithms/`](algorithms/) | Math contracts (per-algorithm); see `SPEC.md` → “Algorithm Index” |
| [`RESUME.md`](RESUME.md) (optional, Reheat) | Session continuity (working/broken, failures, repro, next step) |

## What It Does (pipeline)

```
[NetFT] → /netft/raw_sensor → [wrench_node] → /netft/proc_probe → [admittance_node] → /forward_velocity_controller/commands
                      ↘︎ publishes /netft/proc_sensor and /netft/proc_base for debugging / downstream use
```

## Minimal Run Overview

- Build:
  - `cd ~/ros2_ws && colcon build --packages-select ur_admittance_controller && source install/setup.bash`
- Hardware bringup (TF tree + UR driver):
  - `ros2 launch ur_admittance_controller ur_hardware_bringup.launch.py robot_ip:=169.254.120.1 kinematics_params_file:=$HOME/ur5e_calibration.yaml`
  - Start NetFT driver separately: `ros2 run netft_utils netft_node --address 169.254.120.10 --frame_id netft_link1`
- One-time-ish:
  - Set equilibrium pose: `ros2 run ur_admittance_controller init_robot`
  - Calibrate wrench: `ros2 run ur_admittance_controller wrench_calibration_node`
- Daily loop:
  - Run wrench processing: `ros2 run ur_admittance_controller wrench_node`
  - Switch to velocity controller (only after verifying 0 publishers)
  - Run admittance: `ros2 run ur_admittance_controller admittance_node`

## Dependencies (workspace-level)

- `ur_robot_driver` (hardware) + calibration from `ur_calibration`
- `netft_utils` (hardware sensor driver)
- `ur_simulation_gz` (simulation bringup)
