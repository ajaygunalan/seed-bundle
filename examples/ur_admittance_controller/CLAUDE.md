# CLAUDE.md (example docs set)

## Docs Map (read in this order)

- @README.md — human setup/run overview
- @SPEC.md — rebuildable blueprint (what must remain true)
- @algorithms/ — math contracts (index in @SPEC.md → “Algorithm Index”)
- @RESUME.md (if present) — session continuity (Reheat)

## Project Overview

ROS2 package for force-compliant motion on UR robots:
- `wrench_node` turns raw NetFT readings into gravity/bias compensated wrenches in sensor/probe/base frames.
- `admittance_node` applies a 6D admittance law and publishes joint velocity commands.
- `wrench_calibration_node` estimates gravity/bias parameters used by `wrench_node`.
- `init_robot` moves to equilibrium and writes the equilibrium pose config used by `admittance_node`.

Key contracts (frames/topics/6D ordering): see @SPEC.md → “System Invariants”.

## How to run (quick)

- Build: `colcon build --packages-select ur_admittance_controller`
- Hardware TF + UR driver: `ros2 launch ur_admittance_controller ur_hardware_bringup.launch.py ...`
- NetFT driver: `ros2 run netft_utils netft_node --address 169.254.120.10 --frame_id netft_link1`
- Wrench pipeline: `ros2 run ur_admittance_controller wrench_node`
- Admittance: `ros2 run ur_admittance_controller admittance_node`
