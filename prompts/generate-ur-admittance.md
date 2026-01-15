# Generate Seed Bundle: ur_admittance_controller

## Task

Generate the seed bundle docs for `/home/ajay/ros2_ws/src/ur_admittance_controller` using the templates in `/home/ajay/seed-bundle/files/`.

## Steps

1. Read the templates:
   - `files/SPEC.md` — blueprint structure
   - `files/ALGORITHM.md` — algorithm doc structure (with Obsidian callouts + pseudo-code)

2. Explore the target repo `/home/ajay/ros2_ws/src/ur_admittance_controller`:
   - ROS2 nodes in `src/`
   - Headers in `include/`
   - Launch files, URDF, config
   - Any existing docs

3. Generate these files in `/home/ajay/seed-bundle/examples/ur_admittance_controller/`:
   - `README.md` — human install/usage
   - `CLAUDE.md` — agent runbook
   - `SPEC.md` — rebuildable blueprint
   - `algorithms/wrench_calibration.md` — LROM + Procrustes + LS calibration
   - `algorithms/wrench_node.md` — filtering + gravity/bias compensation
   - `algorithms/admittance_node.md` — 6D admittance + limiter + IK

## Algorithm doc requirements

- Full math derivation as primary narrative
- Use Obsidian callouts: `> [!implementation]`, `> [!note]`, `> [!warning]`
- Symbol-to-code mapping in tables and inline
- Pseudo-code only (no C++/Python), language-agnostic
- Link to theory concepts with `[[wikilinks]]` where relevant

## Key algorithms to document

1. **Wrench calibration**: LROM for gravity estimation, Procrustes for rotation/bias, LS for torque params
2. **Wrench node**: Filtering, gravity compensation, frame transforms
3. **Admittance node**: 6D admittance ODE, velocity limiting, resolved-rates IK
