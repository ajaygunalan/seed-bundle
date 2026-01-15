# RCM Force Control for Surgical Robotics (example docs set)

QP-based Remote Center of Motion (RCM) controller with hybrid force/motion control for UR5e.

This folder is a **Seed Bundle** example for the real repo at `/home/ajay/rcm_qp_drake`.

## What It Does

A surgical probe pivots through a **trocar** (fixed entry point) while:
1. **RCM constraint** keeps the probe passing through the trocar
2. **Force control** regulates contact force with tissue
3. **Same code** runs in simulation and on real hardware (Digital Twin pattern)

## Quick Start

```bash
# 1. Activate environment
source scripts/activate.sh

# 2. Run simulation
python src/demo.py --sim
python src/demo.py --sim --mode rcm_and_force --tissue flat
```

## Documentation (where to look)

| File | Why you’d read it |
|------|-------------------|
| [`README.md`](README.md) | Human-facing install/usage (this file) |
| [`CLAUDE.md`](CLAUDE.md) | Agent runbook + repo conventions |
| [`SPEC.md`](SPEC.md) | Rebuildable blueprint (architecture/contracts/acceptance) |
| [`algorithms/`](algorithms/) | Math contracts (per-algorithm); see `SPEC.md` → “Algorithm Index” |
| [`RESUME.md`](RESUME.md) (optional, Reheat) | Session continuity (working/broken, failures, repro, next step) |

## Dependencies

- Drake (simulation + optimization)
- ur_rtde (UR robot control; hardware)
- NetFT (ATI force sensor; hardware)
