# Generate Seed Bundle: rcm_qp_drake

## Task

Generate the seed bundle docs for `/home/ajay/rcm_qp_drake` using the templates in `/home/ajay/seed-bundle/files/`.

## Steps

1. Read the templates:
   - `files/SPEC.md` — blueprint structure
   - `files/ALGORITHM.md` — algorithm doc structure (with Obsidian callouts + pseudo-code)

2. Explore the target repo `/home/ajay/rcm_qp_drake`:
   - Entrypoints, modules, config files
   - Controller math in `src/controller.py`
   - Driver layer in `driver/`
   - Any existing docs

3. Generate these files in `/home/ajay/seed-bundle/examples/rcm_qp_drake/`:
   - `README.md` — human install/usage
   - `CLAUDE.md` — agent runbook
   - `SPEC.md` — rebuildable blueprint
   - `algorithms/rcm_qp.md` — RCM QP math + implementation mapping
   - `algorithms/rcm_and_force.md` — hybrid force/motion math + implementation mapping

## Algorithm doc requirements

- Full math derivation as primary narrative
- Use Obsidian callouts: `> [!implementation]`, `> [!note]`, `> [!warning]`
- Symbol-to-code mapping in tables and inline
- Pseudo-code only (no C++/Python), language-agnostic
- Link to theory concepts with `[[wikilinks]]` where relevant

## Key algorithms to document

1. **RCM QP**: Remote Center of Motion constraint as QP on joint velocities
2. **Hybrid force/motion**: Admittance-style force control + barrier logic
