# orbiter-data

Bundled celestial-body ephemeris data required to compute planet/moon positions.
Layout mirrors Orbiter's `Src/Celbody/...` so the loader can treat this directory as
the data root.

**Product path**: `resolve_ephemeris_data()` / `ORBITX_EPHEMERIS_DATA` / `--ephemeris-data`
point here. Runtime and app do **not** fall back to a sibling `../orbiter` tree.

**Provenance**: copied from the [Orbiter Space Flight Simulator](https://github.com/orbitersim/orbiter)
source tree (MIT). Keep attribution when redistributing.

## Contents

- `Src/Celbody/Vsop87/Data/Vsop87*.dat` — VSOP87 planetary theory (Sun, Mercury..Neptune)
- `Src/Celbody/Moon/ELP82.dat` — ELP2000-82 lunar theory
- `Src/Celbody/Galsat/ephem_e15.dat` — Galilean moons (Io, Europa, Ganymede, Callisto)
- `Src/Celbody/Satsat/tass17.dat` — TASS1.7 Saturnian moons

## Not bundled

Gravity field models (`GravityModels/*.tab`, ~10 MB) are only used for N-body
propagation, not for rendering positions. If absent, the loader falls back to a
point-mass gravity model. To exercise full high-degree fields in local experiments,
point `--ephemeris-data` / `ORBITX_EPHEMERIS_DATA` at a tree that also contains
`GravityModels/` (e.g. a complete Orbiter install). Product defaults remain the
bundled `assets/orbiter-data` without that dependency.
