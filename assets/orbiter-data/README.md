# orbiter-data

Bundled celestial-body ephemeris data required to compute planet/moon positions,
copied from an Orbiter Space Flight Simulator source tree (`Src/Celbody/...`).

The directory layout mirrors what the ephemeris loader expects, so
`resolve_orbiter_src()` can point here directly (no `ORBITER_SRC` env var needed).

## Contents

- `Src/Celbody/Vsop87/Data/Vsop87*.dat` — VSOP87 planetary theory (Sun, Mercury..Neptune)
- `Src/Celbody/Moon/ELP82.dat` — ELP2000-82 lunar theory
- `Src/Celbody/Galsat/ephem_e15.dat` — Galilean moons (Io, Europa, Ganymede, Callisto)
- `Src/Celbody/Satsat/tass17.dat` — TASS1.7 Saturnian moons

## Not bundled

Gravity field models (`GravityModels/*.tab`, ~10 MB) are only used for N-body
propagation, not for rendering positions. If absent, the loader falls back to a
point-mass gravity model. To use the full high-degree fields, set `ORBITER_SRC`
to a complete Orbiter install.
