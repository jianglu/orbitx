# planets

Equirectangular surface maps for planets and moons, used to texture the 3D
spheres when the camera is close enough (sphere LOD, not billboard).

## Sources

- **Major planets, Sun-adjacent bodies, Moon** (`Mercury/Venus/Earth/Mars/Moon/
  Jupiter/Saturn/Uranus/Neptune.jpg`, 2048x1024): from **Solar System Scope**
  (https://www.solarsystemscope.com/textures), licensed **CC BY 4.0**.
  Attribution: Solar System Scope (INOVE), CC BY 4.0.

- **Minor moons** (`Io/Europa/Ganymede/Callisto/Titan/Triton/Iapetus/Phobos/
  Deimos.png`): low-resolution far-view maps converted from an Orbiter Space
  Flight Simulator source tree (`Textures/<Body>M.bmp`).

Each file is named after the body as it appears in the system config
(`Earth.jpg`, `Io.png`, ...). Bodies without a bundled map (the Sun) fall back
to their flat base color. The loader accepts `.jpg` and `.png`.
