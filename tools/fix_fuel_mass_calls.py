# -*- coding: utf-8 -*-
"""One-shot: v.fuel_mass -> v.fuel_mass(); v.fuel_mass = 0.0 -> v.drain_fuel()."""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(r"e:\Project\SimRocket\orbitx")

SKIP_PARTS = {
    ("orbitx-app", "vessel.rs"),
    ("orbitx-app", "flight_calc.rs"),
    ("orbitx-app", "app.rs"),
    ("orbitx-gfx-hud", "hud.rs"),
    ("orbitx-gfx-hud", "flight_state.rs"),
    ("orbitx-launch", "main.rs"),
}

ASSIGN = re.compile(r"([\w.\[\]]+)\.fuel_mass\s*=\s*0\.0\s*;")
READ = re.compile(r"([\w.\[\]]+)\.fuel_mass(?!\s*[\(:])")


def should_skip(path: Path) -> bool:
    parts = path.parts
    for crate, name in SKIP_PARTS:
        if crate in parts and path.name == name:
            return True
    return False


def fix(text: str) -> str:
    text = ASSIGN.sub(r"\1.drain_fuel();", text)

    def repl(m: re.Match) -> str:
        base = m.group(1)
        # Don't touch struct field init like `something` without dot chain of interest
        # Already matched X.fuel_mass
        return f"{base}.fuel_mass()"

    text = READ.sub(repl, text)
    text = text.replace("fuel_mass()()", "fuel_mass()")
    text = text.replace("drain_fuel();()", "drain_fuel();")
    return text


def main() -> None:
    for path in ROOT.rglob("*.rs"):
        if "target" in path.parts or should_skip(path):
            continue
        old = path.read_text(encoding="utf-8")
        new = fix(old)
        if new != old:
            path.write_text(new, encoding="utf-8", newline="\n")
            print("patched", path.relative_to(ROOT))


if __name__ == "__main__":
    main()
