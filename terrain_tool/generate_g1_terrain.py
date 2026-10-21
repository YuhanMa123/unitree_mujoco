"""Rebuild G1's three terrain lanes using only the Python standard library."""

import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET


OUTPUT = Path(__file__).resolve().parents[1] / "unitree_robots/g1"


def height_samples():
    """A repeatable 5 x 2 m heightfield, tapered to zero at all four edges."""
    rng = random.Random(42)
    rows, cols = 33, 81
    # Bilinear random relief plus smaller ripples; no isolated sharp spikes.
    coarse = [[rng.random() for _ in range(21)] for _ in range(9)]
    pixels = []
    for j in range(rows):
        y = j / (rows - 1)
        gy = y * 8
        iy = min(int(gy), 7)
        ty = gy - iy
        for i in range(cols):
            x = i / (cols - 1)
            gx = x * 20
            ix = min(int(gx), 19)
            tx = gx - ix
            a = coarse[iy][ix] * (1 - tx) + coarse[iy][ix + 1] * tx
            b = coarse[iy + 1][ix] * (1 - tx) + coarse[iy + 1][ix + 1] * tx
            relief = 0.15 + 0.75 * (a * (1 - ty) + b * ty)
            relief += (
                0.10 * math.sin(2 * math.pi * 16 * x) * math.sin(2 * math.pi * 7 * y)
            )
            taper = min(1, x * 10, (1 - x) * 10, y * 8, (1 - y) * 8)
            pixels.append(max(0, round(255 * relief * taper)))
    return rows, cols, pixels


def main():
    root = ET.Element("mujocoinclude")
    asset = ET.SubElement(root, "asset")
    rows, cols, pixels = height_samples()
    # Inline elevations keep the scene self-contained and avoid asset search paths.
    ET.SubElement(
        asset,
        "hfield",
        name="g1_rough_height",
        nrow=str(rows),
        ncol=str(cols),
        elevation=" ".join(f"{p / max(pixels):.5f}" for p in pixels),
        size="2.5 1 0.06 0.05",
    )
    # Convex triangular prism: 1.8 m run, 0.28 m rise, 2 m width.
    ET.SubElement(
        asset,
        "mesh",
        name="g1_ramp_wedge",
        vertex="0 -1 -0.05  0 1 -0.05  1.8 -1 -0.05  1.8 1 -0.05  1.8 -1 0.28  1.8 1 0.28  0 -1 0  0 1 0",
        face="0 3 2 0 1 3  6 5 7 6 4 5  0 4 6 0 2 4  1 5 3 1 7 5  2 5 4 2 3 5  0 7 1 0 6 7",
    )
    world = ET.SubElement(root, "worldbody")

    def geom(name, **attrs):
        return ET.SubElement(
            world, "geom", name=name, friction="1 0.005 0.0001", **attrs
        )

    def box(name, x, y, length, height, color):
        geom(
            name,
            type="box",
            pos=f"{x:g} {y:g} {(height - 0.05) / 2:g}",
            size=f"{length / 2:g} 1 {(height + 0.05) / 2:g}",
            rgba=color,
        )

    # Center lane: rise, broad crest, descend. Every box extends below the floor.
    for i in range(4):
        box(
            f"terrain_step_up_{i + 1}",
            3 + (i + 0.5) * 0.45,
            0,
            0.45,
            (i + 1) * 0.12,
            "0.65 0.43 0.22 1" if i % 2 == 0 else "0.75 0.55 0.30 1",
        )
    box("terrain_stairs_top", 5.25, 0, 0.9, 0.48, "0.78 0.60 0.35 1")
    for i in range(3):
        box(
            f"terrain_step_down_{i + 1}",
            5.7 + (i + 0.5) * 0.45,
            0,
            0.45,
            (3 - i) * 0.12,
            "0.65 0.43 0.22 1" if i % 2 == 0 else "0.75 0.55 0.30 1",
        )
    # Left lane: continuous uphill / plateau / downhill, flush with ground at both ends.
    geom(
        "terrain_ramp_up",
        type="mesh",
        mesh="g1_ramp_wedge",
        pos="3 3 0",
        rgba="0.35 0.56 0.40 1",
    )
    box("terrain_ramp_top", 5.1, 3, 0.6, 0.28, "0.42 0.64 0.47 1")
    geom(
        "terrain_ramp_down",
        type="mesh",
        mesh="g1_ramp_wedge",
        pos="7.2 3 0",
        quat="0 0 0 1",
        rgba="0.35 0.56 0.40 1",
    )
    # Right lane: continuous collision heightfield (not a decorative texture).
    geom(
        "terrain_rough",
        type="hfield",
        hfield="g1_rough_height",
        pos="5.5 -3 0",
        rgba="0.50 0.48 0.44 1",
    )
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(OUTPUT / "terrain_29dof.xml", encoding="unicode")
    print(f"Generated terrain_29dof.xml in {OUTPUT}")


if __name__ == "__main__":
    main()
