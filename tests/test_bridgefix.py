"""Synthetic test: a rectangular ring bridge printed with all lines along Y
(like bridge_angle=90 on a gasket-groove ceiling). The left/right sides then
have long lengthwise lines; bridgefix must turn them across the groove."""
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bridgefix"))
import bridgefix as bf  # noqa: E402

W, SP, EPM = 0.42, 0.387, 0.0252


def ring_gcode():
    # outer 60x40, groove width 3 -> inner 54x34
    ox0, oy0, ox1, oy1 = 0.0, 0.0, 60.0, 40.0
    g = 3.0
    lines = ["; retraction_length = 0.8\n", "; z_hop = 0.4\n", "M83\n", "G90\n",
             "G1 X0 Y0 Z1.16 F30000\n", "; FEATURE: Inner wall\n", "G1 X1 Y0 E.02\n",
             "; FEATURE: Bridge\n", "; LINE_WIDTH: 0.42\n", "M106 S255\n"]
    x = ox0 + W / 2
    up = True
    while x < ox1:
        if x < ox0 + g or x > ox1 - g:
            segs = [(oy0, oy1)]
        else:
            segs = [(oy0, oy0 + g), (oy1 - g, oy1)]
        for a, b in segs:
            y0, y1 = (a, b) if up else (b, a)
            lines.append(f"G1 E-.8 F1800\nG1 X{x:.3f} Y{y0:.3f} F30000\nG1 E.8 F1800\n")
            lines.append(f"G1 F3000\nG1 X{x:.3f} Y{y1:.3f} E{abs(b - a) * EPM:.5f}\n")
        up = not up
        x += SP
    lines += ["G1 E-.8 F1800\n", "G1 X5 Y5 F30000\n", "G1 E.8 F1800\n",
              "; FEATURE: Outer wall\n", "G1 X6 Y5 E.02\n"]
    return "".join(lines)


def test_ring_rewritten_across_groove():
    src = ring_gcode()
    out, reps = bf.process_gcode(src, bf.Settings())
    assert len(reps) == 1
    r = reps[0]
    assert r.rewritten
    assert r.old_max > 35
    assert r.new_max < 6.0, r.new_max

    # coverage close to the original
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    R = r.plan.region
    new = unary_union([LineString(p).buffer(W / 2, cap_style="flat") for p in r.plan.paths if len(p) > 1])
    assert new.intersection(R).area / R.area > 0.93

    # state after the block is unchanged: next feature starts at X5 Y5, unretracted
    blk = out.split("; FEATURE: Bridge", 1)[1].split("; FEATURE: Outer wall", 1)[0]
    assert "M106 S255" in blk
    d = 0.0
    for l in blk.splitlines():
        m = re.search(r"E(-?[\d.]+)", l)
        if not l.startswith("G1") or not m:
            continue
        e = float(m.group(1))
        has_xy = re.search(r"[XY]", l) is not None
        if e < 0:
            d += -e
        elif not has_xy:
            d -= e
        else:
            assert d < 1e-6, f"extrusion while retracted: {l}"
    assert abs(d) < 1e-6
    st = bf.MachineState()
    for l in (out.split("; FEATURE: Outer wall", 1)[0]).splitlines():
        bf.step(st, l)
    assert math.isclose(st.x, 5) and math.isclose(st.y, 5) and math.isclose(st.z, 1.16)


def test_already_optimal_is_kept():
    # a single straight 3 mm-wide strip bridged across: nothing to improve
    lines = ["M83\n", "G90\n", "G1 X0 Y0 Z2 F30000\n", "; FEATURE: Bridge\n", "; LINE_WIDTH: 0.42\n"]
    x = 0.21
    while x < 30:
        lines.append(f"G1 X{x:.3f} Y0 F30000\nG1 X{x:.3f} Y3 E{3 * EPM:.5f}\n")
        x += SP
    lines.append("; FEATURE: Outer wall\nG1 X1 Y1 E.01\n")
    src = "".join(lines)
    out, reps = bf.process_gcode(src, bf.Settings())
    assert not reps[0].rewritten
    assert out == src
