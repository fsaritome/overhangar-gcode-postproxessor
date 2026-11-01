# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 fsaritome
# Part of overhangar: https://github.com/fsaritome/overhangar-gcode-postproxessor
# Additional terms (AGPL-3.0 section 7(b)): see NOTICE; author attribution must be preserved.
"""sagtest: inject strands into a synthetic Bambu-style slice of the post model."""
import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sagtest"))
import sagtest as st  # noqa: E402

END = """; close powerlost recovery
M1003 S0
; WIPE_START
G1 X150 Y102 E-.76
; WIPE_END
G1 E-.04 F1800
G1 Z6.4 F30000
M106 S0
M106 P2 S0
M981 S0 P20000 ; close spaghetti detector
; FEATURE: Custom
; MACHINE_END_GCODE_START
M400
G92 E0
G1 E-0.8 F1800
G1 Z6.5 F900
; EXECUTABLE_BLOCK_END
"""


def fake_slice(lay, ox=80.0, oy=90.0, rotate=False):
    o = ["; HEADER_BLOCK_START\n; HEADER_BLOCK_END\n", "; EXECUTABLE_BLOCK_START\n", "M83\n", "G90\n"]
    zs = [0.2] + [round(0.2 + 0.16 * k, 2) for k in range(1, 40)]
    zs = [z for z in zs if z <= lay.height + 1e-6]
    if abs(zs[-1] - lay.height) > 1e-6:
        zs.append(lay.height)
    for z in zs:
        o.append(f"G1 Z{z}\n")
        for px in lay.post_x():
            x0, x1 = ox + px + 0.21, ox + px + lay.post_w - 0.21
            y0, y1 = oy + 0.21, oy + lay.length_y - 0.21
            if rotate:
                x0, x1, y0, y1 = y0, y1, x0, x1
            o.append(f"G1 X{x0} Y{y0} F30000\n")
            for (x, y) in [(x1, y0), (x1, y1), (x0, y1), (x0, y0)]:
                o.append(f"G1 X{x} Y{y} E.05\n")
    return "".join(o) + END


def simulate(text):
    """Return list of (x, y, z, e) for every move plus final retract deficit."""
    x = y = z = 0.0
    moves = []
    for line in text.splitlines():
        code = line.split(";", 1)[0].strip()
        if not code.startswith("G1"):
            continue
        w = {k: float(v) for k, v in re.findall(r"([XYZEF])([-+]?\d*\.?\d+)", code)}
        x, y, z = w.get("X", x), w.get("Y", y), w.get("Z", z)
        moves.append((x, y, z, w.get("E", 0.0), "X" in w or "Y" in w))
    return moves


def test_inject_positions_and_safety():
    lay = st.Layout()
    src = fake_slice(lay)
    out, info = st.inject_text(src, lay, st.Machine())
    assert abs(info["ztop"] - lay.height) < 1e-6
    assert abs(info["off"][0] - 80.0) < 0.01 and abs(info["off"][1] - 90.0) < 0.01
    # block sits right before the end G-code and after the spaghetti-detector shutdown
    i_blk = out.index("sagtest bridge strands")
    assert out.index("M981 S0") < i_blk < out.index("; MACHINE_END_GCODE_START")
    blk = out[i_blk:out.index("; MACHINE_END_GCODE_START")]
    mv = simulate(blk)
    zmin = min(m[2] for m in mv if m[2] > 0)
    assert zmin >= lay.height - st.MAX_DEPTH - 1e-6
    assert any(abs(m[2] - (lay.height - 0.2)) < 1e-6 for m in mv)  # plunge lanes reach 0.2 below top

    # retraction bookkeeping: never extrude while retracted; end state = start state (0.8)
    d = 0.8
    for x, y, z, e, xy in mv:
        if e < 0:
            d += -e
        elif e > 0 and not xy:
            d -= e
        elif e > 0:
            assert d < 1e-6, "extrusion while retracted"
    assert abs(d - 0.8) < 1e-6

    # every lane crosses every post centre at its own Y
    for i in range(len(lay.lanes)):
        y = 90.0 + lay.lane_y(i)
        xs = [m[0] for m in mv if abs(m[1] - y) < 1e-6 and m[3] > 0]
        for px in lay.post_x():
            c = 80.0 + px + lay.post_w / 2
            assert min(xs) <= c + 1e-6 and max(xs) >= c - 1e-6


def test_rejects_wrong_or_rotated_model():
    lay = st.Layout()
    with pytest.raises(SystemExit):
        st.inject_text(fake_slice(lay, rotate=True), lay, st.Machine())
    with pytest.raises(SystemExit):
        st.inject_text(fake_slice(lay).replace("; MACHINE_END_GCODE_START", "; END"), lay, st.Machine())


def test_depth_limit():
    lay = st.Layout(lanes=[dict(mode="plunge", depth=0.5, dwell=0.3, flow=1.0, speed=50, fan=255)])
    with pytest.raises(ValueError):
        st.inject_text(fake_slice(lay), lay, st.Machine())
