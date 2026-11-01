#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 fsaritome
# Part of overhangar: https://github.com/fsaritome/overhangar-gcode-postproxessor
# Additional terms (AGPL-3.0 section 7(b)): see NOTICE; author attribution must be preserved.
"""sagtest: bridge-sag test piece with anchored ("plunge + heat-stake") bridge strands.

EXPERIMENTAL and AI-assisted. It writes hand-made G-code moves, including the
nozzle pressing up to 0.2 mm INTO printed plastic. Check the slicer preview and
watch the print. No warranty, use at your own risk (see NOTICE).

Workflow:
  1. python sagtest/sagtest.py model -o sagtest_posts.stl
       -> posts on a base plate, plus sagtest_lanes.csv (measurement sheet)
  2. Slice sagtest_posts.stl in Bambu Studio (PLA, supports OFF, no auto-rotate),
     export the plate as .gcode.3mf.
  3. python sagtest/sagtest.py inject sliced.gcode.3mf -o sagtest.gcode.3mf --preview lanes.png
       -> adds one bridge strand per lane after the last post layer,
          just before the machine end G-code.
  4. Print, then measure the sag per gap and fill in the CSV.

Each lane is one strand along +X that crosses every gap. Its settings:
  mode   lay    : normal bridge, strand laid on the posts at bridge height
         touch  : like lay, but pauses (dwell) on every post
         plunge : at each post: retract, press the nozzle `depth` mm into the
                  post top, dwell, then rise while extruding (anchor + heat stake)
  depth  plunge depth below the post top surface (mm, max 0.25)
  dwell  pause on each post (s)
  flow   extrusion ratio across the gap (<1 = stretched strand, more tension)
  speed  strand speed (mm/s)
  fan    part fan 0-255
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import re
import sys
import zipfile
from dataclasses import asdict, dataclass, field

CREDIT = "overhangar by fsaritome - https://github.com/fsaritome/overhangar-gcode-postproxessor"

FIL_AREA = math.pi * (1.75 / 2) ** 2
MAX_DEPTH = 0.25


@dataclass
class Lane:
    mode: str = "lay"
    depth: float = 0.0
    dwell: float = 0.0
    flow: float = 1.0
    speed: float = 50.0
    fan: int = 255


DEFAULT_LANES = [
    Lane("lay"),                                   # 1 baseline
    Lane("lay", flow=0.85),                        # 2 stretched
    Lane("lay", flow=0.70),                        # 3 more stretched
    Lane("lay", speed=20),                         # 4 slow
    Lane("touch", dwell=0.3, flow=0.85),           # 5 dwell without plunge
    Lane("plunge", depth=0.1, dwell=0.3),          # 6
    Lane("plunge", depth=0.2, dwell=0.3),          # 7
    Lane("plunge", depth=0.2, dwell=0.8),          # 8
    Lane("plunge", depth=0.1, dwell=0.3, flow=0.85),  # 9
    Lane("plunge", depth=0.2, dwell=0.3, flow=0.85),  # 10
    Lane("plunge", depth=0.2, dwell=0.3, flow=0.70),  # 11
    Lane("lay"),                                   # 12 baseline repeat (scatter)
]


@dataclass
class Layout:
    gaps: list = field(default_factory=lambda: [5.0, 10.0, 20.0, 30.0])
    post_w: float = 4.0
    height: float = 6.0
    base_t: float = 1.2
    pitch: float = 5.0
    margin: float = 4.0
    front_ext: float = 6.0     # base sticks out at the front (-Y) = lane 1 side
    lanes: list = field(default_factory=lambda: [asdict(l) for l in DEFAULT_LANES])

    def post_x(self):
        """Left edges of the posts (model coords, post 0 at x=0)."""
        xs, x = [], 0.0
        for g in [0.0] + list(self.gaps):
            x += g
            xs.append(x)
            x += self.post_w
        return xs

    @property
    def length_y(self):
        return len(self.lanes) * self.pitch + 2 * self.margin

    def lane_y(self, i):
        return self.margin + (i + 0.5) * self.pitch

    def posts_bbox(self):
        xs = self.post_x()
        return (xs[0], 0.0, xs[-1] + self.post_w, self.length_y)


# --------------------------------------------------------------------------- model
def make_model(lay: Layout, stl_path: str, csv_path: str):
    import trimesh
    parts = []
    x0, y0, x1, y1 = lay.posts_bbox()
    base = trimesh.creation.box(extents=[x1 - x0 + 6, y1 - y0 + 3 + lay.front_ext, lay.base_t])
    base.apply_translation([(x0 + x1) / 2, (y1 + y0) / 2 - lay.front_ext / 2 + 0.0, lay.base_t / 2])
    parts.append(base)
    for px in lay.post_x():
        h = lay.height - lay.base_t
        p = trimesh.creation.box(extents=[lay.post_w, lay.length_y, h])
        p.apply_translation([px + lay.post_w / 2, lay.length_y / 2, lay.base_t + h / 2])
        parts.append(p)
    mesh = trimesh.boolean.union(parts, engine="manifold")
    mesh.export(stl_path)
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["lane", "y_from_front_mm", "mode", "depth_mm", "dwell_s", "flow", "speed_mm_s", "fan"]
                   + [f"sag_{g:g}mm" for g in lay.gaps] + ["adhesion_0to3", "notes"])
        for i, l in enumerate(lay.lanes):
            w.writerow([i + 1, round(lay.lane_y(i) + lay.front_ext + 1.5, 1), l["mode"], l["depth"], l["dwell"],
                        l["flow"], l["speed"], l["fan"]] + [""] * len(lay.gaps) + ["", ""])
    return mesh


# --------------------------------------------------------------------------- gcode analysis
WORD_RE = re.compile(r"([A-Z])([-+]?\d*\.?\d+)")


def top_layer(lines):
    """Return (top Z, list of top-layer extrusion endpoints, E mode relative?)."""
    x = y = z = 0.0
    rel_e = True
    abs_xyz = True
    pts = {}
    in_exec = False
    for line in lines:
        if line.startswith("; EXECUTABLE_BLOCK_START"):
            in_exec = True
        if line.startswith("; MACHINE_END_GCODE_START"):
            break
        code = line.split(";", 1)[0].strip().upper()
        if not code:
            continue
        cmd, _, rest = code.partition(" ")
        if cmd == "M83":
            rel_e = True
        elif cmd == "M82":
            rel_e = False
        elif cmd == "G90":
            abs_xyz = True
        elif cmd == "G91":
            abs_xyz = False
        elif cmd in ("G0", "G1", "G2", "G3"):
            w = {k: float(v) for k, v in WORD_RE.findall(rest)}
            if abs_xyz:
                nx, ny, nz = w.get("X", x), w.get("Y", y), w.get("Z", z)
            else:
                nx, ny, nz = x + w.get("X", 0), y + w.get("Y", 0), z + w.get("Z", 0)
            if in_exec and w.get("E", 0) > 0 and ("X" in w or "Y" in w):
                pts.setdefault(round(nz, 3), []).extend([(x, y), (nx, ny)])
            x, y, z = nx, ny, nz
    if not pts:
        raise SystemExit("no extrusions found")
    ztop = max(pts)
    return ztop, pts[ztop], rel_e


def deficit_before(lines, idx):
    """Filament currently retracted at line idx (sum of retractions since last extrusion)."""
    d = 0.0
    for line in reversed(lines[:idx]):
        code = line.split(";", 1)[0].strip().upper()
        if not code.startswith(("G0", "G1", "G2", "G3")):
            continue
        w = {k: float(v) for k, v in WORD_RE.findall(code[2:])}
        e = w.get("E")
        if e is None:
            continue
        has_xy = "X" in w or "Y" in w
        if e > 0 and has_xy:
            break
        d += -e
    return max(0.0, d)


# --------------------------------------------------------------------------- strand generation
@dataclass
class Machine:
    retract: float = 0.8
    retract_f: float = 1800
    stake_retract: float = 0.4
    travel_f: float = 12000
    plunge_f: float = 120          # 2 mm/s into the post
    rise_dx: float = 0.6           # forward move while rising out of the plunge
    lift: float = 2.0              # travel height above the post tops
    strand_h: float = 0.2          # nozzle tip above post top when laying a strand
    strand_d: float = 0.4          # strand diameter used for E/mm (round "thick bridge")


def f3(v):
    s = f"{v:.3f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def lane_gcode(i, lane: Lane, lay: Layout, off, ztop, m: Machine, start_def):
    """G-code for one lane; returns (lines, end deficit, path for preview)."""
    if lane.mode not in ("lay", "touch", "plunge"):
        raise ValueError(f"lane {i + 1}: unknown mode {lane.mode}")
    if not 0 <= lane.depth <= MAX_DEPTH:
        raise ValueError(f"lane {i + 1}: depth must be 0..{MAX_DEPTH} mm")
    epm = math.pi * (m.strand_d / 2) ** 2 / FIL_AREA
    zb = ztop + m.strand_h
    zp = ztop - lane.depth
    zt = ztop + m.lift
    y = off[1] + lay.lane_y(i)
    centers = [off[0] + px + lay.post_w / 2 for px in lay.post_x()]
    o = [f"; --- lane {i + 1}: {lane.mode} depth={lane.depth} dwell={lane.dwell}s "
         f"flow={lane.flow} speed={lane.speed} fan={lane.fan}\n",
         f"M106 S{int(lane.fan)}\n"]
    path = []
    d = start_def

    def retract_to(target):
        nonlocal d
        if target > d + 1e-6:
            o.append(f"G1 E-{f3(target - d)} F{f3(m.retract_f)}\n")
            d = target

    retract_to(m.retract)
    o.append(f"G1 Z{f3(zt)} F600\n")
    o.append(f"G1 X{f3(centers[0])} Y{f3(y)} F{f3(m.travel_f)}\n")
    x = centers[0]

    def anchor(first):
        """At a post centre: lay / touch / plunge."""
        nonlocal d, x
        if lane.mode == "plunge":
            if not first:
                retract_to(m.stake_retract)
            o.append(f"G1 Z{f3(zp)} F{f3(m.plunge_f)}\n")
            path.append((x, zp))
            if lane.dwell > 0:
                o.append(f"G4 P{int(lane.dwell * 1000)}\n")
            # unretract, then rise while extruding a little (fills the dimple)
            o.append(f"G1 E{f3(d)} F{f3(m.retract_f)}\n")
            d = 0.0
            nx = x + m.rise_dx
            o.append(f"G1 X{f3(nx)} Z{f3(zb)} E{epm * m.rise_dx:.5f} F300\n")
            x = nx
            path.append((x, zb))
        else:
            if first:
                o.append(f"G1 Z{f3(zb)} F600\n")
                o.append(f"G1 E{f3(d)} F{f3(m.retract_f)}\n")
                d = 0.0
                path.append((x, zb))
            if lane.mode == "touch" and lane.dwell > 0:
                o.append(f"G4 P{int(lane.dwell * 1000)}\n")

    anchor(True)
    for k in range(1, len(centers)):
        tx = centers[k]
        L = tx - x
        o.append(f"G1 X{f3(tx)} E{epm * lane.flow * L:.5f} F{f3(lane.speed * 60)}\n")
        x = tx
        path.append((x, zb))
        if k < len(centers) - 1 or lane.mode == "plunge":
            anchor(False)
    # finish on the last post: short tail, retract, lift
    tail = min(1.5, centers[-1] + lay.post_w / 2 - 0.6 - x)
    if tail > 0.1:
        o.append(f"G1 X{f3(x + tail)} E{epm * tail:.5f} F{f3(lane.speed * 60)}\n")
        x += tail
        path.append((x, zb))
    retract_to(m.retract)
    o.append(f"G1 Z{f3(zt)} F600\n")
    return o, d, path


def build_block(lay: Layout, off, ztop, start_def, m: Machine):
    o = ["; FEATURE: Custom\n",
         f"; ===== sagtest bridge strands ({CREDIT}) =====\n",
         "; EXPERIMENTAL, AI-assisted G-code. Watch the print.\n",
         "M83\n", "G90\n", "M204 S2000\n"]
    d = start_def
    paths = []
    for i, ld in enumerate(lay.lanes):
        lines, d, path = lane_gcode(i, Lane(**ld), lay, off, ztop, m, d)
        o += lines
        paths.append(path)
    o += ["M106 S0\n", "M106 P2 S0\n",
          "; ===== sagtest end =====\n"]
    # leave the same retraction state the end G-code expects
    if abs(d - start_def) > 1e-6:
        delta = d - start_def
        o.insert(-1, f"G1 E{f3(delta)} F{f3(m.retract_f)}\n" if delta > 0 else
                 f"G1 E-{f3(-delta)} F{f3(m.retract_f)}\n")
    return o, paths


def check_block(block, lay: Layout, off, ztop):
    """Hard safety limits for every move in the generated block."""
    x0, y0, x1, y1 = lay.posts_bbox()
    bx0, by0, bx1, by1 = x0 + off[0] - 1, y0 + off[1], x1 + off[0] + 1, y1 + off[1]
    for line in block:
        code = line.split(";", 1)[0].strip().upper()
        if not code.startswith("G1"):
            continue
        w = {k: float(v) for k, v in WORD_RE.findall(code[2:])}
        if "X" in w and not bx0 <= w["X"] <= bx1:
            raise SystemExit(f"safety: X out of range: {line.strip()}")
        if "Y" in w and not by0 <= w["Y"] <= by1:
            raise SystemExit(f"safety: Y out of range: {line.strip()}")
        if "Z" in w and w["Z"] < ztop - MAX_DEPTH - 1e-6:
            raise SystemExit(f"safety: Z too low: {line.strip()}")
        if "E" in w and abs(w["E"]) > 3:
            raise SystemExit(f"safety: E step too large: {line.strip()}")


def inject_text(text: str, lay: Layout, m: Machine):
    lines = text.splitlines(keepends=True)
    try:
        end_idx = next(i for i, l in enumerate(lines) if l.startswith("; MACHINE_END_GCODE_START"))
    except StopIteration:
        raise SystemExit("no '; MACHINE_END_GCODE_START' marker: only Bambu Studio / Orca G-code is supported")
    if end_idx > 0 and lines[end_idx - 1].startswith("; FEATURE: Custom"):
        end_idx -= 1
    ztop, pts, rel_e = top_layer(lines)
    if not rel_e:
        raise SystemExit("G-code uses absolute E (M82); relative E (M83) required")
    gx = [p[0] for p in pts]
    gy = [p[1] for p in pts]
    x0, y0, x1, y1 = lay.posts_bbox()
    off = ((min(gx) + max(gx)) / 2 - (x0 + x1) / 2, (min(gy) + max(gy)) / 2 - (y0 + y1) / 2)
    # the top layer must be our posts and nothing else, unrotated
    gw, gh = max(gx) - min(gx), max(gy) - min(gy)
    if abs(gw - (x1 - x0)) > 1.5 or abs(gh - (y1 - y0)) > 1.5:
        raise SystemExit(f"top layer is {gw:.1f} x {gh:.1f} mm, expected {x1 - x0:.1f} x {y1 - y0:.1f}: "
                         "wrong model, rotated, or other objects on the plate")
    rects = [(off[0] + px - 0.8, off[0] + px + lay.post_w + 0.8) for px in lay.post_x()]
    stray = [p for p in pts if not any(a <= p[0] <= b for a, b in rects)]
    if stray:
        raise SystemExit(f"{len(stray)} top-layer points outside the posts: wrong model or layout")
    if abs(ztop - lay.height) > 0.3:
        raise SystemExit(f"top Z {ztop} does not match post height {lay.height}")
    start_def = deficit_before(lines, end_idx)
    block, paths = build_block(lay, off, ztop, start_def, m)
    check_block(block, lay, off, ztop)
    return "".join(lines[:end_idx] + block + lines[end_idx:]), dict(ztop=ztop, off=off, paths=paths)


def inject_file(src, dst, lay, m):
    if src.lower().endswith(".3mf"):
        zin = zipfile.ZipFile(src)
        names = [i.filename for i in zin.infolist()]
        g = [n for n in names if re.fullmatch(r"Metadata/plate_\d+\.gcode", n)]
        if len(g) != 1:
            raise SystemExit(f"expected exactly one plate G-code, found {g}")
        text, info = inject_text(zin.read(g[0]).decode("utf-8"), lay, m)
        data = text.encode("utf-8")
        out = io.BytesIO()
        with zipfile.ZipFile(out, "w") as zout:
            for it in zin.infolist():
                if it.filename == g[0]:
                    zout.writestr(it, data)
                elif it.filename == g[0] + ".md5":
                    zout.writestr(it, hashlib.md5(data).hexdigest().upper())
                else:
                    zout.writestr(it, zin.read(it.filename))
        with open(dst, "wb") as f:
            f.write(out.getvalue())
    else:
        with open(src, encoding="utf-8") as f:
            text, info = inject_text(f.read(), lay, m)
        with open(dst, "w", encoding="utf-8") as f:
            f.write(text)
    return info


def preview(info, lay: Layout, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    zt = info["ztop"]
    fig, axs = plt.subplots(len(lay.lanes), 1, figsize=(12, 1.2 * len(lay.lanes)), sharex=True)
    for i, (ax, p) in enumerate(zip(axs, info["paths"])):
        for px in lay.post_x():
            x = info["off"][0] + px
            ax.fill_between([x, x + lay.post_w], zt - 1, zt, color="#ccc")
        xs, zs = zip(*p)
        ax.plot(xs, zs, "-o", ms=2, lw=1, color="#2e6fbd")
        l = lay.lanes[i]
        ax.set_ylabel(f"{i + 1}", rotation=0, labelpad=12)
        ax.text(1.01, 0.5, f"{l['mode']} d{l['depth']} t{l['dwell']} f{l['flow']} v{l['speed']}",
                transform=ax.transAxes, va="center", fontsize=8)
        ax.set_ylim(zt - 0.6, zt + 0.5)
        ax.set_yticks([])
    axs[0].set_title("nozzle tip path per lane (side view, Z exaggerated); grey = post tops")
    fig.tight_layout()
    fig.savefig(path, dpi=100)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=CREDIT + " | AGPL-3.0 | experimental, use at your own risk")
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("model", help="write the post model (STL) and the measurement sheet (CSV)")
    a.add_argument("-o", "--output", default="sagtest_posts.stl")
    a.add_argument("--csv", default="sagtest_lanes.csv")
    a.add_argument("--layout", help="layout JSON to use/write (default: built-in)")
    b = sub.add_parser("inject", help="add the bridge strands to a sliced post model")
    b.add_argument("input")
    b.add_argument("-o", "--output", required=True)
    b.add_argument("--layout", help="layout JSON (must match the model you sliced)")
    b.add_argument("--preview", help="PNG with the nozzle path per lane")
    args = ap.parse_args(argv)

    lay = Layout()
    if args.layout:
        try:
            with open(args.layout) as f:
                lay = Layout(**json.load(f))
        except FileNotFoundError:
            if args.cmd == "inject":
                raise
            with open(args.layout, "w") as f:
                json.dump(asdict(lay), f, indent=2)
    if args.cmd == "model":
        make_model(lay, args.output, args.csv)
        x0, y0, x1, y1 = lay.posts_bbox()
        print(f"wrote {args.output} ({x1 - x0 + 6:.0f} x {y1 - y0 + 3 + lay.front_ext:.0f} x {lay.height} mm) "
              f"and {args.csv}. Front of the base (longer overhang) = lane 1.", file=sys.stderr)
    else:
        info = inject_file(args.input, args.output, lay, Machine())
        print(f"injected {len(lay.lanes)} lanes at post top Z{info['ztop']}, "
              f"offset X{info['off'][0]:.2f} Y{info['off'][1]:.2f} -> {args.output}", file=sys.stderr)
        if args.preview:
            preview(info, lay, args.preview)


if __name__ == "__main__":
    main()
