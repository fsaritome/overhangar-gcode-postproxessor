#!/usr/bin/env python3
"""bridgefix: re-plan bridge infill so every bridge line crosses the SHORTEST span.

Bambu Studio / OrcaSlicer choose one bridge direction per bridge island (or a
fixed `bridge_angle`). For ring-shaped or L-shaped bridges (gasket-groove
ceilings, slots, frames) that means some lines run lengthwise along the gap
instead of across it. This post-processor:

  1. finds every `; FEATURE: Bridge` block in the G-code,
  2. rebuilds the bridged area from the extrusion moves,
  3. for each point of that area finds the direction with the shortest
     anchor-to-anchor chord, groups points into zones of equal direction,
  4. refills each zone with straight lines in its own direction, keeping
     Bambu's flow (E/mm), speed, line spacing, fan and other commands,
  5. restores position, Z and retraction state at the end of the block, so the
     rest of the file is untouched.

Usage:
  bridgefix.py in.gcode.3mf -o out.gcode.3mf [--preview DIR]
  bridgefix.py in.gcode -o out.gcode
  bridgefix.py path/to/file.gcode            # in place (slicer post-processing script)
"""
from __future__ import annotations

import argparse
import hashlib
import io
import math
import re
import sys
import zipfile
from dataclasses import dataclass, field

import numpy as np
import shapely
from shapely import affinity
from shapely.geometry import LineString, MultiLineString, Point, Polygon, box
from shapely.ops import unary_union

FEATURE_RE = re.compile(r"^;\s*(?:FEATURE|TYPE):\s*(.+?)\s*$")
LAYER_RE = re.compile(r"^;\s*(CHANGE_LAYER|LAYER_CHANGE)\b")
WORD_RE = re.compile(r"([A-Z])([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)")
LINE_WIDTH_RE = re.compile(r"^;\s*LINE_WIDTH:\s*([\d.]+)")


# --------------------------------------------------------------------------- config
@dataclass
class Settings:
    retract_len: float = 0.8
    retract_f: float = 1800.0
    z_hop: float = 0.4
    travel_f: float = 30000.0
    angles_deg: float = 15.0      # direction resolution
    grid: float = 0.8             # zone grid cell (mm)
    min_improvement: float = 0.7  # rewrite only if new max span < this * old max span
    anchor: float = 0.0           # extra extension of line ends into the anchor (mm)
    min_zone_area: float = 6.0    # zones smaller than this (mm^2) merge into a neighbour
    single_slack: float = 2.0     # keep one direction per island if its worst span is within this (mm)
    features: tuple = ("Bridge",)
    force: bool = False


def settings_from_config(text: str, s: Settings) -> Settings:
    def get(key):
        m = re.search(rf"^;\s*{key}\s*=\s*([^\n,]+)", text, re.M)
        return float(m.group(1)) if m else None

    v = get("retraction_length")
    if v is not None:
        s.retract_len = v
    v = get("retraction_speed")
    if v is not None:
        s.retract_f = v * 60
    v = get("z_hop")
    if v is not None:
        s.z_hop = v
    v = get("travel_speed")
    if v is not None:
        s.travel_f = v * 60
    return s


# --------------------------------------------------------------------------- parsing
@dataclass
class Move:
    x0: float
    y0: float
    x1: float
    y1: float
    e: float
    f: float
    z: float


@dataclass
class MachineState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    f: float = 0.0
    abs_xyz: bool = True
    rel_e: bool = True
    deficit: float = 0.0  # filament currently retracted (mm)


def parse_words(line: str) -> tuple[str, dict]:
    code = line.split(";", 1)[0].strip()
    if not code:
        return "", {}
    parts = code.split(None, 1)
    cmd = parts[0].upper()
    words = {k: float(v) for k, v in WORD_RE.findall(parts[1].upper())} if len(parts) > 1 else {}
    return cmd, words


def arc_points(x0, y0, x1, y1, i, j, cw, seg=0.25):
    cx, cy = x0 + i, y0 + j
    r = math.hypot(i, j)
    a0 = math.atan2(y0 - cy, x0 - cx)
    a1 = math.atan2(y1 - cy, x1 - cx)
    if cw:
        da = a0 - a1
        if da <= 1e-9:
            da += 2 * math.pi
        da = -da
    else:
        da = a1 - a0
        if da <= 1e-9:
            da += 2 * math.pi
    n = max(1, int(math.ceil(abs(da) * r / seg)))
    pts = [(cx + r * math.cos(a0 + da * k / n), cy + r * math.sin(a0 + da * k / n)) for k in range(1, n)]
    pts.append((x1, y1))
    return pts


def step(st: MachineState, line: str, moves: list | None = None):
    """Advance machine state by one line; optionally collect XY moves (arcs linearized)."""
    cmd, w = parse_words(line)
    if not cmd:
        return
    if cmd == "G90":
        st.abs_xyz = True
    elif cmd == "G91":
        st.abs_xyz = False
    elif cmd == "M83":
        st.rel_e = True
    elif cmd == "M82":
        st.rel_e = False
    elif cmd == "G92":
        pass  # E reset; we only use relative E
    elif cmd in ("G0", "G1", "G2", "G3"):
        if "F" in w:
            st.f = w["F"]
        x0, y0 = st.x, st.y
        if st.abs_xyz:
            x1, y1, z1 = w.get("X", st.x), w.get("Y", st.y), w.get("Z", st.z)
        else:
            x1, y1, z1 = st.x + w.get("X", 0), st.y + w.get("Y", 0), st.z + w.get("Z", 0)
        e = w.get("E", 0.0)
        has_xy = ("X" in w or "Y" in w) and (abs(x1 - x0) > 1e-9 or abs(y1 - y0) > 1e-9)
        if e < 0:
            st.deficit += -e
        elif e > 0 and not has_xy:
            st.deficit = max(0.0, st.deficit - e)
        if moves is not None and has_xy:
            if cmd in ("G2", "G3") and ("I" in w or "J" in w):
                pts = arc_points(x0, y0, x1, y1, w.get("I", 0), w.get("J", 0), cmd == "G2")
                lens = [math.hypot(px - qx, py - qy) for (px, py), (qx, qy) in zip([(x0, y0)] + pts[:-1], pts)]
                tot = sum(lens) or 1.0
                px, py = x0, y0
                for (qx, qy), ln in zip(pts, lens):
                    moves.append(Move(px, py, qx, qy, e * ln / tot, st.f, z1))
                    px, py = qx, qy
            else:
                moves.append(Move(x0, y0, x1, y1, e, st.f, z1))
        st.x, st.y, st.z = x1, y1, z1


# --------------------------------------------------------------------------- planning
@dataclass
class Plan:
    region: object
    zones: list            # (angle_deg, polygon)
    paths: list            # list of list[(x,y)] polylines, each extruded continuously
    old_spans: list
    new_spans: list
    width: float
    spacing: float
    e_per_mm: float
    f_print: float
    z: float


def chord_length(region, p: Point, theta: float, reach: float) -> float:
    dx, dy = math.cos(theta) * reach, math.sin(theta) * reach
    ln = LineString([(p.x - dx, p.y - dy), (p.x + dx, p.y + dy)])
    inter = region.intersection(ln)
    if inter.is_empty:
        return 0.0
    parts = getattr(inter, "geoms", [inter])
    best = None
    for g in parts:
        if g.geom_type != "LineString":
            continue
        d = g.distance(p)
        if best is None or d < best[0]:
            best = (d, g.length)
    return best[1] if best else 0.0


def detect_spacing(ext: list[Move], width: float) -> float:
    """Median perpendicular distance between consecutive long parallel lines."""
    longs = [m for m in ext if math.hypot(m.x1 - m.x0, m.y1 - m.y0) > 2.0]
    ds = []
    for a, b in zip(longs, longs[1:]):
        ux, uy = a.x1 - a.x0, a.y1 - a.y0
        L = math.hypot(ux, uy)
        vx, vy = b.x1 - b.x0, b.y1 - b.y0
        if abs(ux * vy - uy * vx) / (L * math.hypot(vx, vy)) > 0.05:
            continue  # not parallel
        d = abs((b.x0 - a.x0) * uy - (b.y0 - a.y0) * ux) / L
        if 0.2 * width < d < 2.0 * width:
            ds.append(d)
    return float(np.median(ds)) if ds else width * 0.92


def plan_block(moves: list[Move], width: float, s: Settings) -> Plan | None:
    ext = [m for m in moves if m.e > 0 and math.hypot(m.x1 - m.x0, m.y1 - m.y0) > 1e-6]
    if not ext:
        return None
    lens = np.array([math.hypot(m.x1 - m.x0, m.y1 - m.y0) for m in ext])
    epm = float(np.median([m.e / l for m, l in zip(ext, lens) if l > 0.5] or [ext[0].e / lens[0]]))
    fs = [m.f for m in ext]
    f_print = max(set(fs), key=fs.count)
    zs = [m.z for m in ext]
    z = max(set(zs), key=zs.count)
    spacing = detect_spacing(ext, width)

    segs = [LineString([(m.x0, m.y0), (m.x1, m.y1)]) for m in ext]
    region = unary_union(shapely.buffer(segs, width / 2, cap_style="flat", join_style="mitre"))
    region = region.buffer(0.15, join_style="mitre").buffer(-0.15, join_style="mitre")
    region = region.buffer(0)
    if region.is_empty or region.area < 1.0:
        return None

    # original free spans: straight extrusion runs (merge collinear consecutive moves)
    old_spans = merged_run_lengths(ext)

    reach = math.hypot(*np.ptp(np.array(region.bounds).reshape(2, 2), axis=0)) + 1
    n_ang = max(2, int(round(180 / s.angles_deg)))
    thetas = [math.pi * k / n_ang for k in range(n_ang)]
    g = s.grid
    minx, miny, maxx, maxy = region.bounds
    nx, ny = int(math.ceil((maxx - minx) / g)), int(math.ceil((maxy - miny) / g))
    cells = {}
    prepared = shapely.prepared.prep(region)
    longs = [m for m in ext if math.hypot(m.x1 - m.x0, m.y1 - m.y0) > 0.8]
    orig_idx = longs
    orig_tree = shapely.STRtree([LineString([(m.x0, m.y0), (m.x1, m.y1)]) for m in longs]) if longs else None
    for ix in range(nx):
        for iy in range(ny):
            cb = box(minx + ix * g, miny + iy * g, minx + (ix + 1) * g, miny + (iy + 1) * g)
            if not prepared.intersects(cb):
                continue
            c = cb.centroid
            if not prepared.contains(c):
                piece = region.intersection(cb)
                if piece.is_empty or piece.area < 1e-6:
                    continue
                c = piece.representative_point()
            ch = np.array([chord_length(region, c, t, reach) for t in thetas])
            ch[ch <= 0] = np.inf
            # prefer the slicer's own direction when it is (nearly) as short: fewer changes
            k = orig_angle_index(orig_tree, orig_idx, c, n_ang)
            if k is not None and ch[k] <= ch.min() * 1.08 + 0.1:
                ch = ch.copy()
                ch[k] = ch.min() - 1e-3
            cells[(ix, iy)] = ch
    if not cells:
        return None

    label = label_cells(cells, region, minx, miny, g, s)

    zones = []
    for a in sorted(set(label.values())):
        sq = [box(minx + ix * g, miny + iy * g, minx + (ix + 1) * g, miny + (iy + 1) * g)
              for (ix, iy), lab in label.items() if lab == a]
        zp = unary_union(sq).intersection(region)
        if not zp.is_empty and zp.area > 1e-4:
            zones.append((math.degrees(thetas[a]), zp))

    boundary = region.boundary
    hug = boundary.buffer(width * 0.45)
    segments = []  # (angle, LineString)
    for ang, zp in zones:
        segments += [(ang, l) for l in fill_zone(zp, ang, spacing, hug, s.anchor, 0.0)]
    segments += fill_gaps(region, segments, width, spacing, hug, s)
    if not segments:
        return None
    paths = order_paths(segments, spacing, region)
    new_spans = [l.length for _, l in segments]
    return Plan(region, zones, paths, old_spans, new_spans, width, spacing, epm, f_print, z)


def orig_angle_index(tree, moves, p: Point, n_ang: int):
    """Direction bucket of the nearest original bridge line, if it passes close to p."""
    if tree is None:
        return None
    i = tree.nearest(p)
    m = moves[int(i)]
    if LineString([(m.x0, m.y0), (m.x1, m.y1)]).distance(p) > 0.6:
        return None
    a = math.atan2(m.y1 - m.y0, m.x1 - m.x0) % math.pi
    return int(round(a / (math.pi / n_ang))) % n_ang


def neighbours(k):
    ix, iy = k
    return [(ix + 1, iy), (ix - 1, iy), (ix, iy + 1), (ix, iy - 1)]


def label_cells(cells: dict, region, minx, miny, g, s: Settings) -> dict:
    """Assign a fill direction per grid cell.

    Per connected island: if one direction is nearly as good as the per-cell
    optimum everywhere, use it for the whole island (no fragmentation).
    Otherwise label by per-cell optimum, smooth, then merge small zones.
    """
    label = {}
    geoms = list(getattr(region, "geoms", [region]))
    by_island = {}
    for k, ch in cells.items():
        c = Point(minx + (k[0] + 0.5) * g, miny + (k[1] + 0.5) * g)
        idx = min(range(len(geoms)), key=lambda t: geoms[t].distance(c))
        by_island.setdefault(idx, []).append(k)

    min_cells = max(4, int(math.ceil(s.min_zone_area / (g * g))))
    for keys in by_island.values():
        M = np.array([cells[k] for k in keys])           # cells x angles
        opt = M.min(axis=1)
        worst_opt = opt.max()
        single = M.max(axis=0)                            # worst span per direction
        a_single = int(np.argmin(single))
        if single[a_single] <= max(worst_opt * 1.3, worst_opt + s.single_slack):
            for k in keys:
                label[k] = a_single
            continue

        lab = {k: int(np.argmin(cells[k])) for k in keys}
        ok = {k: set(np.nonzero(cells[k] <= cells[k].min() * 1.15 + 0.3)[0].tolist()) for k in keys}
        for _ in range(4):  # majority smoothing among near-equal directions
            new = {}
            for k in keys:
                votes = {}
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nb = lab.get((k[0] + dx, k[1] + dy))
                        if nb is not None:
                            votes[nb] = votes.get(nb, 0) + 1
                cand = [a for a in sorted(votes, key=lambda a: -votes[a]) if a in ok[k]]
                new[k] = cand[0] if cand else lab[k]
            lab = new
        for _ in range(20):  # merge zones smaller than min_cells into best neighbour
            changed = False
            seen = set()
            for k in keys:
                if k in seen:
                    continue
                comp, stack = [], [k]
                seen.add(k)
                while stack:
                    c = stack.pop()
                    comp.append(c)
                    for nb in neighbours(c):
                        if nb in lab and nb not in seen and lab[nb] == lab[k]:
                            seen.add(nb)
                            stack.append(nb)
                if len(comp) >= min_cells:
                    continue
                cand = {lab[nb] for c in comp for nb in neighbours(c) if nb in lab and lab[nb] != lab[k]}
                if not cand:
                    continue
                best = min(cand, key=lambda a: max(cells[c][a] for c in comp))
                for c in comp:
                    lab[c] = best
                changed = True
            if not changed:
                break
        label.update(lab)
    return label


def merged_run_lengths(ext: list[Move]) -> list[float]:
    runs, cur, last = [], 0.0, None
    for m in ext:
        d = (m.x1 - m.x0, m.y1 - m.y0)
        L = math.hypot(*d)
        if last is not None and abs(last[0] * d[1] - last[1] * d[0]) < 0.02 * L * math.hypot(*last) \
                and (last[0] * d[0] + last[1] * d[1]) > 0 and cont_end == (m.x0, m.y0):
            cur += L
        else:
            if cur:
                runs.append(cur)
            cur = L
        last, cont_end = d, (m.x1, m.y1)
    if cur:
        runs.append(cur)
    return runs


def fill_zone(poly, angle_deg, spacing, hug, anchor, phase, centered=False):
    """Scan-line fill. Lines run to the region edge (nozzle centred on the anchor);
    lines that would just trace along an edge (inside `hug`) are dropped."""
    if poly.is_empty:
        return []
    rot = affinity.rotate(poly, -angle_deg, origin=(0, 0))
    minx, miny, maxx, maxy = rot.bounds
    if centered:
        n = max(1, int(round((maxy - miny) / spacing)))
        ys = [miny + (maxy - miny) * (k + 0.5) / n for k in range(n)]
    else:
        k0, k1 = math.floor(miny / spacing), math.ceil(maxy / spacing)
        ys = [(k + 0.5 + phase) * spacing for k in range(k0, k1 + 1)]
    out = []
    for y in ys:
        ln = LineString([(minx - 1, y), (maxx + 1, y)])
        inter = rot.intersection(ln)
        for g in getattr(inter, "geoms", [inter]):
            if g.geom_type != "LineString" or g.length < 1e-6:
                continue
            (ax, ay), (bx, by) = g.coords[0], g.coords[-1]
            if ax > bx:
                ax, bx = bx, ax
            ax, bx = ax - anchor, bx + anchor
            p = affinity.rotate(LineString([(ax, ay), (bx, by)]), angle_deg, origin=(0, 0))
            if p.length < 0.25 or hug.contains(p):
                continue
            out.append(p)
    return out


def fill_gaps(region, segments, width, spacing, hug, s: Settings):
    """Second pass: fill uncovered pockets (zone seams, slivers) with their own best direction."""
    if not segments:
        return []
    covered = unary_union(shapely.buffer([l for _, l in segments], width / 2, cap_style="flat"))
    gaps = region.difference(covered)
    out = []
    n_ang = max(2, int(round(180 / s.angles_deg)))
    for g in getattr(gaps, "geoms", [gaps]):
        if g.geom_type != "Polygon" or g.area < 0.25:
            continue
        core = g.buffer(-width * 0.25)
        if core.is_empty:
            continue
        best = None
        for k in range(n_ang):
            a = 180.0 * k / n_ang
            ls = fill_zone(g, a, spacing, hug, 0.0, 0.0, centered=True)
            if not ls:
                continue
            cov = unary_union(shapely.buffer(ls, width / 2, cap_style="flat")).intersection(g).area
            score = (round(cov / g.area, 1), -max(l.length for l in ls))
            if best is None or score > best[0]:
                best = (score, a, ls)
        if best:
            out += [(best[1], l) for l in best[2]]
    return out


def order_paths(segments, spacing, region):
    """Greedy nearest-neighbour ordering; short in-region hops become extruded connectors."""
    remaining = list(range(len(segments)))
    paths, cur = [], None
    pos = None
    near = region.buffer(0.05)
    while remaining:
        best = None
        for i in remaining:
            c = segments[i][1].coords
            for rev in (False, True):
                start = c[-1] if rev else c[0]
                d = 0.0 if pos is None else math.hypot(start[0] - pos[0], start[1] - pos[1])
                if best is None or d < best[0]:
                    best = (d, i, rev)
        d, i, rev = best
        remaining.remove(i)
        pts = list(segments[i][1].coords)
        if rev:
            pts.reverse()
        connect = (cur is not None and d <= 2.2 * spacing
                   and near.contains(LineString([pos, pts[0]])))
        if connect:
            cur.extend(pts)
        else:
            cur = pts
            paths.append(cur)
        pos = pts[-1]
    return paths


# --------------------------------------------------------------------------- emitting
def fmt(v):
    s = f"{v:.3f}".rstrip("0").rstrip(".")
    return s if s not in ("-0", "") else "0"


def emit_block(header: list[str], keep: list[str], plan: Plan, st_in: MachineState,
               st_out: MachineState, s: Settings) -> list[str]:
    o = list(header)
    o.append(f"; bridgefix: {len(plan.zones)} zone(s) "
             f"[{', '.join(f'{a:.0f}deg' for a, _ in plan.zones)}], "
             f"max span {max(plan.old_spans):.1f} -> {max(plan.new_spans):.1f} mm\n")
    o += keep
    # retraction state relative to block start (slicers unretract before a feature)
    deficit = 0.0
    z_print = plan.z
    x, y = st_in.x, st_in.y
    z = st_in.z
    if abs(z - z_print) > 1e-4:
        o.append(f"G1 Z{fmt(z_print)} F{fmt(s.travel_f)}\n")
        z = z_print

    def travel(tx, ty, long):
        nonlocal deficit, x, y
        if long:
            if deficit < s.retract_len:
                o.append(f"G1 E-{fmt(s.retract_len - deficit)} F{fmt(s.retract_f)}\n")
                deficit = s.retract_len
            o.append(f"G1 Z{fmt(z_print + s.z_hop)} F{fmt(s.travel_f)}\n")
            o.append(f"G1 X{fmt(tx)} Y{fmt(ty)} F{fmt(s.travel_f)}\n")
            o.append(f"G1 Z{fmt(z_print)}\n")
        else:
            o.append(f"G1 X{fmt(tx)} Y{fmt(ty)} F{fmt(s.travel_f)}\n")
        x, y = tx, ty

    for path in plan.paths:
        sx, sy = path[0]
        dist = math.hypot(sx - x, sy - y)
        travel(sx, sy, dist > 2.0)
        if deficit > 0:
            o.append(f"G1 E{fmt(deficit)} F{fmt(s.retract_f)}\n")
            deficit = 0.0
        o.append(f"G1 F{fmt(plan.f_print)}\n")
        for px, py in path[1:]:
            L = math.hypot(px - x, py - y)
            if L < 1e-4:
                continue
            o.append(f"G1 X{fmt(px)} Y{fmt(py)} E{plan.e_per_mm * L:.5f}\n")
            x, y = px, py

    # restore the state the rest of the file expects
    tx, ty = st_out.x, st_out.y
    if math.hypot(tx - x, ty - y) > 1e-3:
        travel(tx, ty, True)
    if True:  # always re-assert Z for the next feature
        o.append(f"G1 Z{fmt(st_out.z)}\n")
    target = max(0.0, st_out.deficit - st_in.deficit)
    if abs(deficit - target) > 1e-5:
        delta = deficit - target
        o.append(f"G1 E{fmt(delta) if delta > 0 else '-' + fmt(-delta)} F{fmt(s.retract_f)}\n")
    if st_out.f:
        o.append(f"G1 F{fmt(st_out.f)}\n")
    return o


# --------------------------------------------------------------------------- driver
@dataclass
class BlockReport:
    line_no: int
    z: float
    rewritten: bool
    reason: str
    old_max: float = 0.0
    new_max: float = 0.0
    zones: list = field(default_factory=list)
    plan: Plan | None = None
    moves: list = field(default_factory=list)


def process_gcode(text: str, s: Settings) -> tuple[str, list[BlockReport]]:
    s = settings_from_config(text, s)
    lines = text.splitlines(keepends=True)
    out, reports = [], []
    st = MachineState()
    i, n = 0, len(lines)
    width = 0.42
    while i < n:
        line = lines[i]
        m = FEATURE_RE.match(line)
        if not (m and m.group(1) in s.features):
            lw = LINE_WIDTH_RE.match(line)
            if lw:
                width = float(lw.group(1))
            step(st, line)
            out.append(line)
            i += 1
            continue
        # collect the block
        j = i + 1
        while j < n and not FEATURE_RE.match(lines[j]) and not LAYER_RE.match(lines[j]):
            j += 1
        block = lines[i:j]
        st_in = MachineState(**vars(st))
        moves = []
        widths = []
        for bl in block:
            lw = LINE_WIDTH_RE.match(bl)
            if lw:
                widths.append(float(lw.group(1)))
            step(st, bl, moves)
        st_out = MachineState(**vars(st))
        bw = float(np.median(widths)) if widths else width
        rep = BlockReport(i + 1, st_in.z, False, "", moves=moves)
        try:
            plan = plan_block(moves, bw, s)
        except Exception as ex:  # never break a print file because planning failed
            plan, rep.reason = None, f"planning error: {ex!r}"
        if plan is not None:
            rep.plan, rep.z = plan, plan.z
            rep.old_max, rep.new_max = max(plan.old_spans), max(plan.new_spans)
            rep.zones = [a for a, _ in plan.zones]
            if s.force or rep.new_max < s.min_improvement * rep.old_max:
                header, keep = [], []
                seen_move = False
                for bl in block:
                    cmd, _ = parse_words(bl)
                    if cmd in ("G0", "G1", "G2", "G3", "G92"):
                        seen_move = True
                    elif cmd:
                        keep.append(bl)
                    elif not seen_move and bl.lstrip().startswith(";") and "WIPE" not in bl:
                        header.append(bl)
                out += emit_block(header, keep, plan, st_in, st_out, s)
                rep.rewritten, rep.reason = True, "shorter spans"
            else:
                rep.reason = "already near-optimal"
                out += block
        else:
            rep.reason = rep.reason or "no extrusions"
            out += block
        reports.append(rep)
        i = j
    return "".join(out), reports


def preview(reports: list[BlockReport], path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    reps = [r for r in reports if r.plan is not None]
    if not reps:
        return
    fig, axs = plt.subplots(len(reps), 2, figsize=(14, 6 * len(reps)), squeeze=False)
    for (a0, a1), r in zip(axs, reps):
        for ax in (a0, a1):
            for g in getattr(r.plan.region, "geoms", [r.plan.region]):
                ax.fill(*g.exterior.xy, color="#ddd")
                for h in g.interiors:
                    ax.fill(*h.xy, color="white")
            ax.set_aspect("equal")
        for mv in r.moves:
            if mv.e > 0:
                a0.plot([mv.x0, mv.x1], [mv.y0, mv.y1], "r-", lw=0.6)
        cmap = plt.get_cmap("tab10")
        for k, (ang, zp) in enumerate(r.plan.zones):
            for g in getattr(zp, "geoms", [zp]):
                if g.geom_type == "Polygon":
                    a1.fill(*g.exterior.xy, color=cmap(k % 10), alpha=0.15)
        for p in r.plan.paths:
            xs, ys = zip(*p)
            a1.plot(xs, ys, "b-", lw=0.6)
        a0.set_title(f"line {r.line_no}  Z{r.z:.2f}  ORIGINAL  max span {r.old_max:.1f} mm")
        a1.set_title(f"{'REWRITTEN' if r.rewritten else 'NOT rewritten (' + r.reason + ')'}  "
                     f"max span {r.new_max:.1f} mm  zones {', '.join(f'{a:.0f}' for a in r.zones)}")
    plt.tight_layout()
    fig.savefig(path, dpi=110)


def process_file(src: str, dst: str, s: Settings, preview_path: str | None = None):
    if src.lower().endswith(".3mf"):
        zin = zipfile.ZipFile(src)
        buf = io.BytesIO()
        reports_all = []
        with zipfile.ZipFile(buf, "w") as zout:
            new_md5 = {}
            for info in zin.infolist():
                data = zin.read(info.filename)
                if re.fullmatch(r"Metadata/plate_\d+\.gcode", info.filename):
                    text, reps = process_gcode(data.decode("utf-8"), s)
                    reports_all += reps
                    data = text.encode("utf-8")
                    new_md5[info.filename + ".md5"] = hashlib.md5(data).hexdigest().upper().encode()
                zout.writestr(info, data)
            # md5 files may come before or after their gcode in the archive: rewrite at the end
        out = io.BytesIO()
        with zipfile.ZipFile(io.BytesIO(buf.getvalue())) as zmid, zipfile.ZipFile(out, "w") as zfin:
            for info in zmid.infolist():
                data = new_md5.get(info.filename, zmid.read(info.filename))
                zfin.writestr(info, data)
        with open(dst, "wb") as f:
            f.write(out.getvalue())
        reports = reports_all
    else:
        with open(src, encoding="utf-8") as f:
            text, reports = process_gcode(f.read(), s)
        with open(dst, "w", encoding="utf-8") as f:
            f.write(text)
    if preview_path:
        preview(reports, preview_path)
    return reports


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help=".gcode or .gcode.3mf")
    ap.add_argument("-o", "--output", help="output path (default: overwrite input = post-processing-script mode)")
    ap.add_argument("--preview", help="write a before/after PNG to this path")
    ap.add_argument("--grid", type=float, default=0.8, help="zone grid size in mm (default 0.8)")
    ap.add_argument("--angle-step", type=float, default=15.0, help="direction resolution in degrees (default 15)")
    ap.add_argument("--anchor", type=float, default=0.0, help="extra line extension into anchors, mm")
    ap.add_argument("--internal", action="store_true", help="also rewrite 'Internal Bridge' blocks")
    ap.add_argument("--force", action="store_true", help="rewrite even if the gain is small")
    a = ap.parse_args(argv)
    s = Settings(grid=a.grid, angles_deg=a.angle_step, anchor=a.anchor, force=a.force,
                 features=("Bridge", "Internal Bridge") if a.internal else ("Bridge",))
    reps = process_file(a.input, a.output or a.input, s, a.preview)
    for r in reps:
        print(f"line {r.line_no:>7}  Z{r.z:6.2f}  {'REWRITTEN' if r.rewritten else 'kept     '}  "
              f"max span {r.old_max:6.1f} -> {r.new_max:5.1f} mm  {r.reason}", file=sys.stderr)


if __name__ == "__main__":
    main()
