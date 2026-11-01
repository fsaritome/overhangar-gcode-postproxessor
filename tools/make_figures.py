#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 fsaritome
# Part of overhangar: https://github.com/fsaritome/overhangar-gcode-postproxessor
# Additional terms (AGPL-3.0 section 7(b)): see NOTICE; author attribution must be preserved.
"""Regenerate the README figures from examples/heltec-v4-case.

    pip install -r requirements.txt trimesh lxml
    python tools/make_figures.py
"""
import sys
import zipfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "bridgefix"))
import bridgefix as bf  # noqa: E402

EX = ROOT / "examples" / "heltec-v4-case"
OUT = ROOT / "docs" / "images"
OUT.mkdir(parents=True, exist_ok=True)

RED, BLUE, GREY = "#d1495b", "#2e6fbd", "#e4e4e4"
plt.rcParams.update({"font.size": 11, "axes.titleweight": "bold"})


def geoms(g):
    return list(getattr(g, "geoms", [g]))


def draw_region(ax, region):
    for g in geoms(region):
        if g.geom_type != "Polygon":
            continue
        ax.fill(*g.exterior.xy, color=GREY, lw=0)
        for h in g.interiors:
            ax.fill(*h.xy, color="white", lw=0)


def draw_old(ax, rep, lw):
    for m in rep.moves:
        if m.e > 0:
            ax.plot([m.x0, m.x1], [m.y0, m.y1], color=RED, lw=lw, solid_capstyle="butt")


def draw_new(ax, rep, lw):
    for p in rep.plan.paths:
        if len(p) > 1:
            xs, ys = zip(*p)
            ax.plot(xs, ys, color=BLUE, lw=lw, solid_capstyle="butt")


def style(ax, title, lim=None):
    ax.set_aspect("equal")
    ax.set_title(title)
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    if lim:
        ax.set_xlim(lim[0], lim[1])
        ax.set_ylim(lim[2], lim[3])


def model_render():
    import trimesh
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    m = trimesh.load(EX / "MainBody.3mf").to_geometry()
    light = np.array([0.4, -0.6, 0.9])
    light /= np.linalg.norm(light)
    fig = plt.figure(figsize=(12, 5))
    for i, (el, az, t) in enumerate([(35, -60, "top side"), (-35, -60, "bottom side (on the bed)")]):
        ax = fig.add_subplot(1, 2, i + 1, projection="3d")
        sh = np.clip(np.abs(m.face_normals @ light), 0.2, 1.0)
        col = np.c_[0.25 + 0.5 * sh, 0.45 + 0.45 * sh, 0.65 + 0.35 * sh, np.ones(len(sh))]
        col = np.clip(col, 0, 1)
        ax.add_collection3d(Poly3DCollection(m.triangles, facecolors=col, linewidths=0))
        c, r = m.bounds.mean(0), m.extents.max() / 2
        ax.set_xlim(c[0] - r, c[0] + r)
        ax.set_ylim(c[1] - r, c[1] + r)
        ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(el, az)
        ax.set_axis_off()
        ax.set_title(t)
    fig.suptitle("MainBody: 76 x 52 x 14 mm frame, gasket groove + lips on both faces", fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT / "model.png", dpi=110, bbox_inches="tight")
    plt.close(fig)

    # groove cross-section
    import trimesh.intersections as ti
    seg = ti.mesh_plane(m, (1, 0, 0), (25, 0, 0))
    fig, ax = plt.subplots(figsize=(7, 4.2))
    for s in seg:
        if s[:, 1].max() < -19:
            ax.plot(s[:, 1], s[:, 2], "k-", lw=1.5)
    ax.annotate("groove ceiling\n= bridge at Z 1.16", xy=(-23, -4.56), xytext=(-19.5, -2.2),
                arrowprops=dict(arrowstyle="->"), fontsize=10)
    ax.annotate("lips (on bed)", xy=(-25.75, -5.56), xytext=(-28.8, -3.3),
                arrowprops=dict(arrowstyle="->"), fontsize=10)
    ax.axhline(-5.56, color="#999", lw=0.8, ls="--")
    ax.set_aspect("equal")
    ax.set_xlim(-29, -16)
    ax.set_title("Wall cross-section (bottom groove is ~3 mm wide)")
    ax.set_xlabel("mm")
    ax.set_ylabel("Z (mm)")
    fig.tight_layout()
    fig.savefig(OUT / "groove_section.png", dpi=110)
    plt.close(fig)


def gcode_figures():
    text = zipfile.ZipFile(EX / "HeltecV4CaseOutdoorV2.gcode.3mf").read("Metadata/plate_1.gcode").decode()
    _, reps = bf.process_gcode(text, bf.Settings())
    ring = reps[0]

    fig, axs = plt.subplots(1, 2, figsize=(15, 6.2))
    for ax in axs:
        draw_region(ax, ring.plan.region)
    draw_old(axs[0], ring, 0.35)
    draw_new(axs[1], ring, 0.35)
    style(axs[0], f"Bambu Studio (bridge_angle 90): longest line {ring.old_max:.1f} mm")
    style(axs[1], f"overhangar bridgefix: longest line {ring.new_max:.1f} mm")
    fig.suptitle("Bottom groove ceiling, layer Z 1.16", fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT / "groove_before_after.png", dpi=110)
    plt.close(fig)

    lims = [((90.5, 97, 84, 104), "long side"), ((90.5, 104, 70, 85), "corner around a screw boss")]
    fig, axs = plt.subplots(2, 2, figsize=(11, 12), gridspec_kw={"height_ratios": [1.45, 1]})
    for row, (lim, name) in zip(axs, lims):
        for ax in row:
            draw_region(ax, ring.plan.region)
        draw_old(row[0], ring, 1.6)
        draw_new(row[1], ring, 1.6)
        style(row[0], f"before: {name}", lim)
        style(row[1], f"after: {name}", lim)
    fig.tight_layout()
    fig.savefig(OUT / "groove_detail.png", dpi=110)
    plt.close(fig)

    others = [r for r in reps[1:] if r.rewritten][:3]
    fig, axs = plt.subplots(2, len(others), figsize=(5 * len(others), 9))
    for k, r in enumerate(others):
        for ax in axs[:, k]:
            draw_region(ax, r.plan.region)
        draw_old(axs[0, k], r, 1.2)
        draw_new(axs[1, k], r, 1.2)
        style(axs[0, k], f"before: {r.old_max:.1f} mm")
        style(axs[1, k], f"after: {r.new_max:.1f} mm")
    fig.suptitle("Small bridges at Z 12.04: one direction per island, re-chosen", fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT / "islands_before_after.png", dpi=100)
    plt.close(fig)
    return reps


if __name__ == "__main__":
    model_render()
    for r in gcode_figures():
        print(f"Z{r.z:5.2f} {'rewritten' if r.rewritten else 'kept':9s} {r.old_max:5.1f} -> {r.new_max:4.1f} mm")
