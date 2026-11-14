"""Render pocket_test runs to GIFs. Usage (from rb2301/src/rb2301_fp): python3 ../../../media/fp/make_gifs.py ../../../media/fp"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon

sys.path.insert(0, os.getcwd())
import pocket_test as pt

OUT = sys.argv[1]
os.makedirs(OUT, exist_ok=True)
FPS = 15


def phase_text(f):
    c = f["corridor"]
    if c.get("board_phase"):
        return "BOARDS " + c["board_phase"].split(" (")[0]
    if c.get("gap_phase"):
        return "GATE " + c["gap_phase"]
    if c.get("locked"):
        side = "left" if f["w"] > 0 else "right" if f["w"] < 0 else ""
        return f"obstacle: turn {side} (locked)" + ("  BACKING UP" if c.get("backing") else "")
    return f"clear: {c['goal']}" if c.get("goal") else "clear"


def render(name, scn, title, every=3, swing=False, bounds=None):
    pt.SWING = swing
    frames = pt.simulate(scn)
    picks = list(range(0, len(frames), every))
    if picks[-1] != len(frames) - 1:
        picks.append(len(frames) - 1)
    picks += [len(frames) - 1] * FPS  # hold the last frame for a second

    fig, ax = plt.subplots(figsize=(6.4, 6.4), dpi=72)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.93, bottom=0.06)
    pt.draw_world(ax, scn, title)
    xs, ys = [f["x"] for f in frames], [f["y"] for f in frames]
    if bounds is None:
        pad = 0.8
        cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        half = max(max(xs) - min(xs), max(ys) - min(ys)) / 2 + pad
        bounds = (cx - half, cx + half, cy - half, cy + half)
    ax.set_xlim(bounds[0], bounds[1])
    ax.set_ylim(bounds[2], bounds[3])

    path_line, = ax.plot([], [], "b-", lw=1)
    scan = ax.scatter([], [], s=3, c="gray")
    hits = ax.scatter([], [], s=8, c="red")
    corridor = Polygon(pt.corridor_polygon(0, 0, 0), closed=True, fc="orange", ec="darkorange", alpha=0.2)
    body = Polygon(pt.footprint(0, 0, 0), closed=True, fc="steelblue", ec="k", alpha=0.9)
    ax.add_patch(corridor)
    ax.add_patch(body)
    moving = LineCollection([], colors="purple", linewidths=3)
    ax.add_collection(moving)
    walls = LineCollection([], colors="darkorange", linewidths=4, linestyles=":")
    ax.add_collection(walls)
    gap_line, = ax.plot([], [], color="limegreen", lw=5, alpha=0.8)
    text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", family="monospace", fontsize=9,
                   bbox=dict(fc="white", alpha=0.85))

    def draw(k):
        i = picks[k]
        f = frames[i]
        path_line.set_data(xs[: i + 1], ys[: i + 1])
        scan.set_offsets(f["world_pts"])
        hits.set_offsets(f["world_pts"][f["inside"]] if len(f["world_pts"]) else f["world_pts"])
        corridor.set_xy(pt.corridor_polygon(f["x"], f["y"], f["h"]))
        body.set_xy(pt.footprint(f["x"], f["y"], f["h"]))
        moving.set_segments([[p, q] for p, q in f["moving"]])
        walls.set_segments([[tuple(a), tuple(b)] for a, b in f["corridor"].get("virtual_walls", [])])
        c = f["corridor"]
        if c.get("gap_center") is not None and c.get("gap_normal") is not None:
            ctr, n = c["gap_center"], c["gap_normal"]
            tx, ty = -n[1] * c["gap_width"] / 2, n[0] * c["gap_width"] / 2
            gap_line.set_data([ctr[0] - tx, ctr[0] + tx], [ctr[1] - ty, ctr[1] + ty])
        else:
            gap_line.set_data([], [])
        status = "  FINISHED" if f.get("finished") else ""
        text.set_text(f"t={f['t']:5.1f}s  v={f['v']:.2f} m/s{status}\n{phase_text(f)}")
        return []

    anim = FuncAnimation(fig, draw, frames=len(picks), blit=False)
    path = os.path.join(OUT, f"{name}.gif")
    anim.save(path, writer=PillowWriter(fps=FPS))
    plt.close(fig)
    last = frames[-1]
    print(f"{path}: {len(picks)} frames, run {last['t']:.0f}s, "
          f"{'finished' if last.get('finished') else 'collision' if last['collision'] else 'ended'}, "
          f"{os.path.getsize(path) / 1e6:.1f} MB", flush=True)


render("fp_full_course", pt.randomised_fp_scenario(2),
       "Full course (random seed 2, swinging boards)", every=4, swing=True)
render("fp_zigzag", pt.SCENARIOS["zigzag"], "Narrow zigzag corridor", every=4)
render("fp_pocket", pt.SCENARIOS["pocket"], "Reverse-C pocket ahead: turn away and go round it", every=3)
