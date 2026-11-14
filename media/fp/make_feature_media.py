"""Images and short clips that illustrate each FP feature, made with pocket_test and the real controller.

Usage (from rb2301/src/rb2301_fp):  python3 ../../../media/fp/make_feature_media.py ../../../media/fp/features [name ...]
"""
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon

sys.path.insert(0, os.getcwd())
import pocket_test as pt
from rb2301_fp import obstacle_course as oc

OUT = sys.argv[1]
ONLY = set(sys.argv[2:])
os.makedirs(OUT, exist_ok=True)
FPS = 12
DPI = 80


# ---------------------------------------------------------------- shared drawing
def to_world(pts, x, y, h):
    c, s = math.cos(h), math.sin(h)
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    return np.column_stack((x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]))


def base_axes(scn, title, bounds, size=(6.4, 6.4)):
    fig, ax = plt.subplots(figsize=size, dpi=DPI)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.93, bottom=0.06)
    for (px, py), (qx, qy) in scn["segments"]:
        ax.plot([px, qx], [py, qy], color="saddlebrown", lw=3, zorder=1)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(bounds[0], bounds[1])
    ax.set_ylim(bounds[2], bounds[3])
    return fig, ax


def draw_robot(ax, x, y, h, corridor=True):
    if corridor:
        ax.add_patch(Polygon(pt.corridor_polygon(x, y, h), closed=True, fc="orange", ec="darkorange", alpha=0.2, zorder=2))
    ax.add_patch(Polygon(pt.footprint(x, y, h), closed=True, fc="steelblue", ec="k", alpha=0.9, zorder=5))


def caption(ax, text):
    ax.text(0.02, 0.98, text, transform=ax.transAxes, va="top", family="monospace", fontsize=9,
            bbox=dict(fc="white", alpha=0.85), zorder=10)


def save_png(fig, name):
    path = os.path.join(OUT, f"{name}.png")
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    print(f"{path}: {os.path.getsize(path) / 1e3:.0f} kB", flush=True)


def scan_at(scn, x, y, h_deg, n=360):
    return pt.raycast_scan(scn["segments"], x, y, math.radians(h_deg), n)


def wanted(name):
    return not ONLY or name in ONLY


# ---------------------------------------------------------------- clip renderer
def phase_text(f):
    c = f["corridor"]
    if c.get("board_phase"):
        return "BOARDS " + c["board_phase"]
    if c.get("gap_phase"):
        return f"GATE {c['gap_phase']}  (off centre {c['gap_lateral']:+.2f} m)"
    if c.get("deadend_added"):
        return "DEAD END: virtual wall added"
    if c.get("locked"):
        side = "left" if c.get("lock_dir", f["w"]) > 0 else "right"
        extra = ("  lock handed over" if c.get("lock_handover") else "") + ("  BACKING UP" if c.get("backing") else "")
        return f"obstacle {c['nearest']:.2f} m: turn {side} (locked){extra}"
    if c.get("lock_released"):
        return "obstacle left the corridor: lock released"
    return f"clear: {c['goal']}" if c.get("goal") else "clear"


def clip(name, scn, title, t0, t1, every=2, swing=False, bounds=None, cones=False, boards=False, gate=False):
    pt.SWING = swing
    frames = pt.simulate(scn)
    picks = [i for i in range(len(frames)) if t0 <= frames[i]["t"] <= t1][::every]
    picks += [picks[-1]] * FPS
    xs, ys = [f["x"] for f in frames], [f["y"] for f in frames]
    if bounds is None:
        sx, sy = [xs[i] for i in picks], [ys[i] for i in picks]
        cx, cy = (min(sx) + max(sx)) / 2, (min(sy) + max(sy)) / 2
        half = max(max(sx) - min(sx), max(sy) - min(sy)) / 2 + 1.0
        bounds = (cx - half, cx + half, cy - half, cy + half)
    fig, ax = base_axes(scn, title, bounds)
    path_line, = ax.plot([], [], "b-", lw=1, zorder=3)
    scan = ax.scatter([], [], s=3, c="gray", zorder=3)
    hits = ax.scatter([], [], s=10, c="red", zorder=4)
    corridor = Polygon(pt.corridor_polygon(0, 0, 0), closed=True, fc="orange", ec="darkorange", alpha=0.2, zorder=2)
    body = Polygon(pt.footprint(0, 0, 0), closed=True, fc="steelblue", ec="k", alpha=0.9, zorder=5)
    cone_l = Polygon(pt.cone_polygon(0, 0, 0, +1, 0.1), closed=True, fc="green", alpha=0.0, zorder=2)
    cone_r = Polygon(pt.cone_polygon(0, 0, 0, -1, 0.1), closed=True, fc="green", alpha=0.0, zorder=2)
    for p in (corridor, body, cone_l, cone_r):
        ax.add_patch(p)
    moving = LineCollection([], colors="purple", linewidths=3, zorder=4)
    walls = LineCollection([], colors="darkorange", linewidths=4, linestyles=":", zorder=4)
    axes_lc = LineCollection([], colors="magenta", linewidths=1.5, linestyles="--", zorder=3)
    for lc in (moving, walls, axes_lc):
        ax.add_collection(lc)
    gap_line, = ax.plot([], [], color="limegreen", lw=5, alpha=0.8, zorder=4)
    gap_axis, = ax.plot([], [], color="limegreen", lw=1, ls="--", zorder=4)
    text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", family="monospace", fontsize=8.5,
                   bbox=dict(fc="white", alpha=0.85), zorder=10)

    def draw(k):
        i = picks[k]
        f = frames[i]
        c = f["corridor"]
        path_line.set_data(xs[: i + 1], ys[: i + 1])
        scan.set_offsets(f["world_pts"])
        hits.set_offsets(f["world_pts"][f["inside"]])
        corridor.set_xy(pt.corridor_polygon(f["x"], f["y"], f["h"]))
        body.set_xy(pt.footprint(f["x"], f["y"], f["h"]))
        moving.set_segments([[p, q] for p, q in f["moving"]])
        walls.set_segments([[tuple(a), tuple(b)] for a, b in c.get("virtual_walls", [])])
        if cones and c.get("cone_left") is not None:
            for patch, side in ((cone_l, +1), (cone_r, -1)):
                depth = c["cone_left"] if side > 0 else c["cone_right"]
                ang = c["cone_left_angle"] if side > 0 else c["cone_right_angle"]
                patch.set_xy(pt.cone_polygon(f["x"], f["y"], f["h"], side, min(depth, 2.0), ang, oc.SIDE_SUBCONE_HALF_WIDTH_DEG))
                turning = c.get("locked") and np.sign(f["w"]) == side
                patch.set_facecolor("green" if turning else "gray")
                patch.set_alpha(0.35 if turning else 0.15)
        else:
            cone_l.set_alpha(0.0)
            cone_r.set_alpha(0.0)
        if boards:
            axes_lc.set_segments([[tuple(p0), tuple(p1)] for p0, p1 in c.get("board_axes", [])])
        if gate and c.get("gap_center") is not None:
            ctr, n = c["gap_center"], c["gap_normal"]
            tx, ty = -n[1] * c["gap_width"] / 2, n[0] * c["gap_width"] / 2
            gap_line.set_data([ctr[0] - tx, ctr[0] + tx], [ctr[1] - ty, ctr[1] + ty])
            gap_axis.set_data([ctr[0] - n[0] * 1.0, ctr[0] + n[0] * 0.6], [ctr[1] - n[1] * 1.0, ctr[1] + n[1] * 0.6])
        else:
            gap_line.set_data([], [])
            gap_axis.set_data([], [])
        text.set_text(f"t={f['t']:5.1f}s  v={f['v']:+.2f} m/s  turn={f['w']:+.2f} rad/s\n{phase_text(f)}")
        return []

    anim = FuncAnimation(fig, draw, frames=len(picks), blit=False)
    path = os.path.join(OUT, f"{name}.gif")
    anim.save(path, writer=PillowWriter(fps=FPS))
    plt.close(fig)
    print(f"{path}: {len(picks)} frames, {os.path.getsize(path) / 1e6:.2f} MB", flush=True)
    return frames


# ---------------------------------------------------------------- 1. preprocessing: beam order vs bearing order
if wanted("preprocess_bearing"):
    scn = pt.randomised_fp_scenario(2)
    x, y, h = 3.0, -2.75, 0.0
    msg = scan_at(scn, x, y, h)
    pts, _ = oc.scan_to_robot_points(msg)
    near = pts[np.hypot(pts[:, 0], pts[:, 1]) < 2.5]
    raw_rank = np.arange(len(near)) / len(near)
    bearing = np.arctan2(near[:, 1], near[:, 0] - oc.LASER_X_OFFSET)
    sorted_rank = np.argsort(np.argsort(bearing)) / len(near)
    fig, axs = plt.subplots(1, 2, figsize=(10, 5), dpi=DPI)
    fig.subplots_adjust(left=0.05, right=0.98, top=0.86, bottom=0.08, wspace=0.12)
    for ax, rank, title in ((axs[0], raw_rank, "Raw beam order: starts straight ahead,\nso the front is split in two"),
                            (axs[1], sorted_rank, "Sorted by bearing (-180 to 180 deg):\ncontinuous across the front")):
        w = to_world(near, x, y, h)
        ax.scatter(w[:, 0], w[:, 1], c=rank, cmap="turbo", s=10)
        ax.add_patch(Polygon(pt.footprint(x, y, h), closed=True, fc="steelblue", ec="k"))
        ax.annotate("", xy=(x + 0.6, y), xytext=(x, y), arrowprops=dict(arrowstyle="->", lw=1.5))
        ax.set_aspect("equal")
        ax.set_xlim(x - 2.0, x + 2.2)
        ax.set_ylim(y - 1.6, y + 1.6)
        ax.set_title(title, fontsize=10)
        ax.grid(True, alpha=0.3)
    fig.suptitle("Scan points coloured by their position in the list (blue = first, red = last)", fontsize=11)
    save_png(fig, "preprocess_bearing")

# ---------------------------------------------------------------- 2. front corridor: obstacle extraction
if wanted("corridor_extraction"):
    segs = pt.arena(5.0, 5.0) + [((0.35, 0.165), (1.0, 0.165))] + pt.box(1.15, -0.06, 0.2, 0.2) \
        + pt.box(1.75, 0.12, 0.25, 0.25) + pt.circle(2.3, -0.9, 0.2)
    scn = {"segments": segs}
    msg = scan_at(scn, 0.0, 0.0, 0.0)
    pts, rng = oc.scan_to_robot_points(msg)
    cor = oc.analyse_front_corridor(pts, rng)
    inside = (pts[:, 0] > 0) & (pts[:, 0] <= oc.CORRIDOR_LENGTH) & (np.abs(pts[:, 1]) <= oc.CORRIDOR_HALF_WIDTH)
    fig, ax = base_axes(scn, "Front corridor: obstacles inside the rectangle, nearest one decides", (-0.6, 2.8, -1.4, 1.2),
                        size=(7.2, 5.6))
    ax.scatter(pts[:, 0], pts[:, 1], s=4, c="gray", zorder=3, label="scan points (outside: ignored)")
    draw_robot(ax, 0.0, 0.0, 0.0)
    blocking = np.concatenate(cor["clusters"]) if cor["clusters"] else np.empty((0, 2))
    is_blocking = np.array([np.any(np.all(np.isclose(p, blocking), axis=1)) for p in pts]) if len(blocking) else np.zeros(len(pts), bool)
    sliver = inside & ~is_blocking
    ax.scatter(pts[sliver, 0], pts[sliver, 1], s=18, c="gold", ec="k", lw=0.3, zorder=4, label="edge sliver (< 20% in): ignored")
    others = [cl for cl in cor["clusters"] if not np.array_equal(cl, cor["first_points"])]
    for k, cl in enumerate(others):
        ax.scatter(cl[:, 0], cl[:, 1], s=18, c="mediumpurple", zorder=4, label="other obstacle (not used yet)" if k == 0 else None)
    fp = cor["first_points"]
    ax.scatter(fp[:, 0], fp[:, 1], s=26, c="red", zorder=5, label="nearest obstacle: decides speed and turn")
    k = int(np.argmin(np.hypot(fp[:, 0] - oc.LASER_X_OFFSET, fp[:, 1])))
    ax.plot([oc.LASER_X_OFFSET, fp[k, 0]], [0, fp[k, 1]], "r--", lw=1, zorder=4)
    ax.text(fp[k, 0] - 0.75, fp[k, 1] - 0.42, f"nearest = {cor['nearest']:.2f} m\nlabel: {cor['blocking']}", fontsize=9, color="darkred")
    ax.legend(loc="lower left", fontsize=8, framealpha=0.9)
    save_png(fig, "corridor_extraction")

# ---------------------------------------------------------------- 3. side cones and sub-cones
if wanted("side_cones"):
    segs = pt.arena(8.0, 8.0) + pt.box(0.85, -0.05, 0.3, 0.5) + [((-0.3, 0.8), (3.0, 0.8))] + pt.box(2.3, -1.4, 0.5, 0.3)
    scn = {"segments": segs}
    x, y, hd = 0.0, 0.0, 0.0
    msg = scan_at(scn, x, y, hd)
    corridor = {}
    left, right, goal_side = oc.side_cone_scores(msg, hd, oc.TARGET_HEADING_DEG, corridor, oc.NORTH_DEG)
    fig, ax = base_axes(scn, "Side cones: each side's deepest sub-cone (drawn up to 2.4 m), plus a goal-side bonus", (-0.8, 3.2, -2.0, 2.0),
                        size=(7.0, 7.0))
    pts, _ = oc.scan_to_robot_points(msg)
    ax.scatter(pts[:, 0], pts[:, 1], s=4, c="gray", zorder=3)
    lo = oc.SIDE_CONE_ANGLE_DEG - oc.SIDE_CONE_HALF_WIDTH_DEG + oc.SIDE_SUBCONE_HALF_WIDTH_DEG
    hi = oc.SIDE_CONE_ANGLE_DEG + oc.SIDE_CONE_HALF_WIDTH_DEG - oc.SIDE_SUBCONE_HALF_WIDTH_DEG
    centres = np.arange(lo, hi + 1e-6, oc.SIDE_SUBCONE_STEP_DEG)
    for side, colour in ((+1, "tab:blue"), (-1, "tab:green")):
        ax.add_patch(Polygon(pt.cone_polygon(x, y, 0.0, side, 2.4), closed=True, fill=False, ec=colour, ls="--", lw=1, zorder=2))
        depths = oc._cone_depths(msg, side * centres, oc.SIDE_SUBCONE_HALF_WIDTH_DEG)
        best = int(np.argmax(depths))
        for k, (cdeg, d) in enumerate(zip(side * centres, depths)):
            ax.add_patch(Polygon(pt.cone_polygon(x, y, 0.0, side, min(d, 2.4), cdeg, oc.SIDE_SUBCONE_HALF_WIDTH_DEG), closed=True,
                                 fc=colour, alpha=0.55 if k == best else 0.12, ec=colour if k == best else "none", zorder=2))
        bdeg = math.radians(side * centres[best])
        r = min(depths[best], 2.4) * 0.6
        ax.text(oc.LASER_X_OFFSET + r * math.cos(bdeg), r * math.sin(bdeg), f"{depths[best]:.2f} m", fontsize=10, fontweight="bold",
                color=colour, ha="center", va="center", bbox=dict(fc="white", ec=colour, alpha=0.9), zorder=8)
    draw_robot(ax, x, y, 0.0, corridor=True)
    win = "LEFT" if left > right else "RIGHT"
    caption(ax, f"left  depth {corridor['cone_left']:.2f} m  score {left:.2f}\n"
                f"right depth {corridor['cone_right']:.2f} m  score {right:.2f}\n"
                f"goal side: {'left' if goal_side > 0 else 'right'} (+{oc.SIDE_CONE_GOAL_BONUS:.2f} m)\n"
                f"-> turn {win}")
    ax.annotate("north", xy=(3.05, 1.8), xytext=(2.45, 1.8), arrowprops=dict(arrowstyle="->"), fontsize=9, va="center")
    ax.annotate("east", xy=(2.6, 0.95), xytext=(2.6, 1.6), arrowprops=dict(arrowstyle="->"), fontsize=9, ha="center", va="bottom")
    save_png(fig, "side_cones")

# ---------------------------------------------------------------- 4. turn lock and hand-over (clip)
if wanted("turn_lock"):
    segs = pt.arena(6.0, 5.0) + pt.box(1.0, -0.15, 0.3, 0.3) + pt.box(1.7, 0.45, 0.3, 0.3) + pt.box(2.6, -0.2, 0.35, 0.35)
    scn = {"desc": "", "segments": segs, "start": (0.0, 0.0, 0.0), "max_time": 20}
    clip("turn_lock", scn, "Turn lock: one side per obstacle, held until it leaves the corridor", 0.0, 14.0,
         every=2, cones=True, bounds=(-0.6, 4.4, -2.2, 2.2))

# ---------------------------------------------------------------- 5. side check: sweep zones
if wanted("side_check"):
    gx, gy = np.meshgrid(np.linspace(-0.45, 0.6, 140), np.linspace(-0.45, 0.45, 120))
    grid = np.column_stack((gx.ravel(), gy.ravel()))
    left_zone, right_zone, _ = oc._sweep_zones(grid, oc.TURN_CHECK_RADIUS, oc.REAR_SIDE_CLEARANCE)
    fig, axs = plt.subplots(1, 2, figsize=(10, 4.6), dpi=DPI)
    fig.subplots_adjust(left=0.05, right=0.98, top=0.84, bottom=0.08, wspace=0.12)
    for ax, zone, title, colour in ((axs[0], left_zone, "Turning LEFT sweeps front-left + rear-right", "tab:blue"),
                                    (axs[1], right_zone, "Turning RIGHT sweeps front-right + rear-left", "tab:green")):
        ax.scatter(grid[zone, 0], grid[zone, 1], s=2, c=colour, alpha=0.5)
        ax.add_patch(Polygon(pt.footprint(0, 0, 0), closed=True, fc="steelblue", ec="k"))
        ax.plot(0, 0, "k+", ms=10)
        ax.annotate("front", xy=(0.45, 0), xytext=(0.25, 0), arrowprops=dict(arrowstyle="->"), fontsize=9, va="center")
        ax.set_aspect("equal")
        ax.set_xlim(-0.45, 0.6)
        ax.set_ylim(-0.45, 0.45)
        ax.set_title(title, fontsize=10)
        ax.grid(True, alpha=0.3)
    fig.suptitle("Side check: a point in the turn's zone -> turn away from it until clear (+ = rear axle)", fontsize=11)
    save_png(fig, "side_check")

# ---------------------------------------------------------------- 6. back-up and dead-end memory (clip)
if wanted("deadend"):
    a = math.radians(-45.0)
    d, n = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))

    def P(s, t):
        return (-0.5 * d[0] + s * d[0] + t * n[0], -0.5 * d[1] + s * d[1] + t * n[1])
    pocket = [(P(0, 0.3), P(1.2, 0.3)), (P(1.2, 0.3), P(1.2, -0.3)), (P(1.2, -0.3), P(0, -0.3))]  # 0.6 m wide dead end
    # Arena walls kept well away from the dead end's mouth (pt.arena() always has its left wall at x = -0.5)
    scn = {"desc": "", "segments": pt.box(0.5, 0.0, 6.0, 6.0) + pocket, "start": (0.0, 0.0, -45.0), "max_time": 48}
    clip("deadend", scn, "Dead end: back up, virtual wall after 5 back-ups, then drive out", 0.0, 46.0,
         every=3, bounds=(-1.6, 3.2, -2.4, 2.4))

# ---------------------------------------------------------------- 7. corridor clear: heading layers
if wanted("heading_layers"):
    segs = pt.arena(5.0, 5.0) + [((0.5, -0.2), (0.5, -1.6)), ((0.5, -0.2), (1.4, -0.2))]
    scn = {"segments": segs}
    x, y, hd = 0.0, 0.0, 70.0
    msg = scan_at(scn, x, y, hd)
    target, label = oc.choose_goal_heading(msg, hd, oc.TARGET_HEADING_DEG, oc.NORTH_DEG, {})
    fig, ax = base_axes(scn, "Corridor clear: three heading layers (here layer 3, a detour)", (-1.6, 1.8, -1.9, 1.5),
                        size=(6.8, 6.8))
    world = np.arange(-180.0, 180.0, oc.HEADING_CANDIDATE_STEP_DEG)
    depths = oc._cone_depths(msg, [oc.wrap_deg(w - hd) for w in world])
    lx, ly = x + oc.LASER_X_OFFSET * math.cos(math.radians(hd)), y + oc.LASER_X_OFFSET * math.sin(math.radians(hd))
    for w, dep in zip(world, depths):
        # One cone per candidate direction (+-HEADING_CONE_HALF_DEG, median range); drawn up to 1.3 m
        col = "tab:green" if dep >= oc.HEADING_OPEN_DEPTH else "tab:red"
        ax.add_patch(Polygon(pt.cone_polygon(x, y, math.radians(hd), +1, min(dep, 1.3), oc.wrap_deg(w - hd), oc.HEADING_CONE_HALF_DEG),
                             closed=True, fc=col, ec="white", lw=0.8, alpha=0.45, zorder=2))
    t = math.radians(oc.TARGET_HEADING_DEG)
    half = math.radians(oc.HEADING_ON_COURSE_DEG / 2)
    ax.add_patch(Polygon([(x, y)] + [(x + 1.5 * math.cos(a_), y + 1.5 * math.sin(a_)) for a_ in np.linspace(t - half, t + half, 20)],
                         closed=True, fc="gold", alpha=0.15, ec="goldenrod", ls="--", zorder=1))
    ax.annotate("", xy=(x + 1.45 * math.cos(t), y + 1.45 * math.sin(t)), xytext=(x, y),
                arrowprops=dict(arrowstyle="->", color="goldenrod", lw=2))
    ax.text(x + 1.1 * math.cos(t) + 0.05, y + 1.1 * math.sin(t) - 0.15, "north-east\n(blocked)", color="darkgoldenrod", fontsize=9)
    ta = math.radians(target)
    ax.annotate("", xy=(x + 1.3 * math.cos(ta), y + 1.3 * math.sin(ta)), xytext=(x, y),
                arrowprops=dict(arrowstyle="-|>", color="k", lw=2.5))
    draw_robot(ax, x, y, math.radians(hd), corridor=False)
    caption(ax, f"heading {hd:+.0f} deg: outside the 110 deg window (layer 1: no)\n"
                f"north-east cone < {oc.HEADING_OPEN_DEPTH:.1f} m deep (layer 2: no)\n"
                f"layer 3 -> {label}\n"
                f"cones every {oc.HEADING_CANDIDATE_STEP_DEG:.0f} deg (+-{oc.HEADING_CONE_HALF_DEG} deg, median range)\n"
                f"green = open (>= {oc.HEADING_OPEN_DEPTH:.1f} m), red = blocked")
    ax.text(1.0, 1.35, "north ->", fontsize=9)
    save_png(fig, "heading_layers")

# ---------------------------------------------------------------- 8. gate detection: the closed-line check
if wanted("gate_detection"):
    room = pt.box(1.0, 0.0, 6.0, 6.0)
    behind = pt.box(2.2, 0.0, 0.3, 0.3) + pt.circle(2.6, 1.3, 0.2) + pt.circle(2.4, -1.2, 0.2)
    wall = [((0.95, 0.2), (0.95, 3.0)), ((0.95, -0.2), (0.95, -3.0)), ((1.05, 0.2), (1.05, 3.0)), ((1.05, -0.2), (1.05, -3.0)),
            ((0.95, 0.2), (1.05, 0.2)), ((0.95, -0.2), (1.05, -0.2))]
    boxes = pt.box(1.0, 0.45, 0.1, 0.5) + pt.box(1.0, -0.45, 0.1, 0.5)
    lidar = np.array((oc.LASER_X_OFFSET, 0.0))
    fig, axs = plt.subplots(1, 2, figsize=(12, 6.0), dpi=DPI)
    fig.subplots_adjust(left=0.05, right=0.98, top=0.88, bottom=0.06, wspace=0.1)
    for ax, segs, title in ((axs[0], room + behind + wall, "Real gate: a gap in a solid wall"),
                            (axs[1], room + behind + boxes, "Fake gate: a gap between two boxes")):
        pts, _ = oc.scan_to_robot_points(pt.raycast_scan(segs, 0.0, 0.0, 0.0, 360))
        found = oc.detect_gaps(pts, 0.0, oc.NORTH_DEG)
        keep = oc._line_closed
        oc._line_closed = lambda *a: True
        candidate = oc.detect_gaps(pts, 0.0, oc.NORTH_DEG)[0]  # what the other checks accept
        oc._line_closed = keep
        a, b, ctr = candidate["a"], candidate["b"], candidate["center"]
        along = (b - a) / np.hypot(*(b - a))
        normal = np.array((-along[1], along[0]))
        if (lidar - ctr) @ normal > 0:
            normal = -normal
        for (px, py), (qx, qy) in segs:
            ax.plot([px, qx], [py, qy], color="saddlebrown", lw=3, zorder=1)
        # Beams to points more than GATE_CLOSED_DEPTH beyond the line: where do they cross it?
        depth = (pts - ctr) @ normal
        bad = good = 0
        for p in pts[depth > oc.GATE_CLOSED_DEPTH]:
            lidar_depth = (lidar - ctr) @ normal
            fr = lidar_depth / (lidar_depth - (p - ctr) @ normal)
            u = abs((lidar + fr * (p - lidar) - ctr) @ along)
            if u <= candidate["width"] / 2 + oc.GATE_CLOSED_EDGE_MARGIN:
                col, good = "tab:green", good + 1
            elif u <= oc.GATE_CLOSED_EXTENT:
                col, bad = "red", bad + 1
            else:
                continue
            ax.plot([lidar[0], p[0]], [lidar[1], p[1]], color=col, lw=0.8, alpha=0.6, zorder=2)
        ax.scatter(pts[:, 0], pts[:, 1], s=5, c="k", zorder=3)
        # The gate line, checked out to GATE_CLOSED_EXTENT from the gap centre, and the depth band
        e0, e1 = ctr - along * oc.GATE_CLOSED_EXTENT, ctr + along * oc.GATE_CLOSED_EXTENT
        ax.plot([e0[0], e1[0]], [e0[1], e1[1]], color="gray", ls="--", lw=1.2, zorder=3)
        band = [e0 + normal * oc.GATE_CLOSED_DEPTH, e1 + normal * oc.GATE_CLOSED_DEPTH]
        ax.plot([band[0][0], band[1][0]], [band[0][1], band[1][1]], color="gray", ls=":", lw=1, zorder=3)
        ok = bool(found)
        ax.plot([a[0], b[0]], [a[1], b[1]], color="limegreen" if ok else "red", lw=5, ls="-" if ok else "--", zorder=5)
        ax.add_patch(Polygon(pt.footprint(0, 0, 0), closed=True, fc="steelblue", ec="k", zorder=5))
        ax.plot([], [], color="tab:green", label=f"beam through the gap: allowed ({good})")
        ax.plot([], [], color="red", label=f"beam past the line outside the gap ({bad})")
        ax.plot([], [], color="gray", ls="--", label=f"gate line, checked to {oc.GATE_CLOSED_EXTENT:.0f} m each side")
        ax.plot([], [], color="gray", ls=":", label=f"{oc.GATE_CLOSED_DEPTH * 100:.0f} cm beyond the line")
        ax.set_aspect("equal")
        ax.set_xlim(-0.5, 3.0)
        ax.set_ylim(-1.8, 1.8)
        ax.grid(True, alpha=0.3)
        ax.set_title(title + (": line closed -> ACCEPTED" if ok else ": line not closed -> REJECTED"), fontsize=10)
        ax.legend(loc="lower left", fontsize=8, framealpha=0.9)
    fig.suptitle("Closed-line check: apart from the gap itself, nothing may be seen beyond the gate line", fontsize=11)
    save_png(fig, "gate_detection")

# ---------------------------------------------------------------- 9. gate manoeuvre (clip)
if wanted("gate_manoeuvre"):
    scn = pt.randomised_fp_scenario(2)
    pt.SWING = True
    probe = pt.simulate(scn)
    t_a = next(f["t"] for f in probe if f["corridor"].get("gap_phase"))
    t_b = next(f["t"] for f in probe if f["corridor"].get("board_phase"))
    clip("gate_manoeuvre", scn, "Gate manoeuvre: APPROACH -> ALIGN -> TURN -> GO", t_a - 1.0, t_b + 1.0,
         every=1, swing=True, gate=True)

# ---------------------------------------------------------------- 10. board crossing (clip)
BOARD_SEED, BOARD_SWING = 16, False  # A course where the board really sweeps across the robot's path while it waits
if wanted("boards"):
    scn = pt.randomised_fp_scenario(BOARD_SEED)
    pt.SWING = BOARD_SWING
    probe = pt.simulate(scn)
    t_a = next(f["t"] for f in probe if f["corridor"].get("board_phase"))
    clip("boards", scn, f"Board crossing (seed {BOARD_SEED}): SCAN, SIDE, FACE, APPROACH, WAIT, CROSS", t_a, probe[-1]["t"],
         every=2, swing=BOARD_SWING, boards=True)
