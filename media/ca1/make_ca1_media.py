"""CA1 figure: what each controller sees and picks, on a generated can field.

Runs the real ObstacleAvoidanceNode methods (the node is created but never spun) on a fake 720-beam scan
(the real robot's LiDAR; beam 0 = straight ahead, counter-clockwise), ray-cast against a can layout from
obstacle_generator.generate_maze().
Usage (ROS sourced, from the repo root):  python3 media/ca1/make_ca1_media.py
"""
import math
import os
import sys
import types

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Wedge, Circle

HERE = os.path.dirname(os.path.realpath(__file__))
SRC = os.path.join(HERE, "..", "..", "rb2301", "src")
sys.path.insert(0, os.path.join(SRC, "rb2301_ca1"))
sys.path.insert(0, os.path.join(SRC, "rb2301_ca1", "rb2301_ca1"))
import obstacle_generator as og
import rclpy
from rb2301_ca1 import obstacle_avoidance as ca1

CAN_R = 0.03
N_BEAMS = 720
SEED = 0


def cans(seed):
    maze = og.generate_maze(random_seed=seed)
    return [(x * 2 / og.size_div, (y - og.width / 2) / og.size_div) for x, y in zip(*np.nonzero(maze))]


def scan(can_xy, rx, ry, max_range=10.0):
    '''Ranges for beams k = 0..N-1 at robot-frame bearing 2*pi*k/N (0 = ahead, counter-clockwise).'''
    ang = 2 * math.pi * np.arange(N_BEAMS) / N_BEAMS
    dx, dy = np.cos(ang), np.sin(ang)
    out = np.full(N_BEAMS, np.inf)
    for cx, cy in can_xy:
        ox, oy = cx - rx, cy - ry
        b = dx * ox + dy * oy
        disc = b * b - (ox * ox + oy * oy - CAN_R ** 2)
        t = b - np.sqrt(np.maximum(disc, 0.0))
        hit = (disc > 0) & (t > 0.05) & (t < out)
        out = np.where(hit, t, out)
    out[out > max_range] = np.inf
    return out


def nhien_gaps(raw360):
    '''The gap search of ObstacleAvoidanceNode.nhien(), step for step, returning what it found (for drawing).'''
    s = np.clip(raw360.copy(), 0, 20)
    copy = s.copy()

    def boundary_distance(deg):
        return 20.0 * 4.0 ** (-(deg - 90.0) / 30.0)
    open_runs = []
    for lo, hi in ((0, len(s) / 2), (None, len(s))):
        ws = 0 if lo == 0 else ws
        while ws <= hi - 90:
            we = ws + 90
            if np.all(s[ws:we] == 20):
                while we < hi and s[we] == 20:
                    we += 1
                s[ws:we] = boundary_distance(we - ws)
                open_runs.append((ws, we))
                ws = we
                continue
            ws += 1
    s[100:260:5] = 0.1
    s = np.clip(np.concatenate((s, s)), 0, 20)
    gaps, best, best_score, ws = [], None, 0, 0
    while ws <= len(s) - 20:
        we = ws + 20
        if np.all(s[ws:we] >= 0.5):
            while we < len(s) and s[we] >= 0.5:
                we += 1
            mean = float(np.mean(s[ws:we]))
            if we - ws >= 20 and mean >= 2:
                score = (we - ws + mean) / 2
                gaps.append((ws, we, mean, score))
                if score > best_score:
                    best_score, best = score, ws + (we - ws) / 2
            s[ws:we] = mean
            ws = we
            continue
        ws = we
    return copy, open_runs, gaps, best


def robot_patch(ax):
    ax.add_patch(Polygon([(0.12, 0.115), (0.12, -0.115), (-0.12, -0.115), (-0.12, 0.115)], closed=True, fc="steelblue", ec="k", zorder=6))


def main():
    rclpy.init()
    node = ca1.ObstacleAvoidanceNode()
    sent = {}
    node.move_2D = lambda x=0.0, y=0.0, turn=0.0: sent.update(x=x, y=y, turn=turn)
    field = cans(SEED)

    # A spot inside the field where max is avoiding (front blocked) and finds a gap target (it often finds none and sidesteps)
    pick = None
    for rx in np.arange(0.6, 6.0, 0.1):
        for ry in np.arange(-0.8, 0.81, 0.1):
            if any(math.hypot(cx - rx, cy - ry) < 0.2 for cx, cy in field):
                continue
            s = scan(field, rx, ry)
            pts, idx, ang, rng = node.get_front_scan(s[::4])
            blocked = np.any((pts[:, 0] > 0) & (pts[:, 0] < node.stop_distance) & (np.abs(pts[:, 1]) < 0.12))
            if blocked and node.choose_gap(node.find_gap_targets(pts, idx)) is not None:
                pick = (rx, ry, s)
                break
        if pick:
            break
    rx, ry, s = pick
    rel = [(cx - rx, cy - ry) for cx, cy in field if math.hypot(cx - rx, cy - ry) < 2.6]

    fig, axs = plt.subplots(1, 2, figsize=(13, 6.6), dpi=90)
    fig.subplots_adjust(left=0.05, right=0.99, top=0.86, bottom=0.08, wspace=0.12)

    # ---- nhien: whole-scan gap scoring
    ax = axs[0]
    raw360 = s[::2]
    node.last_scan = raw360.copy()
    node.tempo_blind_left = node.tempo_blind_right = 0
    node.nhien()
    copy, open_runs, gaps, best = nhien_gaps(raw360)
    # The scan is searched twice over (so a gap can wrap past straight ahead): the same gap can be found in both
    # copies. Draw each gap once, keeping the higher score.
    unique = []
    for g in sorted(gaps, key=lambda g: -g[3]):
        span = set(int(a) % 360 for a in range(g[0], g[1]))
        if all(not span & set(int(a) % 360 for a in range(u[0], u[1])) for u in unique):
            unique.append(g)
    for ws, we, mean, score in unique:
        a0, a1 = ws % 360, we % 360 if we % 360 > ws % 360 else we % 360 + 360
        ax.add_patch(Wedge((0, 0), min(mean, 2.4), a0, a1, fc="tab:green", alpha=0.18, ec="tab:green", zorder=1))
        mid = math.radians((ws + we) / 2)
        ax.text(1.0 * math.cos(mid), 1.0 * math.sin(mid), f"{score:.0f}", fontsize=8, color="darkgreen", ha="center", zorder=5)
    ax.add_patch(Wedge((0, 0), 0.6, 100, 260, fc="none", ec="gray", hatch="///", lw=0.5, zorder=1))
    ax.text(-0.45, 0.0, "back\nmasked", fontsize=8, color="dimgray", ha="center", va="center", zorder=5)
    for cx, cy in rel:
        ax.add_patch(Circle((cx, cy), CAN_R * 2.5, fc="saddlebrown", zorder=3))
    k = np.isfinite(raw360)
    ang = np.radians(np.arange(360))[k]
    ax.scatter(raw360[k] * np.cos(ang), raw360[k] * np.sin(ang), s=6, c="k", zorder=4)
    d = math.atan2(sent["y"], sent["x"])
    ax.annotate("", xy=(1.6 * math.cos(d), 1.6 * math.sin(d)), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", lw=3, color="crimson"), zorder=7)
    robot_patch(ax)
    ax.set_title("nhien: score every open gap all round; drive toward the best", fontsize=10)
    ax.text(0.02, 0.98, f"gap = run of >= 20 readings, all >= 0.5 m\nscore = (width deg + mean m) / 2, needs mean >= 2 m\n"
                        f"best gap centre {best % 360:.0f} deg -> command x={sent['x']:+.2f}, y={sent['y']:+.2f} m/s\n"
                        f"(arrow: the command the real nhien() sent)",
            transform=ax.transAxes, va="top", fontsize=8.5, family="monospace", bbox=dict(fc="white", alpha=0.9), zorder=8)

    # ---- max: front gaps between cans
    ax = axs[1]
    pts, idx, angs, rngs = node.get_front_scan(s[::4])
    targets = node.find_gap_targets(pts, idx)
    target = node.choose_gap(targets)
    ax.add_patch(Wedge((0, 0), node.look_ahead, -90, 90, fc="lightyellow", ec="goldenrod", ls="--", zorder=0))
    ax.add_patch(Polygon([(0, 0.12), (node.stop_distance, 0.12), (node.stop_distance, -0.12), (0, -0.12)], closed=True,
                         fc="orange", alpha=0.3, ec="darkorange", zorder=1))
    for cx, cy in rel:
        ax.add_patch(Circle((cx, cy), CAN_R * 2.5, fc="saddlebrown", alpha=0.35, zorder=2))
    colours = plt.cm.tab10(np.linspace(0, 1, 10))
    for k_, cl in enumerate(node.debug_clusters):
        ax.scatter(cl[:, 0], cl[:, 1], s=14, color=colours[k_ % 10], zorder=4)
    for right_edge, left_edge, mid in node.debug_candidate_gaps:
        ax.plot([right_edge[0], left_edge[0]], [right_edge[1], left_edge[1]], color="tab:green", lw=2, zorder=5)
        ax.plot(*mid, "o", color="tab:green", ms=7, zorder=5)
    if target is not None:
        ax.annotate("", xy=tuple(target), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", lw=3, color="crimson"), zorder=7)
    robot_patch(ax)
    ax.set_title("max: thread between the cans ahead", fontsize=10)
    ax.text(0.02, 0.98, f"orange: front strip blocked -> avoid\ncans = points within 12 cm of each other (colours)\n"
                        f"green: gaps wide enough with a clear swept path ({len(targets)})\n"
                        f"arrow: choose_gap() pick (straight-ahead gap first)",
            transform=ax.transAxes, va="top", fontsize=8.5, family="monospace", bbox=dict(fc="white", alpha=0.9), zorder=8)

    for ax in axs:
        ax.set_aspect("equal")
        ax.set_xlim(-1.0, 2.6)
        ax.set_ylim(-1.8, 1.8)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("forward (m)")
        ax.set_ylabel("left (m)")
    fig.suptitle(f"CA1 controllers on the same spot of a generated can field (seed {SEED}, robot at x={rx:.1f}, y={ry:+.1f}; "
                 f"720-beam scan as on the real robot)", fontsize=10.5)
    path = os.path.join(HERE, "ca1_controllers.png")
    fig.savefig(path, dpi=90)
    print(f"{path}: {os.path.getsize(path) / 1e3:.0f} kB; nhien sent {sent}, nhien best gap {best}, max target {target}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
