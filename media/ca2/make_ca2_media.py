"""CA2 figures, made with the real planner code in path_planning.py (no ROS node is started).

Usage (ROS sourced, from the repo root):  python3 media/ca2/make_ca2_media.py
"""
import math
import os
import sys
import types

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Polygon, FancyArrowPatch

HERE = os.path.dirname(os.path.realpath(__file__))
PKG = os.path.join(HERE, "..", "..", "rb2301", "src", "rb2301_ca2", "rb2301_ca2")
sys.path.insert(0, PKG)
import path_planning as pp

DPI = 90
cfg = pp.sim_config
grid = np.load(os.path.join(PKG, cfg["map_file"]), allow_pickle=True)
origin, res, goals = cfg["origin"], cfg["resolution"], cfg["goal_list"]
start = (0.0, 0.0)
UP = 4  # map_upsample_factor in WaypointNode.__init__


def preprocessed():
    fake = types.SimpleNamespace(map_array=grid, map_upsample_factor=UP)
    return pp.WaypointNode.preprocess_map(fake)


def build_tree(fine, n_nodes=2000, seed=0):
    '''The same RRT* loop as WaypointNode.timer_callback (seeded so the figure is repeatable).'''
    np.random.seed(seed)
    fres = res / UP
    tree = pp.Tree()
    tree.add_node(pp.TreeNode(pos=start, cost=0, parent_id=-1, id=0, fixed_cost=0))
    while len(tree.nodes) < n_nodes:
        rx = np.random.uniform(origin[0], origin[0] + fine.shape[0] * fres)
        ry = np.random.uniform(origin[1], origin[1] + fine.shape[1] * fres)
        near = tree.NearestNode(rx, ry)
        new_pos = (near.pos[0] + np.clip(rx - near.pos[0], -0.2, 0.2), near.pos[1] + np.clip(ry - near.pos[1], -0.2, 0.2))
        if tree.line_intersects_obstacle(near.pos, new_pos, fine, origin, fres):
            continue
        cell = pp.world_to_grid(new_pos[0], new_pos[1], origin, fres)
        extra = fine[cell[0], cell[1]]
        node = pp.TreeNode(pos=new_pos, cost=near.cost + tree.dist(*near.pos, *new_pos) + extra, fixed_cost=extra, parent_id=near.id)
        tree.add_node(node)
        tree.rewire(node, fine, origin, fres, r=0.2)
    return tree


def routes(tree):
    out = []
    for k, goal in enumerate(goals):
        start_node = tree.nodes[0] if k == 0 else tree.NearestNode(*goals[k - 1])
        out.append(tree.path_to_goal(start_node, tree.NearestNode(*goal)))
    return out


def show_map(ax, fine, title):
    # 0 free, 75 soft ring, 99 hard (original wall or hard ring)
    original = np.repeat(np.repeat(grid, UP, axis=0), UP, axis=1) == 99
    layer = np.where(original, 3, np.where(fine == 99, 2, np.where(fine == 75, 1, 0)))
    cmap = ListedColormap(["white", "#fde3b0", "#e8902c", "#5a3a1a"])
    x1, y1 = origin[0] + grid.shape[0] * res, origin[1] + grid.shape[1] * res
    # Drawn like the code's print_map(): +x up, +y to the left (rows = x, columns = y, y axis reversed)
    ax.imshow(layer, origin="lower", extent=[origin[1], y1, origin[0], x1], cmap=cmap, vmin=0, vmax=3, interpolation="nearest")
    ax.set_xlim(y1, origin[1])
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("y (m)   (+y to the left)")
    ax.set_ylabel("x (m)")


def figure_planner():
    fine = preprocessed()
    print("building the RRT* tree (2000 nodes)...", flush=True)
    tree = build_tree(fine)
    paths = routes(tree)
    fig, axs = plt.subplots(1, 2, figsize=(12, 8.8), dpi=DPI)
    fig.subplots_adjust(left=0.05, right=0.99, top=0.9, bottom=0.14, wspace=0.1)

    show_map(axs[0], fine, f"1. Map preprocessing: {UP}x finer grid + two obstacle rings")
    from matplotlib.patches import Patch
    axs[0].legend(handles=[Patch(fc="#5a3a1a", label="wall (original map)"), Patch(fc="#e8902c", label="hard ring: wall (99)"),
                           Patch(fc="#fde3b0", ec="#e0c080", label="soft ring: allowed, costs +75"), Patch(fc="white", ec="gray", label="free")],
                  loc="lower right", fontsize=8, framealpha=0.95)

    show_map(axs[1], fine, "2-3. RRT* tree (2000 nodes) and the route to each goal")
    for node in tree.nodes:
        parent = tree.getParentNode(node.id)
        if parent is not None:
            axs[1].plot([parent.pos[1], node.pos[1]], [parent.pos[0], node.pos[0]], color="0.55", lw=0.4, zorder=2)
    colours = ["tab:blue", "tab:green", "tab:purple", "tab:red"]
    prev = start
    for k, (path, goal, col) in enumerate(zip(paths, goals, colours)):
        xs, ys = [n.pos[0] for n in path], [n.pos[1] for n in path]
        # Earlier routes thicker and underneath, so shared stretches still show every route
        axs[1].plot(ys, xs, color=col, lw=7.0 - 1.6 * k, alpha=0.9, solid_capstyle="round", zorder=4 + k,
                    label=f"route {k + 1}: {prev} -> {goal}")
        axs[1].plot(goal[1], goal[0], marker="o", ms=11, color=col, mec="k", zorder=5)
        axs[1].text(goal[1] - 0.15, goal[0] + 0.15, str(k + 1), fontsize=11, fontweight="bold", color=col, zorder=6,
                    bbox=dict(fc="white", ec=col, boxstyle="circle,pad=0.15"))
        prev = goal
    axs[1].plot(start[1], start[0], marker="s", ms=10, color="k", zorder=5)
    axs[1].text(start[1] - 0.15, start[0] - 0.3, "start", fontsize=9, zorder=6)
    axs[1].legend(loc="upper center", bbox_to_anchor=(0.5, -0.09), ncol=2, fontsize=8, framealpha=0.95)
    fig.suptitle("CA2 planner on the simulation maze (one tree from the start, reused for every goal)", fontsize=11)
    path = os.path.join(HERE, "ca2_planner.png")
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    print(f"{path}: {os.path.getsize(path) / 1e3:.0f} kB  (tree {len(tree.nodes)} nodes, route lengths {[len(p) for p in paths]})")


def figure_carrot():
    route = [(0.0, 0.0), (0.6, 0.0), (0.9, 0.3), (1.0, 0.9), (1.6, 1.1), (2.2, 1.1)]
    lookahead = 3 * res
    fig, axs = plt.subplots(1, 2, figsize=(13, 5.6), dpi=DPI)
    fig.subplots_adjust(left=0.04, right=0.99, top=0.86, bottom=0.07, wspace=0.08)
    for ax, (pose, title) in zip(axs, (((0.35, -0.22, 40.0), "Carrot ahead: drive forward, PID turns the front toward it"),
                                        ((0.75, 0.45, -150.0), "Carrot more than 90 deg behind: drive in REVERSE (back = front)"))):
        fake = types.SimpleNamespace(path=[types.SimpleNamespace(pos=p) for p in route], pose=np.array(pose),
                                     current_node_index=0, carrot_lookahead_distance=lookahead)
        carrot = pp.WaypointNode.get_carrot_target(fake)
        pts = np.array(route)
        ax.plot(pts[:, 0], pts[:, 1], "-o", color="tab:green", lw=2, ms=5, zorder=2, label="planned route (RRT* nodes)")
        # projection of the robot onto the route (the segment get_carrot_target picked)
        seg = fake.current_node_index
        a, b = pts[seg], pts[seg + 1]
        r = np.array(pose[:2])
        f = np.clip((r - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1)
        proj = a + f * (b - a)
        # the lookahead walked along the route, projection -> carrot
        walk = [proj] + [pts[k] for k in range(seg + 1, len(pts)) if np.hypot(*(pts[k] - proj)) < lookahead + 1e-9
                         and np.hypot(*(pts[k] - np.array(carrot))) > 1e-6 and k <= seg + 3] + [np.array(carrot)]
        walk = np.array(walk)
        ax.plot(walk[:, 0], walk[:, 1], color="darkorange", lw=6, alpha=0.55, zorder=3, label="lookahead: 0.6 m along the route")
        ax.plot([r[0], proj[0]], [r[1], proj[1]], "k:", lw=1.2, zorder=3)
        ax.plot(*proj, "ks", ms=7, zorder=5, label="projection (closest point ahead)")
        ax.plot(*carrot, marker="*", ms=20, color="gold", mec="k", zorder=6, label="carrot")
        # robot body and its front
        h = math.radians(pose[2])
        c, s = math.cos(h), math.sin(h)
        corners = [(0.12, 0.09), (0.12, -0.09), (-0.12, -0.09), (-0.12, 0.09)]
        ax.add_patch(Polygon([(r[0] + c * px - s * py, r[1] + s * px + c * py) for px, py in corners], closed=True,
                             fc="steelblue", ec="k", zorder=6))
        reverse = abs(math.degrees(math.atan2(-s * (carrot[0] - r[0]) + c * (carrot[1] - r[1]),
                                              c * (carrot[0] - r[0]) + s * (carrot[1] - r[1])))) > 90
        facing = h + (math.pi if reverse else 0.0)
        tip = r + 0.45 * np.array((math.cos(facing), math.sin(facing)))
        ax.add_patch(FancyArrowPatch(r, tip, arrowstyle="-|>", mutation_scale=16, color="k", lw=1.5, zorder=7))
        ax.text(*(tip + 0.04), "driving direction\n(back of the robot)" if reverse else "front", fontsize=9, zorder=7)
        ax.plot([r[0], carrot[0]], [r[1], carrot[1]], color="crimson", ls="--", lw=1.4, zorder=4)
        err = math.degrees(math.atan2(carrot[1] - r[1], carrot[0] - r[0]) - facing)
        err = (err + 180) % 360 - 180
        ax.text(0.02, 0.97, f"heading error {err:+.0f} deg  -> PID turn rate\n"
                            f"speed scaled down by distance, error and turn demand\n"
                            + ("reverse: linear.x < 0, turn x1.5" if reverse else "forward: linear.x > 0"),
                transform=ax.transAxes, va="top", family="monospace", fontsize=8.5, bbox=dict(fc="white", alpha=0.9), zorder=8)
        ax.set_aspect("equal")
        ax.set_xlim(-0.3, 2.5)
        ax.set_ylim(-0.7, 1.5)
        ax.grid(True, alpha=0.3)
        ax.set_title(title, fontsize=10)
        ax.legend(loc="lower right", fontsize=8, framealpha=0.95)
    fig.suptitle("CA2 path following: a carrot point 0.6 m ahead on the route, chased by a heading PID", fontsize=11)
    path = os.path.join(HERE, "ca2_carrot.png")
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    print(f"{path}: {os.path.getsize(path) / 1e3:.0f} kB")


if __name__ == "__main__":
    which = set(sys.argv[1:]) or {"carrot", "planner"}
    if "carrot" in which:
        figure_carrot()
    if "planner" in which:
        figure_planner()
