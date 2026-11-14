"""Run randomised final-project courses in pocket_test and save each run as a GIF named with its result.

Courses where the robot spawns on top of an obstacle are rejected (not counted). Results:
    COMPLETE   reached the finish line
    TIMEOUT    still driving when pocket_test's time limit ran out
    COLLISION  touched an obstacle (the run stops there)

Usage (from rb2301/src/rb2301_fp):
    python3 sim_runs/make_sim_runs.py               # 20 runs, hand-swung boards, seeds from 0
    python3 sim_runs/make_sim_runs.py --runs 5 --first-seed 100 --motorised
"""
import argparse
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import pocket_test as pt

FPS = 15
EVERY = 6          # draw every 6th control step (0.6 s of robot time per frame)
DPI = 60


def result_of(frames):
    last = frames[-1]
    if last.get("spawn_overlap"):
        return "SPAWN_OVERLAP"
    if last.get("finished"):
        return "COMPLETE"
    if last["collision"]:
        return "COLLISION"
    return "TIMEOUT"


def state_text(f):
    c = f["corridor"]
    if c.get("board_phase"):
        return "BOARDS " + c["board_phase"].split(" (")[0]
    if c.get("gap_phase"):
        return "GATE " + c["gap_phase"]
    if c.get("locked"):
        return "obstacle: turn " + ("left" if f["w"] > 0 else "right") + (" BACKING UP" if c.get("backing") else "")
    return "clear: " + c["goal"] if c.get("goal") else "clear"


def save_gif(path, scn, frames, title, result):
    picks = list(range(0, len(frames), EVERY))
    if picks[-1] != len(frames) - 1:
        picks.append(len(frames) - 1)
    picks += [len(frames) - 1] * (2 * FPS)  # hold the result for 2 s

    fig, ax = plt.subplots(figsize=(6.4, 6.4), dpi=DPI)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.93, bottom=0.06)
    pt.draw_world(ax, scn, title)
    xs, ys = [f["x"] for f in frames], [f["y"] for f in frames]
    allx = [p[0] for seg in scn["segments"] for p in seg] + xs
    ally = [p[1] for seg in scn["segments"] for p in seg] + ys
    cx, cy = (min(allx) + max(allx)) / 2, (min(ally) + max(ally)) / 2
    half = max(max(allx) - min(allx), max(ally) - min(ally)) / 2 + 0.3
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)

    path_line, = ax.plot([], [], "b-", lw=1)
    scan = ax.scatter([], [], s=2, c="gray")
    corridor = Polygon(pt.corridor_polygon(0, 0, 0), closed=True, fc="orange", ec="darkorange", alpha=0.2)
    body = Polygon(pt.footprint(0, 0, 0), closed=True, fc="steelblue", ec="k", alpha=0.9)
    ax.add_patch(corridor)
    ax.add_patch(body)
    moving = LineCollection([], colors="purple", linewidths=3)
    ax.add_collection(moving)
    text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", family="monospace", fontsize=9,
                   bbox=dict(fc="white", alpha=0.85))
    colour = {"COMPLETE": "green", "TIMEOUT": "darkorange", "COLLISION": "red"}[result]

    def draw(k):
        i = picks[k]
        f = frames[i]
        path_line.set_data(xs[: i + 1], ys[: i + 1])
        scan.set_offsets(f["world_pts"])
        corridor.set_xy(pt.corridor_polygon(f["x"], f["y"], f["h"]))
        body.set_xy(pt.footprint(f["x"], f["y"], f["h"]))
        moving.set_segments([[p, q] for p, q in f["moving"]])
        end = i == len(frames) - 1
        text.set_text(f"t={f['t']:5.1f}s  v={f['v']:+.2f} m/s\n" + (f"RESULT: {result}" if end else state_text(f)))
        text.get_bbox_patch().set_edgecolor(colour if end else "black")
        text.get_bbox_patch().set_linewidth(2.5 if end else 1.0)
        return []

    FuncAnimation(fig, draw, frames=len(picks), blit=False).save(path, writer=PillowWriter(fps=FPS))
    plt.close(fig)


def make_grid(cols=5, tile=360):
    '''One image of every run's final frame (full path + result), framed in its result colour.'''
    import glob
    from PIL import Image, ImageDraw
    files = sorted(glob.glob(os.path.join(HERE, "run_*.gif")))
    colours = {"COMPLETE": (40, 160, 60), "TIMEOUT": (230, 140, 0), "COLLISION": (220, 40, 40)}
    rows = (len(files) + cols - 1) // cols
    pad = 6
    grid = Image.new("RGB", (cols * (tile + pad) + pad, rows * (tile + pad) + pad), "white")
    for k, path in enumerate(files):
        im = Image.open(path)
        im.seek(im.n_frames - 1)
        frame = im.convert("RGB").resize((tile, tile))
        result = os.path.basename(path).rsplit("_", 1)[1].split(".")[0]
        ImageDraw.Draw(frame).rectangle([0, 0, tile - 1, tile - 1], outline=colours.get(result, (0, 0, 0)), width=6)
        grid.paste(frame, (pad + (k % cols) * (tile + pad), pad + (k // cols) * (tile + pad)))
    out = os.path.join(HERE, "all_runs.png")
    grid.save(out, optimize=True)
    print(f"{out}: {len(files)} runs, {os.path.getsize(out) / 1e6:.1f} MB")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--runs", type=int, default=20, help="Valid runs to make (default 20)")
    parser.add_argument("--first-seed", type=int, default=0, help="First course seed to try (default 0)")
    parser.add_argument("--motorised", action="store_true", help="Full-swing motorised boards instead of hand-swung ones")
    parser.add_argument("--grid-only", action="store_true", help="Only rebuild all_runs.png from the GIFs already here")
    args = parser.parse_args()
    if args.grid_only:
        make_grid()
        return
    pt.SWING = not args.motorised
    boards = "motorised boards" if args.motorised else "hand-swung boards"

    rows, rejected, seed = [], [], args.first_seed
    time_limit = pt.randomised_fp_scenario(args.first_seed).get("max_time", pt.MAX_TIME)
    while len(rows) < args.runs:
        scn = pt.randomised_fp_scenario(seed)
        frames = pt.simulate(scn)
        result = result_of(frames)
        if result == "SPAWN_OVERLAP":
            rejected.append(seed)
            print(f"seed {seed}: spawned on top of an obstacle, rejected", flush=True)
            seed += 1
            continue
        n = len(rows) + 1
        name = f"run_{n:02d}_seed_{seed}_{result}.gif"
        travelled = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(frames, frames[1:]))
        gate = next((f["t"] for f in frames if f["corridor"].get("gate_passed") or f["corridor"].get("board_phase")), None)
        save_gif(os.path.join(HERE, name), scn, frames, f"Run {n} (seed {seed}, {boards})", result)
        rows.append((n, seed, result, frames[-1]["t"], travelled, gate, name))
        print(f"run {n:2d}  seed {seed:3d}  {result:9s}  t={frames[-1]['t']:5.1f}s  {travelled:5.1f} m  -> {name}", flush=True)
        seed += 1

    counts = {r: sum(1 for row in rows if row[2] == r) for r in ("COMPLETE", "TIMEOUT", "COLLISION")}
    done = [row[3] for row in rows if row[2] == "COMPLETE"]
    with open(os.path.join(HERE, "README.md"), "w") as f:
        f.write(f"# pocket_test: {len(rows)} randomised runs\n\n")
        f.write(f"Randomised final-project courses (same randomisation as `obstacle_modifier.py`) with {boards}, "
                f"run by the real controller in `pocket_test.py`. Time limit {time_limit:.0f} s per run.\n\n")
        f.write("![All runs](all_runs.png)\n\n*Final frame of every run: green frame = COMPLETE, orange = TIMEOUT, red = COLLISION.*\n\n")
        f.write("| Result | Runs |\n| --- | --- |\n")
        for r, c in counts.items():
            f.write(f"| {r} | {c} |\n")
        if done:
            f.write(f"\nCompleted runs took {min(done):.0f}–{max(done):.0f} s (mean {sum(done) / len(done):.0f} s).\n")
        if rejected:
            f.write(f"\nRejected (robot spawned on top of an obstacle): seeds {', '.join(map(str, rejected))}.\n")
        f.write("\n| Run | Seed | Result | Time (s) | Distance (m) | Gate passed at (s) | Animation |\n"
                "| --- | --- | --- | --- | --- | --- | --- |\n")
        for n, sd, r, t, dist, g, name in rows:
            f.write(f"| {n} | {sd} | **{r}** | {t:.1f} | {dist:.1f} | {'-' if g is None else f'{g:.1f}'} | [{name}]({name}) |\n")
        f.write("\nRe-run: `python3 sim_runs/make_sim_runs.py` from `rb2301/src/rb2301_fp` (see the script for options).\n")
    make_grid()
    print(counts, "rejected:", rejected)


if __name__ == "__main__":
    main()
