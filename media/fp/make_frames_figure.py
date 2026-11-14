"""FP frames figure: simulation vs real-robot (OptiTrack) axes against the compass.  Usage: python3 media/fp/make_frames_figure.py"""
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.realpath(__file__))
# Values from obstacle_course.py
NORTH_DEG, TARGET_HEADING_DEG = 0.0, -45.0            # simulation
REAL_NORTH_DEG, REAL_TARGET_HEADING_DEG = -90.0, -135.0  # real robot (OptiTrack)


def arrow(ax, ang, length, colour, text, lw=2.5, ls="-", off=1.12, size=11):
    """Arrow from the centre at compass angle ang (0 = up = north, clockwise positive)."""
    dx, dy = length * math.sin(math.radians(ang)), length * math.cos(math.radians(ang))
    ax.annotate("", xy=(dx, dy), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color=colour, lw=lw, ls=ls, mutation_scale=18))
    ax.text(dx * off, dy * off, text, color=colour, ha="center", va="center", fontsize=size, fontweight="bold")


def panel(ax, title, x_compass, y_compass, north_deg, target_deg, notes):
    ax.set_xlim(-1.75, 1.75)
    ax.set_ylim(-1.95, 1.75)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(title, fontsize=11)
    # Compass (light): north up, east right
    for ang, name in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
        arrow(ax, ang, 1.35, "0.75", name, lw=1.2, off=1.13, size=10)
    # North-east: the goal heading
    arrow(ax, 45, 1.0, "goldenrod", f"north-east\n= {target_deg:+.0f} deg", lw=3, off=1.4, size=9.5)
    # The frame's own axes
    arrow(ax, x_compass, 0.8, "tab:red", "+x", lw=2.5, off=1.22)
    arrow(ax, y_compass, 0.8, "tab:blue", "+y", lw=2.5, off=1.22)
    ax.plot(0, 0, "ko", ms=5)
    ax.text(-1.72, -1.92, notes, ha="left", va="bottom", fontsize=8.5, family="monospace",
            bbox=dict(fc="white", ec="0.7"))


fig, axs = plt.subplots(1, 2, figsize=(11, 5.6), dpi=90)
fig.subplots_adjust(left=0.01, right=0.99, top=0.86, bottom=0.02, wspace=0.02)
panel(axs[0], "Simulation (Gazebo / pocket_test)", 0, 270, NORTH_DEG, TARGET_HEADING_DEG,
      f"NORTH_DEG = {NORTH_DEG:+.0f}  (north = +x)\neast = -y\nTARGET_HEADING_DEG = {TARGET_HEADING_DEG:+.0f}")
panel(axs[1], "Real robot (OptiTrack: +x = West, +y = South)", 270, 180, REAL_NORTH_DEG, REAL_TARGET_HEADING_DEG,
      f"REAL_NORTH_DEG = {REAL_NORTH_DEG:+.0f}  (north = -y)\neast = -x\nREAL_TARGET_HEADING_DEG = {REAL_TARGET_HEADING_DEG:+.0f}")
fig.suptitle("Same course, two frames: the controller only needs which way north is (headings: 0 deg = +x, counter-clockwise +)",
             fontsize=11)
path = os.path.join(HERE, "frames.png")
fig.savefig(path, dpi=90)
print(f"{path}: {os.path.getsize(path) / 1e3:.0f} kB")
