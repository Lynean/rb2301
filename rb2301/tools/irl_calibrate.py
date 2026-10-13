#!/usr/bin/env python3
"""
Work out origin_x, origin_y and rotation_deg for src/rb2301_ca2/rb2301_ca2/optitrack_variables.config.

No ROS needed. Put the robot (or any marker Optitrack can see) at two or more
points whose MAZE coordinates you know, read the Optitrack (x, y) at each one
(ros2 run rb2301_ca2 path_planning --calibrate prints it, or
`ros2 topic echo /vrpn_mocap/bingda_003/pose`), then:

    python3 tools/irl_calibrate.py \
        --pair 0.5 0.5  <opti_x> <opti_y> \
        --pair 2.9 2.9  <opti_x> <opti_y>

Each --pair is: maze_x maze_y optitrack_x optitrack_y. Good maze points: the
centre of any free cell, e.g. a run's start point (see README), and a point far
from it so the rotation is well determined. More pairs = a least-squares fit
and a residual check.

Frame convention (same as path_planning.py): optitrack = origin + R(rotation_deg) * maze.
"""
import argparse
import math
import sys


def solve(pairs):
    n = len(pairs)
    mx = sum(p[0] for p in pairs) / n
    my = sum(p[1] for p in pairs) / n
    ox = sum(p[2] for p in pairs) / n
    oy = sum(p[3] for p in pairs) / n
    dot = cross = 0.0
    for a, b, c, d in pairs:
        ax, ay, bx, by = a - mx, b - my, c - ox, d - oy
        dot += ax * bx + ay * by
        cross += ax * by - ay * bx
    rot = math.atan2(cross, dot)
    cs, sn = math.cos(rot), math.sin(rot)
    origin = (ox - (cs * mx - sn * my), oy - (sn * mx + cs * my))
    res = []
    for a, b, c, d in pairs:
        px, py = origin[0] + cs * a - sn * b, origin[1] + sn * a + cs * b
        res.append(math.hypot(px - c, py - d))
    return origin, math.degrees(rot), res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pair", nargs=4, type=float, action="append", required=True,
                    metavar=("MAZE_X", "MAZE_Y", "OPTI_X", "OPTI_Y"))
    args = ap.parse_args()
    if len(args.pair) < 2:
        ap.error("need at least 2 --pair points")
    if len({(p[0], p[1]) for p in args.pair}) < 2:
        ap.error("the maze points must be different from each other")
    (ox, oy), rot, res = solve(args.pair)
    print("Paste into optitrack_variables.config, section [frame]:\n")
    print(f"origin_x = {ox:.3f}\norigin_y = {oy:.3f}\nrotation_deg = {rot:.2f}\n")
    worst = max(res)
    print("Fit residual per point (m): " + ", ".join(f"{r:.3f}" for r in res))
    if worst > 0.05:
        print(f"WARNING: worst residual {worst*100:.1f} cm. A rigid fit should match to ~1-2 cm. Re-read the poses, "
              f"check the maze points, and check Motive isn't streaming Y-up (x,y must be the floor plane).")
        sys.exit(1)
    print("Fit looks good.")


if __name__ == "__main__":
    main()
