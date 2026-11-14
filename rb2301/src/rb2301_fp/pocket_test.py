#!/usr/bin/env python3
"""2D matplotlib test of the front-corridor avoidance in rb2301_fp/obstacle_course.py.

Runs the real avoidance_command() against a ray-cast fake LiDAR in a few 2D worlds
(walls, boxes, angled walls, a reverse-C pocket) and animates the robot, its rectangular
front corridor and the scan points the corridor sees.

Usage (from src/rb2301_fp):
    python3 pocket_test.py                 # animate every scenario, one after another
                                           # (player: slider, Play/Pause, Rewind, Stop, < > steps, speed;
                                           #  keys: space, left/right, r, s; close the window for the next one)
    python3 pocket_test.py pocket          # animate one scenario
    python3 pocket_test.py --no-anim       # just draw the final paths of all scenarios
    python3 pocket_test.py --real          # use the real robot's speed caps and 720-beam scan
    python3 pocket_test.py random          # a new randomised Gazebo-style course (like obstacle_modifier.py)
    python3 pocket_test.py random --seed 7 # a specific random course (repeatable)
    python3 pocket_test.py random --seed 0 --trials 20 --headless   # success rate over 20 courses
"""
import argparse
import math
import os
import sys
import types

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from rb2301_fp import obstacle_course as oc  # noqa: E402

DT = 0.1                  # Control/scan period (s): LiDAR runs at 10 Hz
MAX_TIME = 60.0           # Give up after this many simulated seconds
# Real nanocar footprint from the URDF meshes (base_link, wheels, LiDAR), relative to the
# odometry origin (rear axle). Only used for drawing and collision checking.
BODY_REAR, BODY_FRONT = -0.034, 0.181   # m along x
BODY_HALF_WIDTH = 0.110                 # m (wheels stick out to +-0.11)
GOAL_RADIUS = 0.25


# ---------------------------------------------------------------- worlds
def box(cx, cy, w, h):
    '''Axis-aligned rectangle as 4 segments.'''
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


def angled_wall(cx, cy, length, angle_deg):
    a = math.radians(angle_deg)
    dx, dy = math.cos(a) * length / 2, math.sin(a) * length / 2
    return [((cx - dx, cy - dy), (cx + dx, cy + dy))]


def arena(w=4.0, h=4.0):
    return box(w / 2 - 0.5, 0.0, w, h)


def circle(cx, cy, r, n=16):
    pts = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
    return [(pts[k], pts[(k + 1) % n]) for k in range(n)]


def zigzag_corridor(centre, width):
    '''Walls of a corridor of this width along the centre polyline (mitred corners, both ends closed).'''
    c = np.asarray(centre, dtype=float)
    d = np.diff(c, axis=0)
    d /= np.hypot(d[:, 0], d[:, 1])[:, None]
    n = np.column_stack((-d[:, 1], d[:, 0]))  # left normal of each leg
    sides = []
    for sign in (1.0, -1.0):
        pts = [c[0] + sign * width / 2 * n[0]]
        for k in range(1, len(c) - 1):
            m = n[k - 1] + n[k]
            m /= np.hypot(*m)
            pts.append(c[k] + sign * width / 2 * m / float(m @ n[k]))  # mitre keeps the width on both legs
        pts.append(c[-1] + sign * width / 2 * n[-1])
        sides.append([tuple(p) for p in pts])
    walls = [(a, b) for side in sides for a, b in zip(side, side[1:])]
    return walls + [(sides[0][0], sides[1][0]), (sides[0][-1], sides[1][-1])]


# ---------------------------------------------------------------- Gazebo final-project world
_WORLDS_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "rb2301_gz", "worlds")
FP_WORLD = os.path.join(_WORLDS_DIR, "obstacle_course_world_fp.sdf")  # Re-randomised by every fp_gazebo launch

# Footprints taken from the model meshes in rb2301_gz/meshes
WALL_HALF_LENGTH, WALL_OFFSET = 0.6, 0.6   # nist maze wall 120: 1.2 m long, 0.6 m to the model's +y side
DRAWER_HALF_X, DRAWER_HALF_Y = 0.205, 0.24  # drawer/2 Drawer_Col.obj
LITTERBIN_RADIUS = 0.226                   # litterbin/2
COKE_RADIUS = 0.03                         # coke/6


def _pose(text):
    v = [float(t) for t in text.split()]
    return v[0], v[1], v[5]  # x, y, yaw


def _transform(points, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in points]


def load_fp_world(path=FP_WORLD, tree=None):
    '''Parse the Gazebo final-project SDF (file path or ElementTree) into static segments and moving actors.'''
    import xml.etree.ElementTree as ET
    world = (tree if tree is not None else ET.parse(path)).getroot().find("world")
    segments = []
    for inc in world.findall("include"):
        uri, pose = inc.findtext("uri", ""), inc.find("pose")
        if pose is None:
            continue
        x, y, yaw = _pose(pose.text)
        if "nist maze wall" in uri:
            a, b = _transform([(-WALL_HALF_LENGTH, WALL_OFFSET), (WALL_HALF_LENGTH, WALL_OFFSET)], x, y, yaw)
            segments.append((a, b))
        elif "drawer" in uri:
            corners = _transform([(-DRAWER_HALF_X, -DRAWER_HALF_Y), (DRAWER_HALF_X, -DRAWER_HALF_Y),
                                  (DRAWER_HALF_X, DRAWER_HALF_Y), (-DRAWER_HALF_X, DRAWER_HALF_Y)], x, y, yaw)
            segments += [(corners[k], corners[(k + 1) % 4]) for k in range(4)]
        elif "litterbin" in uri:
            segments += circle(x, y, LITTERBIN_RADIUS)
        elif "coke" in uri:
            segments += circle(x, y, COKE_RADIUS, n=8)

    actors = []
    for actor in world.findall("actor"):
        size = [float(v) for v in actor.find(".//box/size").text.split()]
        waypoints = [(float(w.findtext("time")), _pose(w.findtext("pose")))
                     for w in actor.find(".//trajectory").findall("waypoint")]
        actors.append((size[0] / 2, size[1] / 2, waypoints))
    return segments, actors


def actor_segments(actors, t):
    '''Box segments of every looping actor at time t (linear motion between waypoints).'''
    segs = []
    for hx, hy, wps in actors:
        period = wps[-1][0]
        tt = t % period if period > 0 else 0.0
        for (t0, p0), (t1, p1) in zip(wps, wps[1:]):
            if t0 <= tt <= t1:
                f = (tt - t0) / (t1 - t0) if t1 > t0 else 0.0
                x, y, yaw = (p0[i] + (p1[i] - p0[i]) * f for i in range(3))
                break
        corners = _transform([(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)], x, y, yaw)
        segs += [(corners[k], corners[(k + 1) % 4]) for k in range(4)]
    return segs


SWING_DECAY_TIME = 70.0   # --swing: swing amplitude falls to 1/e after this long (s)
SWING_BEND = 0.03         # --swing: board middle bulges this far out of flat (m)
SWING_WOBBLE_DEG = 10.0   # --swing: board twists up to this much while swinging (deg)


def swing_actor_segments(actors, t):
    '''--swing: the boards as hand-swung pendulums instead of motorised actors. Same travel line as the
    actor (centred between its first two waypoints), but sinusoidal, with a decaying amplitude, a bent
    (not flat) board and a small twist.'''
    segs = []
    for k, (hx, hy, wps) in enumerate(actors):
        p0, p1 = np.array(wps[0][1][:2]), np.array(wps[1][1][:2])
        centre, amp = (p0 + p1) / 2.0, (p1 - p0) / 2.0
        period = float(np.clip(wps[-1][0] * 0.5, 2.0, 4.0))
        phase = 2 * math.pi * t / period + 1.3 * k
        x, y = centre + amp * math.exp(-t / SWING_DECAY_TIME) * math.sin(phase)
        yaw = wps[0][1][2] + math.radians(SWING_WOBBLE_DEG) * math.cos(phase) * math.exp(-t / SWING_DECAY_TIME)
        outline = [(-hx, -hy), (hx, -hy), (hx - SWING_BEND, 0.0), (hx, hy), (-hx, hy), (-hx - SWING_BEND, 0.0)]
        corners = _transform(outline, x, y, yaw)
        segs += [(corners[i], corners[(i + 1) % len(corners)]) for i in range(len(corners))]
    return segs


FP_ORIGINAL = os.path.join(_WORLDS_DIR, "obstacle_course_world_fp_original.sdf")


def _goal_x(tree):
    '''Finish line: just past the gate (the configurable gate walls sit at x = 4.2 + x_offset).'''
    for inc in tree.getroot().find("world").findall("include"):
        if "configurable" in inc.findtext("name", ""):
            return _pose(inc.find("pose").text)[0] + 0.8
    return None


SWING = False  # Set by --swing


def _fp_scenario(tree, desc):
    static, actors = load_fp_world(tree=tree)
    move = swing_actor_segments if SWING else actor_segments
    return {
        "desc": desc + (" [swinging boards]" if SWING else ""),
        "segments": static,
        "segments_at": lambda t: static + move(actors, t),
        "start": (0.05, 0.0, 0.0),  # fp_gazebo.launch.py spawn pose
        "goal_x": _goal_x(tree),
        "max_time": 180.0,
    }


def fp_course_scenario():
    '''The world Gazebo loaded at its last launch.'''
    import xml.etree.ElementTree as ET
    if not os.path.exists(FP_WORLD):
        return None
    return _fp_scenario(ET.parse(FP_WORLD), "Gazebo final-project course (last fp_gazebo launch)")


def randomised_fp_scenario(seed):
    '''Same randomisation as rb2301_fp/obstacle_modifier.py, applied in memory to the original SDF
    (the Gazebo world file is not touched).'''
    import xml.etree.ElementTree as ET
    rng = np.random.default_rng(seed)
    tree = ET.parse(FP_ORIGINAL)
    world = tree.getroot().find("world")

    # obstacle_modifier: randomise_zigzag_size, randomise_gate_position, randomise_obstacle_speed
    x_offset = -rng.random() * 0.8
    y_offset = -rng.random() * 0.6
    gate_y_start = -rng.random() * 0.5 - 1.4
    gate_y_end = gate_y_start - 1.6
    for element in world:
        if element.tag == "include":
            name, pose_el = element.findtext("name", ""), element.find("pose")
            if pose_el is None:
                continue
            pose = pose_el.text.split(" ")
            if name == "nist_maze_wall_120_configurable_left":
                pose_el.text = f"{4.2 + x_offset} {gate_y_start + y_offset} -0.7 0 0 1.57079633"
            elif name == "nist_maze_wall_120_configurable_right":
                pose_el.text = f"{4.2 + x_offset} {gate_y_end + y_offset} -0.7 0 0 1.57079633"
            elif "nist_maze_wall" in name:
                if float(pose[0]) > 1.0:
                    pose[0] = str(float(pose[0]) + x_offset)
                if float(pose[1]) < -2.6:
                    pose[1] = str(float(pose[1]) + y_offset)
                pose_el.text = " ".join(pose)
        elif element.tag == "actor":
            trajectory = element.find("script").find("trajectory")
            half_period = rng.random() * 4.0 + 2.0
            trajectory[1][0].text = str(half_period)
            trajectory[2][0].text = str(half_period * 2)
            for k in range(3):
                wp = trajectory[k][1].text.split(" ")
                wp[0] = str(float(wp[0]) + x_offset)
                wp[1] = str(float(wp[1]) + y_offset)
                trajectory[k][1].text = " ".join(wp)

    # obstacle_modifier: spawn_static_obstacles + randomise_static_obstacles
    models = ["://rb2301_gz/meshes/coke/6", "://rb2301_gz/meshes/drawer/2", "://rb2301_gz/meshes/litterbin/2"]
    base = [(0.6, 0.0), (2.4 + x_offset, -0.6 + y_offset), (2.4 + x_offset, -1.4 + y_offset),
            (2.4 + x_offset, -2.4 + y_offset)]
    for idx, (ox, oy) in enumerate(base):
        ox += (rng.random() - 0.5) * 0.8
        oy += (rng.random() - 0.5) * 0.6
        inc = ET.SubElement(world, "include")
        ET.SubElement(inc, "uri").text = models[int(rng.integers(3))]
        ET.SubElement(inc, "name").text = f"static_obstacle{idx}"
        ET.SubElement(inc, "pose").text = f"{ox} {oy} 0 0 0 0"
    return _fp_scenario(tree, f"Randomised final-project course (seed {seed})")


# Robot starts at (0, 0) facing +x unless the scenario says otherwise.
SCENARIOS = {
    "box_right": {
        "desc": "Box on the right of the corridor -> should turn left",
        "segments": arena() + box(1.2, -0.15, 0.3, 0.3),
        "start": (0.0, 0.0, 0.0),
    },
    "box_left": {
        "desc": "Box on the left of the corridor -> should turn right",
        "segments": arena() + box(1.2, 0.15, 0.3, 0.3),
        "start": (0.0, 0.0, 0.0),
    },
    "perpendicular": {
        "desc": "Wall square across the path -> should turn right",
        "segments": arena() + angled_wall(1.2, 0.0, 1.2, 90.0),
        "start": (0.0, 0.0, 0.0),
    },
    "slash": {
        "desc": "'/' wall (right end farther) -> should turn right",
        "segments": arena() + angled_wall(1.2, 0.0, 1.2, 90.0 + 25.0),
        "start": (0.0, 0.0, 0.0),
    },
    "backslash": {
        "desc": "'\\' wall (left end farther) -> should turn left",
        "segments": arena() + angled_wall(1.2, 0.0, 1.2, 90.0 - 25.0),
        "start": (0.0, 0.0, 0.0),
    },
    "pocket": {
        "desc": "Reverse-C pocket open to the left, robot drives in",
        "segments": arena(5.0, 5.0)
        + [((1.0, -0.6), (1.6, -0.6)), ((1.6, -0.6), (1.6, 0.6)), ((1.6, 0.6), (1.0, 0.6))],
        "start": (0.0, 0.0, 0.0),
    },
    "pocket_deep": {
        "desc": "Deep pocket opening toward the robot",
        "segments": arena(5.0, 5.0)
        + [((0.6, -0.5), (2.0, -0.5)), ((2.0, -0.5), (2.0, 0.5)), ((2.0, 0.5), (0.6, 0.5))],
        "start": (0.0, 0.0, 0.0),
    },
    "course": {
        "desc": "Mixed course heading north-east",
        "segments": arena(5.0, 5.0)
        + box(0.9, 0.2, 0.3, 0.3) + box(1.6, 1.0, 0.3, 0.3)
        + angled_wall(1.4, -0.3, 0.8, 120.0) + angled_wall(2.4, 1.6, 1.0, 60.0)
        + box(2.6, 0.4, 0.4, 0.4),
        "start": (0.0, 0.0, 45.0),
    },
    "zigzag": {
        "desc": "Zigzag corridor (0.5 m wide) heading north, 0.85 m legs alternating NE / NW",
        "segments": zigzag_corridor([(-0.3, 0.3)] + [(0.6 * k, -0.6 * (k % 2)) for k in range(1, 10)], 0.5),
        "start": (0.0, 0.0, -45.0),
        "goal_x": 5.0,
        "max_time": 90.0,
    },
}
_fp = fp_course_scenario()
if _fp is not None:
    SCENARIOS["fp_course"] = _fp


# ---------------------------------------------------------------- fake sensor
def raycast_scan(segments, x, y, heading, n_beams):
    '''Fake LaserScan for a robot at (x, y, heading rad) using the real mounting offsets.'''
    msg = types.SimpleNamespace(angle_min=-math.pi, angle_increment=2 * math.pi / n_beams,
                                range_min=0.05, range_max=8.0)
    sx = x + oc.LASER_X_OFFSET * math.cos(heading)
    sy = y + oc.LASER_X_OFFSET * math.sin(heading)
    angles = heading + oc.LASER_YAW_OFFSET + msg.angle_min + np.arange(n_beams) * msg.angle_increment
    dx, dy = np.cos(angles), np.sin(angles)
    ranges = np.full(n_beams, np.inf)
    for (px, py), (qx, qy) in segments:
        ex, ey = qx - px, qy - py
        den = dx * (-ey) - dy * (-ex)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((px - sx) * (-ey) - (py - sy) * (-ex)) / den
            u = (dx * (py - sy) - dy * (px - sx)) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        ranges = np.where(hit & (t < ranges), t, ranges)
    msg.ranges = list(ranges)
    return msg


def footprint(x, y, heading):
    c, s = math.cos(heading), math.sin(heading)
    corners = [(BODY_FRONT, BODY_HALF_WIDTH), (BODY_FRONT, -BODY_HALF_WIDTH),
               (BODY_REAR, -BODY_HALF_WIDTH), (BODY_REAR, BODY_HALF_WIDTH)]
    return np.array([(x + c * px - s * py, y + s * px + c * py) for px, py in corners])


CONE_DRAW_MAX = oc.SIDE_CONE_MAX_RANGE  # Side-cone wedges are drawn up to the cones' own max range (m)
CONE_DRAW_IDLE = 0.6   # Length of the faint outline when the cones weren't measured on that step (m)


def cone_polygon(x, y, heading, side, length, centre_deg=None, half_deg=None):
    '''Wedge from the LiDAR, `length` m long. Default: one side's whole search sector (side +1 left,
    -1 right); with centre_deg / half_deg: a specific cone (robot-frame bearing, positive = left).'''
    lx, ly = x + oc.LASER_X_OFFSET * math.cos(heading), y + oc.LASER_X_OFFSET * math.sin(heading)
    centre = heading + (math.radians(centre_deg) if centre_deg is not None
                        else side * math.radians(oc.SIDE_CONE_ANGLE_DEG))
    half = math.radians(half_deg if half_deg is not None else oc.SIDE_CONE_HALF_WIDTH_DEG)
    arc = [(lx + length * math.cos(a), ly + length * math.sin(a))
           for a in np.linspace(centre - half, centre + half, 9)]
    return np.array([(lx, ly)] + arc)


def corridor_polygon(x, y, heading):
    c, s = math.cos(heading), math.sin(heading)
    w, L = oc.CORRIDOR_HALF_WIDTH, oc.CORRIDOR_LENGTH
    corners = [(0, w), (L, w), (L, -w), (0, -w)]
    return np.array([(x + c * px - s * py, y + s * px + c * py) for px, py in corners])


def collides(segments, x, y, heading):
    '''True if any wall segment comes within the robot rectangle.'''
    c, s = math.cos(heading), math.sin(heading)
    for (px, py), (qx, qy) in segments:
        for t in np.linspace(0.0, 1.0, 25):
            wx, wy = px + (qx - px) * t - x, py + (qy - py) * t - y
            fx, fy = c * wx + s * wy, -s * wx + c * wy
            if BODY_REAR <= fx <= BODY_FRONT and abs(fy) <= BODY_HALF_WIDTH:
                return True
    return False


# ---------------------------------------------------------------- simulation
def simulate(scn, real=False):
    '''Run one scenario. Returns a list of per-step frames.'''
    max_v = 0.5 if real else 1.4  # Same caps as ObstacleCourseNode.move_2D
    n_beams = 720 if real else 360
    x, y, h = scn["start"][0], scn["start"][1], math.radians(scn["start"][2])
    state = {}  # avoidance_command() memory (turn lock, gap manoeuvre)
    frames = []
    for step in range(int(scn.get("max_time", MAX_TIME) / DT)):
        t = step * DT
        segments = scn["segments_at"](t) if "segments_at" in scn else scn["segments"]
        msg = raycast_scan(segments, x, y, h, n_beams)
        forward, turn, state, corridor = oc.avoidance_command(msg, state, (x, y, math.degrees(h)))
        forward = float(np.clip(forward, -max_v, max_v))
        turn = float(np.clip(turn, -2 * max_v, 2 * max_v))

        pts, _ = oc.scan_to_robot_points(msg)
        c, s = math.cos(h), math.sin(h)
        world_pts = np.column_stack((x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]))
        inside = ((pts[:, 0] > 0) & (pts[:, 0] <= oc.CORRIDOR_LENGTH)
                  & (np.abs(pts[:, 1]) <= oc.CORRIDOR_HALF_WIDTH))
        hit = collides(segments, x, y, h)
        if hit and step == 0:
            frames.append(dict(t=t, x=x, y=y, h=h, v=0.0, w=0.0, corridor=corridor, world_pts=world_pts,
                               inside=inside, collision=True, spawn_overlap=True, finished=False,
                               moving=segments[len(scn["segments"]):]))
            break
        finished = scn.get("goal_x") is not None and x >= scn["goal_x"]
        frames.append(dict(t=t, x=x, y=y, h=h, v=forward, w=turn, corridor=corridor,
                           world_pts=world_pts, inside=inside, collision=hit, finished=finished,
                           moving=segments[len(scn["segments"]):]))
        if hit or finished:
            break
        # Unicycle motion (the controller only uses linear.x and angular.z)
        x += forward * math.cos(h) * DT
        y += forward * math.sin(h) * DT
        h += turn * DT
    return frames


def summary(name, frames):
    last = frames[-1]
    stuck = len(frames) > 50 and all(abs(f["v"]) < 1e-3 and abs(f["w"]) < 1e-3 for f in frames[-30:])
    travelled = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(frames, frames[1:]))
    status = ("SPAWN OVERLAP" if last.get("spawn_overlap") else "COLLISION" if last["collision"] else f"FINISHED in {last['t']:.0f}s" if last.get("finished")
              else "STUCK (v=0, no turn)" if stuck else "ok")
    first_turn = next((f for f in frames if f["w"] != 0.0), None)
    turn_text = "never turned" if first_turn is None else (
        f"first turn {'LEFT' if first_turn['w'] > 0 else 'RIGHT'} at t={first_turn['t']:.1f}s "
        f"(blocking={first_turn['corridor']['blocking']})")
    print(f"{name:14s} {status:22s} travelled={travelled:5.2f} m  {turn_text}")
    return status


def draw_world(ax, scn, title):
    for (px, py), (qx, qy) in scn["segments"]:
        ax.plot([px, qx], [py, qy], color="saddlebrown", lw=3)
    if scn.get("goal_x") is not None:
        ax.axvline(scn["goal_x"], color="green", ls="--", lw=1)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    ax.grid(True, alpha=0.3)


def animate(name, scn, frames):
    '''Interactive player: time slider, Play/Pause, Rewind (play backwards), Stop, single steps, speed.
    Keys: space = play/pause, left/right = step, r = rewind, s = stop (back to start).'''
    from matplotlib.collections import LineCollection
    from matplotlib.widgets import Button, Slider

    # Free the keys we use from matplotlib's default shortcuts (back/forward/home/save)
    for key_map, keys in (("keymap.back", ["left"]), ("keymap.forward", ["right"]),
                          ("keymap.home", ["r"]), ("keymap.save", ["s"])):
        plt.rcParams[key_map] = [k for k in plt.rcParams[key_map] if k not in keys]

    fig = plt.figure(figsize=(8, 9))
    ax = fig.add_axes([0.08, 0.22, 0.88, 0.74])
    draw_world(ax, scn, f"{name}: {scn['desc']}")
    xs, ys = [f["x"] for f in frames], [f["y"] for f in frames]
    pad = 0.6
    ax.set_xlim(min(xs) - pad - 0.5, max(xs) + pad + 1.0)
    ax.set_ylim(min(ys) - pad - 1.0, max(ys) + pad + 1.0)
    path_line, = ax.plot([], [], "b-", lw=1)
    scan_dots = ax.scatter([], [], s=4, c="lightgray")
    corridor_dots = ax.scatter([], [], s=10, c="red")
    body = Polygon(footprint(0, 0, 0), closed=True, fc="steelblue", ec="k", alpha=0.8)
    corridor = Polygon(corridor_polygon(0, 0, 0), closed=True, fc="orange", ec="darkorange", alpha=0.2)
    ax.add_patch(corridor)
    ax.add_patch(body)
    moving = LineCollection([], colors="purple", linewidths=2)
    ax.add_collection(moving)
    gap_line, = ax.plot([], [], color="limegreen", lw=4, alpha=0.8)       # The gap being used
    gap_axis, = ax.plot([], [], color="limegreen", lw=1, ls="--")         # Its centre line
    board_axes = LineCollection([], colors="magenta", linewidths=1.5, linestyles="--")  # Detected board travel axes
    ax.add_collection(board_axes)
    virtual_walls = LineCollection([], colors="darkorange", linewidths=4, linestyles=":")  # Dead-end walls
    ax.add_collection(virtual_walls)
    cone_left = Polygon(cone_polygon(0, 0, 0, +1, CONE_DRAW_IDLE), closed=True, alpha=0.25)  # Side cones
    cone_right = Polygon(cone_polygon(0, 0, 0, -1, CONE_DRAW_IDLE), closed=True, alpha=0.25)
    ax.add_patch(cone_left)
    ax.add_patch(cone_right)
    heading_arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="k"))
    text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", family="monospace", fontsize=9,
                   bbox=dict(fc="white", alpha=0.8))

    def render(i):
        f = frames[i]
        path_line.set_data(xs[: i + 1], ys[: i + 1])
        moving.set_segments([[p, q] for p, q in f["moving"]])
        scan_dots.set_offsets(f["world_pts"])
        corridor_dots.set_offsets(f["world_pts"][f["inside"]] if np.any(f["inside"]) else np.empty((0, 2)))
        body.set_xy(footprint(f["x"], f["y"], f["h"]))
        body.set_facecolor("crimson" if f["collision"] else "steelblue")
        corridor.set_xy(corridor_polygon(f["x"], f["y"], f["h"]))
        heading_arrow.set_position((f["x"], f["y"]))
        heading_arrow.xy = (f["x"] + 0.3 * math.cos(f["h"]), f["y"] + 0.3 * math.sin(f["h"]))
        c = f["corridor"]
        if c.get("gap_phase"):
            gc, gn = c["gap_center"], c["gap_normal"]
            gt = np.array((-gn[1], gn[0])) * c["gap_width"] / 2.0
            gap_line.set_data([gc[0] - gt[0], gc[0] + gt[0]], [gc[1] - gt[1], gc[1] + gt[1]])
            gap_axis.set_data([gc[0] - gn[0] * 0.8, gc[0] + gn[0] * 0.5], [gc[1] - gn[1] * 0.8, gc[1] + gn[1] * 0.5])
        else:
            gap_line.set_data([], [])
            gap_axis.set_data([], [])
        board_axes.set_segments([[p0, p1] for p0, p1 in c.get("board_axes", [])])
        virtual_walls.set_segments([[a, b] for a, b in c.get("virtual_walls", [])])
        if "cone_left" in c:
            # Measured this step: length = depth; higher-scoring side green, other red (tie: both green)
            depths = {+1: c["cone_left"], -1: c["cone_right"]}
            scores = {+1: c.get("cone_left_score", c["cone_left"]), -1: c.get("cone_right_score", c["cone_right"])}
            best = {+1: c.get("cone_left_angle"), -1: c.get("cone_right_angle")}
            for patch, side in ((cone_left, +1), (cone_right, -1)):
                # Draw the deepest sub-cone found on that side
                patch.set_xy(cone_polygon(f["x"], f["y"], f["h"], side, min(depths[side], CONE_DRAW_MAX),
                                          centre_deg=best[side], half_deg=oc.SIDE_SUBCONE_HALF_WIDTH_DEG))
                deeper = scores[side] >= scores[-side] - oc.SIDE_CONE_TIE
                patch.set_facecolor("limegreen" if deeper else "red")
                patch.set_edgecolor("darkgreen" if deeper else "darkred")
                patch.set_alpha(0.35)
        else:
            # Not measured this step: faint outline of each side's search sector
            for patch, side in ((cone_left, +1), (cone_right, -1)):
                patch.set_xy(cone_polygon(f["x"], f["y"], f["h"], side, CONE_DRAW_IDLE))
                patch.set_facecolor("none")
                patch.set_edgecolor("gray")
                patch.set_alpha(0.4)
        near = "clear" if math.isinf(c["nearest"]) else f"{c['nearest']:.2f} m"
        turn = "LEFT" if f["w"] > 0 else "RIGHT" if f["w"] < 0 else "none"
        text.set_text(f"t={f['t']:5.1f}s  v={f['v']:.2f} m/s  turn={turn}  ({i + 1}/{len(frames)})\n"
                      f"nearest={near}  blocking={c['blocking']}  obstacles={c.get('obstacles', 0)}"
                      + (f" (+{c['ignored_obstacles']} ignored)" if c.get("ignored_obstacles") else "")
                      + ("  (side blocked: counter-turning)" if c.get("side_blocked") else "")
                      + ("  (intended turn kept being blocked: switched side)" if c.get("side_turn_given_up") else "")
                      + ("\nturn LOCKED on obstacle" if c.get("locked") else "")
                      + ("  (cones switched turn side)" if c.get("cone_switch") else "")
                      + ("\nDEAD END: virtual wall added" if c.get("deadend_added") else "")
                      + ("  (lock handed to next obstacle, same turn)" if c.get("lock_handover")
                         else "  (lock released: object left corridor)" if c.get("lock_released") else "")
                      + ("\nBACKING UP" + ("  (rear blocked, holding)" if c.get("rear_blocked") else "")
                         if c.get("backing") else "")
                      + (f"  side check r={c['side_check_radius']:.2f} m" if c.get("side_check_radius") else "")
                      + (f"\nside cones: left={c['cone_left']:.2f} m @{c.get('cone_left_angle', 0):+.0f}deg"
                         f"  right={c['cone_right']:.2f} m @{c.get('cone_right_angle', 0):+.0f}deg"
                         + (f"  (scores L={c['cone_left_score']:.2f} R={c['cone_right_score']:.2f}, goal side "
                            f"{ {1: 'left', -1: 'right', 0: '-'}[c['cone_goal_side']]})" if "cone_left_score" in c else "")
                         if "cone_left" in c else "")
                      + (f"\nGAP {c['gap_phase']}  width={c['gap_width']:.2f} m  off centre={c['gap_lateral']:+.2f} m"
                         if c.get("gap_phase") else "")
                      + (f"\nBOARDS {c['board_phase']}"
                         + (f"  board {c['board_index'] + 1}/2  axes found={len(c['board_axes'])}"
                            f"  moving tracks={c['board_tracks']}" if "board_axes" in c else "")
                         if c.get("board_phase") else "")
                      + ("\nCOLLISION" if f["collision"] else "")
                      + ("\nFINISHED" if f.get("finished") else ""))
        fig.canvas.draw_idle()

    # ---- controls ----
    slider = Slider(fig.add_axes([0.12, 0.13, 0.76, 0.03]), "time", 0, len(frames) - 1,
                    valinit=0, valstep=1, valfmt="%d")
    slider.valtext.set_text(f"{frames[0]['t']:.1f}s")
    speed = Slider(fig.add_axes([0.12, 0.085, 0.30, 0.025]), "speed", 0.25, 8.0, valinit=1.0, valfmt="%.2gx")
    b_stop = Button(fig.add_axes([0.08, 0.02, 0.12, 0.045]), "Stop")
    b_rew = Button(fig.add_axes([0.22, 0.02, 0.14, 0.045]), "<< Rewind")
    b_back = Button(fig.add_axes([0.38, 0.02, 0.08, 0.045]), "<")
    b_play = Button(fig.add_axes([0.48, 0.02, 0.14, 0.045]), "Play")
    b_fwd = Button(fig.add_axes([0.64, 0.02, 0.08, 0.045]), ">")

    state = {"direction": 0, "pos": 0.0}  # direction: +1 playing, -1 rewinding, 0 paused
    TICK_MS = 50

    def set_direction(direction):
        state["direction"] = direction
        b_play.label.set_text("Pause" if direction == +1 else "Play")
        b_rew.label.set_text("Pause" if direction == -1 else "<< Rewind")
        fig.canvas.draw_idle()

    def goto(i):
        i = int(np.clip(i, 0, len(frames) - 1))
        state["pos"] = float(i)
        if int(slider.val) != i:
            slider.set_val(i)  # triggers on_slider -> render
        else:
            render(i)

    def on_slider(val):
        i = int(val)
        if int(state["pos"]) != i:
            state["pos"] = float(i)  # Slider dragged by hand: continue playback from here
        slider.valtext.set_text(f"{frames[i]['t']:.1f}s")
        render(i)

    def tick():
        if state["direction"] == 0:
            return
        # Real time at 1x: one frame is DT seconds of simulation
        state["pos"] += state["direction"] * speed.val * (TICK_MS / 1000.0) / DT
        if state["pos"] >= len(frames) - 1:
            state["pos"] = len(frames) - 1
            set_direction(0)
        elif state["pos"] <= 0:
            state["pos"] = 0
            set_direction(0)
        i = int(state["pos"])
        if int(slider.val) != i:
            slider.set_val(i)

    def play(_=None):
        if state["direction"] == +1:
            set_direction(0)
        else:
            if int(slider.val) >= len(frames) - 1:
                goto(0)  # at the end: start again
            set_direction(+1)

    def rewind(_=None):
        set_direction(0 if state["direction"] == -1 else -1)

    def stop(_=None):
        set_direction(0)
        goto(0)

    def step(delta):
        set_direction(0)
        goto(int(slider.val) + delta)

    slider.on_changed(on_slider)
    b_play.on_clicked(play)
    b_rew.on_clicked(rewind)
    b_stop.on_clicked(stop)
    b_back.on_clicked(lambda _: step(-1))
    b_fwd.on_clicked(lambda _: step(+1))

    def on_key(event):
        if event.key == " ":
            play()
        elif event.key == "left":
            step(-1)
        elif event.key == "right":
            step(+1)
        elif event.key == "r":
            rewind()
        elif event.key == "s":
            stop()
    fig.canvas.mpl_connect("key_press_event", on_key)

    timer = fig.canvas.new_timer(interval=TICK_MS)
    timer.add_callback(tick)
    timer.start()
    fig._player = (slider, speed, b_stop, b_rew, b_back, b_play, b_fwd, timer)  # keep widgets alive

    render(0)
    set_direction(+1)  # start playing
    plt.show()  # blocks until the window is closed; then the next scenario opens


def overview(results):
    n = len(results)
    cols = min(4, n)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(4.5 * cols, 4.5 * rows), squeeze=False)
    for ax, (name, scn, frames) in zip(axes.flat, results):
        draw_world(ax, scn, name)
        ax.plot([f["x"] for f in frames], [f["y"] for f in frames], "b-", lw=1.5)
        last = frames[-1]
        ax.add_patch(Polygon(footprint(last["x"], last["y"], last["h"]), closed=True,
                             fc="crimson" if last["collision"] else "steelblue", ec="k"))
        ax.plot(frames[0]["x"], frames[0]["y"], "go")
        xs, ys = [f["x"] for f in frames], [f["y"] for f in frames]
        ax.set_xlim(min(xs) - 1.0, max(xs) + 1.5)
        ax.set_ylim(min(ys) - 1.5, max(ys) + 1.5)
    for ax in list(axes.flat)[n:]:
        ax.axis("off")
    fig.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", nargs="?", choices=list(SCENARIOS) + ["random"],
                        help="Run only this scenario ('random' = randomised Gazebo course)")
    parser.add_argument("--seed", type=int, default=None, help="Seed for 'random' (default: new random map)")
    parser.add_argument("--trials", type=int, default=1, help="Number of random courses to run for 'random'")
    parser.add_argument("--no-anim", action="store_true", help="Skip the animation, only show final paths")
    parser.add_argument("--real", action="store_true", help="Real-robot caps (0.5 m/s, 1.0 rad/s) and 720-beam scan")
    parser.add_argument("--swing", action="store_true", help="Boards swing like hand-pushed pendulums (decaying, bent) instead of motorised")
    parser.add_argument("--headless", action="store_true", help="Print results only, no plots")
    args = parser.parse_args()
    global SWING
    SWING = args.swing

    if args.scenario == "random":
        first = args.seed if args.seed is not None else int(np.random.default_rng().integers(1_000_000))
        runs = [(f"random_{seed}", randomised_fp_scenario(seed)) for seed in range(first, first + args.trials)]
    else:
        names = [args.scenario] if args.scenario else list(SCENARIOS)
        runs = [(name, SCENARIOS[name]) for name in names]
    results, statuses = [], []
    for name, scn in runs:
        frames = simulate(scn, real=args.real)
        statuses.append(summary(name, frames))
        results.append((name, scn, frames))
    if len(runs) > 1 and args.scenario == "random":
        done = sum(st.startswith("FINISHED") for st in statuses)
        hits = sum(st == "COLLISION" for st in statuses)
        spawn = sum(st == "SPAWN OVERLAP" for st in statuses)
        print(f"--> {done}/{len(runs)} finished, {hits} collisions, {spawn} spawned inside an obstacle, "
              f"{len(runs) - done - hits - spawn} timed out")

    if args.headless:
        return
    if args.no_anim:
        overview(results)
    else:
        for name, scn, frames in results:
            animate(name, scn, frames)


if __name__ == "__main__":
    main()
