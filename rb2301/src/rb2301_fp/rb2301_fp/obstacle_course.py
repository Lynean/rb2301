import argparse
import configparser
import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

np.set_printoptions(2, suppress=True, threshold=np.inf) # Print numpy arrays to specified d.p., suppress scientific notation and do not truncate
set_logger_level("obstaclecourse", level=LoggingSeverity.INFO) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  


_PACKAGE_DIR = os.path.dirname(os.path.realpath(__file__))

# ---- Speed tuning (change these first) ----
FORWARD_SPEED = 0.22           # Cruise speed when the corridor is clear (m/s)
MAX_TURN_SPEED = 0.8          # Cap on ALL turning: avoidance and heading hold (rad/s)
MIN_TURN_SPEED = 0.15         # Avoidance turning rate when the obstacle is at TURN_DISTANCE (rad/s);
                              # rises to MAX_TURN_SPEED at STOP_DISTANCE and closer
HEADING_GAIN = 0.8            # Heading hold: rad/s of turn per rad of heading error
BACKUP_SPEED = 0.1           # Reverse speed when stopped by an obstacle (m/s)

# ---- Front-corridor avoidance parameters ----
LASER_YAW_OFFSET = math.pi    # LiDAR is mounted facing backwards (URDF laser_joint yaw)
LASER_X_OFFSET = 0.11423      # LiDAR is this far ahead of the robot centre (m)
ROBOT_HALF_WIDTH = 0.17       # m
CORRIDOR_HALF_WIDTH = ROBOT_HALF_WIDTH + 0.02  # Rectangle half-width incl. margin (m)
CORRIDOR_LENGTH = 2.0         # Rectangle length ahead of the robot centre (m)
SLOWDOWN_DISTANCE = 0.6       # Start slowing when the nearest corridor scan is closer than this (m)
STOP_DISTANCE = 0.17           # Forward speed is 0 at this nearest-scan distance (m)
TURN_DISTANCE = 0.8           # Only turn when the nearest corridor scan is closer than this (m)
WHOLE_FRONT_COVERAGE = 0.8    # Fraction of corridor width occupied to count as "whole front" blocked
COVERAGE_BINS = 8             # Lateral bins used to measure that coverage
# New turn (any obstacle): compare two small scan cones on either side of the corridor, turn toward the deeper one
SIDE_CONE_ANGLE_DEG = 40.0    # Centre of each side's search sector, left and right of straight ahead (LiDAR view) (deg)
SIDE_CONE_HALF_WIDTH_DEG = 40.0  # Half-width of each side's search sector (deg)
SIDE_SUBCONE_HALF_WIDTH_DEG = 7.5  # Each side's depth = the deepest narrow sub-cone (this half-width) in its sector,
SIDE_SUBCONE_STEP_DEG = 15.0   # tried every this many degrees; the median range of a sub-cone is its depth
SIDE_CONE_MAX_RANGE = 6.0     # Beams with no return count as this deep (m)
SIDE_CONE_GOAL_BONUS = 0.20   # Cone score = depth, + this for the goal side (m): the side whose deepest sub-cone
                              # points closest to NORTH or EAST (smaller of its two angles; see side_cone_scores)
SIDE_CONE_TIE = 0.10          # Scores closer than this are a tie (m): partly blocked -> turn away from the
                              # obstacle, whole front blocked -> goal side
# While a turn is locked, the cones may overrule it: switch to the other side only if it is the side the
# pursued heading is on AND its score beats the current side's by a margin, for several scans in a row.
SIDE_CONE_SWITCH_MARGIN = 0.15  # Other side's score must beat the current side's by this much (m)
SIDE_CONE_SWITCH_SCANS = 5      # ...for this many control steps in a row
SIDE_CONE_SWITCH_COOLDOWN = 20  # No further switch for this many control steps after one (node: 20 per s)
SIDE_CONE_SWITCH_SLOW_DIFF = 0.50  # A switch where the other side's score beat the current one by at least this (m)...
SIDE_CONE_SWITCH_SLOW_SPEED = 0.10  # ...caps forward speed at this (a cap: no effect if already slower) (m/s)...
SIDE_CONE_SWITCH_SLOW_STEPS = 20    # ...for this many control steps (node: 20 per s)
MIN_EDGE_INTRUSION = 0.20     # A corridor obstacle only counts if it reaches in from the corridor edge by at least
                              # this fraction of the corridor width (edge -> obstacle's innermost point);
                              # slivers along the edges are ignored
OBSTACLE_GAP = 0.2           # Corridor points further apart than this belong to different obstacles (m);
                              # only the obstacle nearest the robot is used for the decision
OBJECT_MATCH_DISTANCE = 0.15  # Turn lock: an obstacle this close to where the locked obstacle was on the
                              # previous scan is the same object. The lock is released only when the
                              # locked object has no points left in the corridor.
TURN_CHECK_RADIUS = 0.14      # Side check while turning: obstacle within this distance of the BODY EDGE, in the
                              # front zone the turn sweeps (front-left for a left turn, front-right for right) (m)
REAR_SIDE_CLEARANCE = 0.05    # Same for the rear zone (rear-right for a left turn, rear-left for right); the rear
                              # barely swings out (body is only 0.034 m behind the axle), so this is small (m)
SIDE_CLEAR_HYSTERESIS = 0.05  # Once side blocked, keep counter-turning until the zone is clear by this much more (m)
SIDE_BLOCK_FLIP_COUNT = 10     # If the side check blocks the intended turn this many times in one lock, give the
                              # turn up: the lock switches to the counter-turn direction (away from the obstacle)
SIDE_COUNTER_TURN = 1.5       # Side blocked (an obstacle beside the robot, in the area the body sweeps when
                              # turning): turn AWAY
                              # from the obstacle's side at this fraction of the intended turn speed
                              # (the lock keeps its direction, so the intended turn resumes afterwards)
BACKUP_START_DISTANCE = STOP_DISTANCE + 0.02  # Start backing up when the nearest corridor scan is this close (m).
                              # (The slowdown makes the speed approach 0 but never quite reach it, so
                              # waiting for exactly v = 0 would never trigger.)
BACKUP_UNTIL_DISTANCE = STOP_DISTANCE + 0.10  # Once backing up, keep reversing until the nearest corridor
                              # scan is at least this far (m)
ROBOT_REAR = 0.034            # Body extends this far behind the odometry origin (rear axle) (m)
REAR_CLEARANCE = 0.10         # Don't reverse if anything is within this distance behind the body (m)
# Heading hold toward north-east while the corridor is clear (no avoidance turn active).
# North-east = the robot's front-right at the start: it spawns facing +x, so NE is world -45 deg.
TARGET_HEADING_DEG = -45.0    # World-frame heading to hold (deg) - SIMULATION
# Real robot: the Optitrack pose has +x pointing WEST and +y pointing SOUTH, so north = -y (-90 deg),
# east = -x (180 deg) and north-east = -135 deg.
REAL_TARGET_HEADING_DEG = -135.0  # Heading to hold on the REAL robot (deg, frame of self.pose)
HEADING_HOLD_CLEARANCE = TURN_DISTANCE  # Heading hold only steers when the corridor is clear beyond this (m)
# Which heading to hold: north-east if the scan toward it is open; else the open direction closest to NORTH or
# EAST (a detour), kept until north-east opens again or the detour direction gets blocked
HEADING_ON_COURSE_DEG = 110.0  # FIRST: already heading within this window around north-east (+-half of it)? Keep
                              # going straight - no steering, the next two rules are skipped (deg)
HEADING_OPEN_DEPTH = 1.0      # A direction is open if the median range of its cone is at least this (m)
HEADING_CONE_HALF_DEG = 7.5   # Half-width of the cone used to measure a direction (deg)
HEADING_CANDIDATE_STEP_DEG = 15.0  # Detour directions tried every this many degrees, all round (deg)

# ---- Gate: detected in front (line up, drive through) or behind (already through); then the boards ----
GAP_DETECT_RANGE = 1.2        # Gate detection uses the WHOLE 360 deg scan, every point closer than this (m)
GAP_MIN_WIDTH = 0.30          # Narrowest gap to use (robot is 0.22 m wide) (m); the course gate is 0.40 m
GAP_MAX_WIDTH = 0.80          # Widest opening still treated as a "gap" (m)
# The gate is a gap between the ENDS of two straight walls. Each wall can run along the world X or Y axis
# (all its points within GATE_WALL_TOLERANCE of one line through the end point). The gap itself (end to
# end) must run East-West within GATE_GAP_ALIGN_DEG. Looked for in the whole scan every step, then sorted by
# where the gap is relative to the robot (not by the robot's heading):
#   BACK  = gap centre in the SOUTH half (south of the robot): start the board crossing right away
#   FRONT = gap centre in the NORTH half: line up with it and drive through (APPROACH/ALIGN/TURN/GO),
#           then start the board crossing
NORTH_DEG = 0.0               # SIMULATION: world direction of NORTH (deg). Robot spawns facing +x = north
                              # (TARGET_HEADING_DEG -45 = north-east), so east is -y
REAL_NORTH_DEG = -90.0        # REAL robot: Optitrack +y is SOUTH, so north is -y (-90 deg) and east is -x
GATE_WALL_TOLERANCE = 0.1    # Wall-end points within +- this of an X or Y line through the end belong to it (m)
GATE_WALL_LENGTH = 0.50       # Look at up to this much wall from each end (m)
GATE_WALL_MIN_LENGTH = 0.30   # Each wall end must show at least this much straight wall - a LONG wall (m)
# The gate line must be CLOSED: the only scans beyond it are the ones through the gap itself. A scan point more
# than GATE_CLOSED_DEPTH beyond the line whose beam crossed the line OUTSIDE the gap (within GATE_CLOSED_EXTENT of
# its centre) means the line is not a wall there - not a gate (e.g. a box's corner and the wall it hides).
GATE_CLOSED_DEPTH = 0.20      # (m)
GATE_CLOSED_EXTENT = 1.0      # (m)
GATE_CLOSED_EDGE_MARGIN = 0.03  # Beams this close outside the gap's edges still count as through the gap (m)
GATE_WALL_MIN_POINTS = 4      # ...with at least this many points
GATE_WALL_END_SKIP = 0.10     # The straight wall may start up to this far from the gap edge: the bent lead-in is the
                              # wall's END FACE (its thickness), seen at a steep angle e.g. looking back after the gate (m)
GATE_WALL_STRAIGHTNESS = 0.015  # ...and within this of their own best-fit line (a big cylinder's edge is within
                              # the 6 cm band over 0.3 m, but bulges ~3 cm from a straight line) (m)
GATE_WALL_STEP = 0.08         # Neighbouring points on one wall are closer than this (m)...
GATE_WALL_STEP_PER_M = 0.04   # ...or than this per metre of range, if more (far away the beams spread apart) (m/m)
GATE_GAP_ALIGN_DEG = 10.0     # The gap (end to end) must run East-West within this (deg)
# Gate IN FRONT: line up with it, face it, drive through (APPROACH -> ALIGN -> TURN -> GO), then the boards
GAP_MIN_STANDOFF = 0.10       # ALIGN: line up on the gap's centre line at least this far before it (m)
GAP_MAX_STANDOFF = 0.20       # ...and at most this far (m)
GAP_ALIGN_TOLERANCE = 0.02    # ALIGN is done when within this of the centre line (m)
GAP_ALIGN_SPEED = 0.15        # Forward speed while aligning (m/s)
GAP_SPEED = 0.25               # GO: drive through the gap at this speed (m/s); move_2D caps it at 0.3 on the real robot
GAP_STOP_DISTANCE = 0.20      # GO: wait if anything is this close straight ahead of the body (e.g. a moving box) (m)
GAP_MAX_CALLS = 600           # Give up on a gap after this many control steps (node: 20 per s)
GAP_ALIGN_START_DISTANCE = 0.20  # APPROACH hands over to ALIGN within this distance of the alignment point (m)
GAP_APPROACH_PROGRESS = 0.05  # APPROACH must get this much closer to the alignment point...
GAP_APPROACH_STALL_CALLS = 100  # ...within this many control steps (node: 20 per s), or it gives up on the gap
GAP_OPEN_DEPTH = 0.30         # The space this deep behind the gap must be empty (m)
GAP_ALIGN_GAIN = 1.5          # ALIGN / GO steering gain (rad/s per rad)
GAP_FACE_TOLERANCE_DEG = 4.0  # TURN is done when facing the gap within this (deg)
BODY_HALF_WIDTH = 0.11        # Real body half-width (wheels), used for the GO stop strip (m)
BODY_FRONT = 0.181            # Front of the body ahead of the rear axle (m)
GATE_GO_PAST = 0.2           # GO keeps driving north until the body's REAR is this far past the gate line, then
                              # the board crossing starts (scan, east wall, ...) (m)
GAP_SAME_DISTANCE = 0.60      # A gap whose centre is within this of an already-passed gap is ignored (m)
GAP_RETRY_CALLS = 200         # A gap the manoeuvre gave up on is ignored for this many control steps, then
                              # tried again (node: 20 per s)

# ---- Dead-end memory: block a spot where the robot keeps shoving in and backing out ----
DEADEND_BACKUPS = 5           # Back-ups at the same spot before it is declared a dead end
DEADEND_RADIUS = 0.30         # "Same spot": robot front within this distance of each other (m)
DEADEND_HEADING_DEG = 60.0    # ...and facing within this angle of each other (deg)
DEADEND_WALL_WIDTH = 0.60     # A virtual wall this wide is placed across the dead end (m)
DEADEND_WALL_OFFSET = 0.05    # ...this far ahead of where the bumper was when it backed up (m)
DEADEND_MAX_WALLS = 10        # Keep at most this many virtual walls (oldest dropped)

# ---- Moving boards after the gate (board_crossing_command, only active after passing a gap) ----
# The real boards are SWUNG by hand (pendulums), not motorised: the swing slows down and gets shorter over
# time, and the boards are not perfectly flat. So nothing below relies on the board reaching the far ends
# it reached earlier: only the last BOARD_RECENT_TIME seconds of its motion are used.
BOARD_SCAN_DEPTH = 1.6        # Look for boards up to this far ahead along the travel direction (m)
BOARD_SCAN_HALF_WIDTH = 1.0   # ...and this far to each side (m)
BOARD_MAX_LENGTH = 0.45       # A board seen by the LiDAR is at most this long sideways (boards are 0.3 m) (m)
BOARD_MAX_DEPTH = 0.15        # ...and at most this deep along the travel direction (0.05 m board, but bent / tilted) (m)
BOARD_DEPTH_ERROR = 0.08      # Detections within 8% of their distance (min BOARD_DEPTH_TOLERANCE) belong to the same travel axis
BOARD_DEPTH_TOLERANCE = 0.06  # Minimum of that tolerance (m) - loose, because the boards are not flat
BOARD_MIN_DETECTIONS = 15     # A travel axis is confident after this many detections...
BOARD_MIN_TRAVEL = 0.20       # ...covering at least this much sideways travel (m)
BOARD_MOVING_TRAVEL = 0.10    # A track that moved at least this far sideways is a moving board (m)
BOARD_COUNT = 2               # Boards to cross. SCAN looks for ONE board at a time and runs again after each
                              # crossing, until this many are crossed
BOARD_SCAN_FALLBACK_TIME = 12.0  # SCAN: after this long, take the nearest board that moved at all, even if not confident (s)
BOARD_NONE_TIME = 6.0         # SCAN: NOTHING moving at all within this long -> no (more) boards, drive on normally (s).
                              # On the first scan that means a FALSE gate: it is ignored from then on and gate
                              # detection carries on looking for the real one
BOARD_HALF_THICKNESS = 0.025  # Half the board thickness: the LiDAR sees its near face (m)
BOARD_WAIT_GAP = 0.08         # APPROACH stops with the body front this far before a board's near face (m)
BOARD_APPROACH_SPEED = 0.15   # Speed while moving up to a board's axis (m/s)
BOARD_PATH_MARGIN = 0.05      # Robot's path strip: body half-width + this on each side (m)
BOARD_CROSS_SPEED = 0.4       # Speed while crossing a board's axis (m/s); move_2D caps it at 0.3 on the real robot
BOARD_CLEAR_MARGIN = 0.05     # CROSS ends when the body's rear is this far past the board's far face (m)
# SIDE: before the first board, go EAST to the east WALL: turn to face east, drive until the wall is just
# ahead, turn back north. A pendulum sweeps the centre twice per swing at full speed; near the side it only
# comes by once per swing - and as the swing dies down it stops reaching the side at all.
BOARD_SIDE_OFFSET = 1.0       # SIDE: drive east at most this far (0 disables SIDE) (m)
BOARD_SIDE_WALL_CLEARANCE = 0.05  # ...stopping with the body front this far from the east wall (after turning
                              # north, the body side is then ~0.17 m from it) (m)
BOARD_SIDE_SPEED = 0.15       # Speed while driving east (m/s)
BOARD_SIDE_MAX_TIME = 15.0    # Give up driving east after this long and carry on from there (s)
# WAIT: when is it safe to cross?
BOARD_RECENT_TIME = 6.0       # Only the board's motion in the last this-many seconds counts (swing decays) (s)
BOARD_MOVING_SPEED = 0.03     # Board slower than this sideways counts as not moving (m/s)
BOARD_STILL_TRAVEL = 0.04     # Board that moved less than this in the last 3 s has stopped (m)
BOARD_LOST_TIME = 2.0         # Board not seen for this long, last seen off the path -> cross (s)
BOARD_LOST_CLEARANCE = 0.15   # ...only if it was last seen at least this far beside the path (m)
BOARD_WAIT_MAX_TIME = 30.0    # WAIT timeout: cross if the path is clear, else give up and drive normally (s)


def scan_to_robot_points(msg) -> tuple:
    '''Convert a LaserScan into robot-frame (x forward, y left) points and their ranges.'''
    ranges = np.asarray(msg.ranges, dtype=float)
    angles = float(msg.angle_min) + np.arange(ranges.size) * float(msg.angle_increment) + LASER_YAW_OFFSET
    valid = np.isfinite(ranges) & (ranges > max(float(msg.range_min), 0.01))
    if math.isfinite(float(msg.range_max)):
        valid &= ranges <= float(msg.range_max)
    r, a = ranges[valid], angles[valid]
    points = np.column_stack((LASER_X_OFFSET + r * np.cos(a), r * np.sin(a)))
    return points, r


def analyse_front_corridor(points:np.ndarray, ranges:np.ndarray) -> dict:
    '''Find the nearest obstacle in the front corridor and label how it blocks it.'''
    inside = (points[:, 0] > 0.0) & (points[:, 0] <= CORRIDOR_LENGTH) & (np.abs(points[:, 1]) <= CORRIDOR_HALF_WIDTH)
    result = {"nearest": math.inf, "blocking": "clear", "turn": 0, "obstacles": 0,
              "whole": False, "clusters": [], "first_points": np.empty((0, 2))}
    if not np.any(inside):
        return result
    pts, rng = points[inside], ranges[inside]

    # Split the corridor points into separate obstacles: order them by bearing across the
    # corridor (continuous in front of the robot, unlike the raw scan order, which wraps
    # at the front because the LiDAR faces backwards) and cut wherever neighbours are more
    # than OBSTACLE_GAP apart. Keep only the obstacle that contains the nearest point.
    order = np.argsort(np.arctan2(pts[:, 1], pts[:, 0]))
    pts, rng = pts[order], rng[order]
    gaps = np.hypot(*np.diff(pts, axis=0).T) > OBSTACLE_GAP
    labels = np.concatenate(([0], np.cumsum(gaps)))

    # Only obstacles that really block count: they must reach in from the corridor edge (to their
    # innermost point, closest to the centre line) by at least MIN_EDGE_INTRUSION of the corridor width.
    # Slivers grazing the corridor edges are ignored; anything across the middle always counts.
    width = 2.0 * CORRIDOR_HALF_WIDTH
    blocking = []
    for k in range(int(labels[-1]) + 1):
        lat = pts[labels == k][:, 1]
        crosses_middle = lat.min() < 0.0 < lat.max()
        intrusion = CORRIDOR_HALF_WIDTH - (0.0 if crosses_middle else float(np.min(np.abs(lat))))
        if intrusion >= MIN_EDGE_INTRUSION * width:
            blocking.append(k)
    result["ignored_obstacles"] = int(labels[-1]) + 1 - len(blocking)
    if not blocking:
        return result  # Only edge slivers: the way is clear
    keep = np.isin(labels, blocking)
    pts, rng, labels = pts[keep], rng[keep], labels[keep]

    first = labels[int(np.argmin(rng))]
    result["obstacles"] = len(blocking)
    result["clusters"] = [pts[labels == k] for k in blocking]
    pts, rng = pts[labels == first], rng[labels == first]
    result["first_points"] = pts
    result["nearest"] = float(np.min(rng))

    # How much of the corridor width is occupied
    bins = np.floor((pts[:, 1] + CORRIDOR_HALF_WIDTH) / (2 * CORRIDOR_HALF_WIDTH) * COVERAGE_BINS)
    coverage = np.unique(np.clip(bins, 0, COVERAGE_BINS - 1)).size / COVERAGE_BINS

    if coverage < WHOLE_FRONT_COVERAGE:
        # Partial block: which side is the obstacle on? (y < 0 is the robot's right)
        if float(np.mean(pts[:, 1])) < 0.0:
            result["blocking"], result["turn"] = "right", +1
        else:
            result["blocking"], result["turn"] = "left", -1
        return result

    # Whole front blocked: the side cones decide the turn in _obstacle_avoidance (goal side on a tie);
    # turn right only if there is no heading to pick a goal side from
    result["blocking"], result["turn"], result["whole"] = "whole", -1, True
    return result


def side_cone_depths(scan_msg, with_angles:bool=False) -> tuple:
    '''Measure how open each side of the corridor is (deepest sub-cone per side).'''
    lo = SIDE_CONE_ANGLE_DEG - SIDE_CONE_HALF_WIDTH_DEG + SIDE_SUBCONE_HALF_WIDTH_DEG
    hi = SIDE_CONE_ANGLE_DEG + SIDE_CONE_HALF_WIDTH_DEG - SIDE_SUBCONE_HALF_WIDTH_DEG
    centres = np.arange(lo, max(hi, lo) + 1e-6, SIDE_SUBCONE_STEP_DEG)

    def deepest(side):
        '''Deepest sub-cone on one side: (depth, bearing deg).'''
        depths = _cone_depths(scan_msg, side * centres, SIDE_SUBCONE_HALF_WIDTH_DEG)
        k = int(np.argmax(depths))
        return float(depths[k]), side * float(centres[k])

    left_depth, left_angle = deepest(+1)
    right_depth, right_angle = deepest(-1)
    if with_angles:
        return left_depth, right_depth, left_angle, right_angle
    return left_depth, right_depth


def side_cone_scores(scan_msg, heading_deg, target_heading_deg, corridor, north_deg=None) -> tuple:
    '''Score each side for a turn: its cone depth plus a bonus for the goal side.'''
    left_depth, right_depth, left_angle, right_angle = side_cone_depths(scan_msg, with_angles=True)
    goal_side = 0
    if heading_deg is not None:
        goal_side = +1 if wrap_deg(target_heading_deg - float(heading_deg)) > 0.0 else -1
        if north_deg is not None:
            def off_north_east(cone_angle):
                '''Smaller of the angles from this cone's world direction to north and to east.'''
                world = float(heading_deg) + cone_angle
                return min(abs(wrap_deg(world - north_deg)), abs(wrap_deg(world - (north_deg - 90.0))))
            left_off, right_off = off_north_east(left_angle), off_north_east(right_angle)
            if abs(left_off - right_off) > 1.0:
                goal_side = +1 if left_off < right_off else -1
    left_score = left_depth + (SIDE_CONE_GOAL_BONUS if goal_side > 0 else 0.0)
    right_score = right_depth + (SIDE_CONE_GOAL_BONUS if goal_side < 0 else 0.0)
    corridor.update({"cone_left": left_depth, "cone_right": right_depth,
                     "cone_left_angle": left_angle, "cone_right_angle": right_angle,
                     "cone_left_score": left_score, "cone_right_score": right_score, "cone_goal_side": goal_side})
    return left_score, right_score, goal_side


def _min_distance(a:np.ndarray, b:np.ndarray) -> float:
    '''Smallest distance between two point sets.'''
    if len(a) == 0 or len(b) == 0:
        return math.inf
    return float(np.min(np.hypot(a[:, None, 0] - b[None, :, 0], a[:, None, 1] - b[None, :, 1])))


def _sweep_zones(points:np.ndarray, front_radius:float, rear_radius:float) -> tuple:
    '''Find the points the body would sweep into when turning left or right.'''
    # Distance from each point to the body outline (rectangle: ROBOT_REAR behind to BODY_FRONT ahead of the rear
    # axle, +-BODY_HALF_WIDTH wide); 0 inside the body. Turning about the rear axle, a LEFT turn sweeps the
    # front-left and rear-right of the body, a RIGHT turn the front-right and rear-left.
    dx = np.maximum.reduce([-ROBOT_REAR - points[:, 0], np.zeros(len(points)), points[:, 0] - BODY_FRONT])
    dy = np.maximum(np.abs(points[:, 1]) - BODY_HALF_WIDTH, 0.0)
    dist = np.hypot(dx, dy)
    near_front, near_rear = dist < front_radius, dist < rear_radius
    front, rear = points[:, 0] > 0.0, points[:, 0] <= 0.0
    left, right = points[:, 1] > 0.0, points[:, 1] < 0.0
    left_zone = (near_front & front & left) | (near_rear & rear & right)
    right_zone = (near_front & front & right) | (near_rear & rear & left)
    return left_zone, right_zone, dist


def _sweep_threat(points:np.ndarray, direction:int, front_radius:float=TURN_CHECK_RADIUS,
                  rear_radius:float=REAR_SIDE_CLEARANCE) -> int:
    '''Side of the nearest point the body would hit when turning, or 0 if clear.'''
    left_zone, right_zone, dist = _sweep_zones(points, front_radius, rear_radius)
    if not np.any(left_zone if direction > 0 else right_zone):
        return 0
    # Turn away from the NEAREST point in either sweep zone (not only the turn's own): there may be something
    # even closer on the other side, and turning away from the farther one would drive into it.
    both = left_zone | right_zone
    return +1 if points[both][int(np.argmin(dist[both]))][1] > 0.0 else -1


def _cone_depths(scan_msg, centres_deg, half_deg:float=HEADING_CONE_HALF_DEG) -> np.ndarray:
    '''Depth (median range) of narrow cones at the given bearings.'''
    ranges = np.asarray(scan_msg.ranges, dtype=float)
    angles = float(scan_msg.angle_min) + np.arange(ranges.size) * float(scan_msg.angle_increment) + LASER_YAW_OFFSET
    valid = np.isfinite(ranges) & (ranges > max(float(scan_msg.range_min), 0.01))
    depth = np.where(valid, np.minimum(ranges, SIDE_CONE_MAX_RANGE), SIDE_CONE_MAX_RANGE)
    out = []
    for centre_deg in np.atleast_1d(centres_deg):
        diff = (angles - math.radians(float(centre_deg)) + math.pi) % (2.0 * math.pi) - math.pi
        in_cone = np.abs(diff) <= math.radians(half_deg)
        out.append(float(np.median(depth[in_cone])) if np.any(in_cone) else SIDE_CONE_MAX_RANGE)
    return np.array(out)


def choose_goal_heading(scan_msg, heading_deg:float, target_deg:float, north_deg:float, state:dict) -> tuple:
    '''Choose the heading to hold when the corridor is clear (on course, north-east, or a detour).'''
    if abs(wrap_deg(heading_deg - target_deg)) <= HEADING_ON_COURSE_DEG / 2.0:
        state["detour_deg"] = None
        return heading_deg, "on course (keep going)"
    if _cone_depths(scan_msg, [wrap_deg(target_deg - heading_deg)])[0] >= HEADING_OPEN_DEPTH:
        state["detour_deg"] = None
        return target_deg, "north-east"
    detour = state.get("detour_deg")
    if detour is not None and _cone_depths(scan_msg, [wrap_deg(detour - heading_deg)])[0] >= HEADING_OPEN_DEPTH:
        return detour, f"detour {detour:+.0f} deg"
    world = np.arange(-180.0, 180.0, HEADING_CANDIDATE_STEP_DEG)
    depths = _cone_depths(scan_msg, [wrap_deg(w - heading_deg) for w in world])
    best, best_key = None, None
    for w, d in zip(world, depths):
        if d < HEADING_OPEN_DEPTH:
            continue
        off = min(abs(wrap_deg(w - north_deg)), abs(wrap_deg(w - (north_deg - 90.0))))
        key = (off, -d)
        if best_key is None or key < best_key:
            best, best_key = float(w), key
    state["detour_deg"] = best
    if best is None:
        return target_deg, "north-east (nothing open)"
    return best, f"detour {best:+.0f} deg"


def _obstacle_avoidance(scan_msg, lock, heading_deg=None, target_heading_deg=None, north_deg=None) -> tuple:
    '''One step of corridor obstacle avoidance: speed, turn choice, turn lock and safety checks.'''
    if target_heading_deg is None:
        target_heading_deg = TARGET_HEADING_DEG
    points, ranges = scan_to_robot_points(scan_msg)
    corridor = analyse_front_corridor(points, ranges)
    nearest = corridor["nearest"]

    # Forward speed: full when far, linear slowdown, 0 at STOP_DISTANCE
    scale = (nearest - STOP_DISTANCE) / (SLOWDOWN_DISTANCE - STOP_DISTANCE)
    forward = FORWARD_SPEED * float(np.clip(scale, 0.0, 1.0))

    # Track the locked obstacle: is it still somewhere in the corridor?
    corridor["lock_released"] = False
    if lock is not None:
        previous = lock["points"]
        if heading_deg is not None and lock.get("heading") is not None:
            # The robot turned since the last scan: rotate the old points into the new robot frame
            d = math.radians(wrap_deg(float(heading_deg) - lock["heading"]))
            c, s_ = math.cos(d), math.sin(d)
            previous = np.column_stack((c * previous[:, 0] + s_ * previous[:, 1],
                                        -s_ * previous[:, 0] + c * previous[:, 1]))
        matches = [(_min_distance(cluster, previous), cluster) for cluster in corridor["clusters"]]
        matches = [m for m in matches if m[0] <= OBJECT_MATCH_DISTANCE]
        if matches:
            same_object = min(matches, key=lambda m: m[0])[1]
            lock = {**lock, "points": same_object, "heading": heading_deg}  # Keeps direction and backing
        else:
            released = lock  # The locked obstacle has left the corridor
            lock = None
            corridor["lock_released"] = True

    # No lock: start one when the first obstacle is within TURN_DISTANCE
    corridor["lock_handover"] = False
    if lock is None and nearest < TURN_DISTANCE:
        if corridor["lock_released"]:
            # Hand-over: the locked obstacle just left but another one is already close. Keep turning
            # the same way (and keep backing up if it was) instead of re-deciding from the new
            # obstacle, so the robot doesn't swing back and forth between two obstacles.
            lock = {**released, "points": corridor["first_points"], "heading": heading_deg}
            corridor["lock_handover"] = True
        else:
            # Any obstacle: turn toward the side with the higher cone score (depth, + a bonus for the
            # goal side). On a tie: partly blocked -> turn away from the obstacle; whole front blocked
            # -> goal side.
            direction = corridor["turn"]
            left_score, right_score, goal_side = side_cone_scores(scan_msg, heading_deg, target_heading_deg, corridor, north_deg)
            if abs(left_score - right_score) > SIDE_CONE_TIE:
                direction = +1 if left_score > right_score else -1
            elif corridor["whole"] and goal_side != 0:
                direction = goal_side
            lock = {"direction": direction, "points": corridor["first_points"], "heading": heading_deg}

    # Locked turn: let the side cones overrule it when the other side is clearly better - much deeper
    # AND toward the pursued heading - for several steps in a row. Cooldown after a switch.
    corridor["cone_switch"] = False
    if lock is not None and heading_deg is not None:
        if lock.get("cooldown", 0) > 0:
            lock["cooldown"] -= 1
            lock["switch_count"] = 0
        else:
            left_score, right_score, goal_side = side_cone_scores(scan_msg, heading_deg, target_heading_deg, corridor, north_deg)
            other = -lock["direction"]
            other_score, own_score = (left_score, right_score) if other > 0 else (right_score, left_score)
            if goal_side == other and other_score >= own_score + SIDE_CONE_SWITCH_MARGIN:
                lock["switch_count"] = lock.get("switch_count", 0) + 1
                if lock["switch_count"] >= SIDE_CONE_SWITCH_SCANS:
                    lock["direction"] = other
                    lock["switch_count"] = 0
                    lock["cooldown"] = SIDE_CONE_SWITCH_COOLDOWN
                    corridor["cone_switch"] = True
                    if other_score - own_score >= SIDE_CONE_SWITCH_SLOW_DIFF:
                        lock["slow_steps"] = SIDE_CONE_SWITCH_SLOW_STEPS  # Big swing: slow down for it
            else:
                lock["switch_count"] = 0

    turn_direction = lock["direction"] if lock is not None else 0
    corridor["locked"] = lock is not None

    # Back up: when the obstacle has (practically) stopped the robot, reverse slowly while still
    # turning, until the nearest corridor scan is at least BACKUP_UNTIL_DISTANCE away. Never reverse
    # into something directly behind the body.
    corridor["backing"] = corridor["rear_blocked"] = False
    if lock is not None:
        if nearest <= BACKUP_START_DISTANCE:
            lock["backing"] = True
        elif nearest >= BACKUP_UNTIL_DISTANCE:
            lock["backing"] = False
        if lock.get("backing"):
            behind = ((points[:, 0] < -ROBOT_REAR) & (points[:, 0] > -ROBOT_REAR - REAR_CLEARANCE)
                      & (np.abs(points[:, 1]) <= CORRIDOR_HALF_WIDTH))
            corridor["rear_blocked"] = bool(np.any(behind))
            forward = 0.0 if corridor["rear_blocked"] else -BACKUP_SPEED
            corridor["backing"] = True

    # After a big cone switch: cap the forward speed for a while (a cap - no effect if already slower,
    # and backing up is not touched)
    corridor["switch_slow"] = False
    if lock is not None and lock.get("slow_steps", 0) > 0:
        lock["slow_steps"] -= 1
        if forward > SIDE_CONE_SWITCH_SLOW_SPEED:
            forward = SIDE_CONE_SWITCH_SLOW_SPEED
            corridor["switch_slow"] = True

    # Turning rate grows as the obstacle gets closer (the opposite of the forward speed):
    # MIN_TURN_SPEED at TURN_DISTANCE, MAX_TURN_SPEED at STOP_DISTANCE and closer.
    closeness = float(np.clip((TURN_DISTANCE - nearest) / (TURN_DISTANCE - STOP_DISTANCE), 0.0, 1.0))
    turn_speed = MIN_TURN_SPEED + (MAX_TURN_SPEED - MIN_TURN_SPEED) * closeness

    # Side check: the corridor only looks ahead, so before/while turning, make sure the body is
    # not about to sweep into something. Turning on the spot about the rear axle, the body sweeps:
    #   turning LEFT : front-left (front corner swings forward-left) and rear-right
    #   turning RIGHT: front-right and rear-left
    # Only points in those zones count, measured from the body EDGE: within TURN_CHECK_RADIUS (front
    # zone) or REAR_SIDE_CLEARANCE (rear zone). E.g. something behind-right is never hit by a right turn. If the current turn's zone has something, the robot turns the
    # opposite way at SIDE_COUNTER_TURN x the intended speed until that zone is clear again; the lock
    # keeps its direction, so the intended turn resumes afterwards. Forward / back-up motion continues.
    corridor["side_blocked"] = corridor["side_turn_given_up"] = False
    holding = lock is not None and lock.get("side_hold", False)  # Was side blocked on the previous step
    margin = SIDE_CLEAR_HYSTERESIS if holding else 0.0  # Hysteresis: must clear by a bit more to release
    corridor["side_check_radius"] = TURN_CHECK_RADIUS + margin if turn_direction != 0 else 0.0
    if turn_direction != 0:
        threat_side = _sweep_threat(points, turn_direction, TURN_CHECK_RADIUS + margin, REAR_SIDE_CLEARANCE + margin)
        if threat_side != 0:
            corridor["side_blocked"], corridor["threat_side"] = True, threat_side

    if lock is not None:
        if corridor["side_blocked"] and not holding:
            # A new side block of the intended turn. Too many in one lock: the intended turn can't be
            # done here, so switch the lock to the counter-turn direction (away from the obstacle).
            lock["side_events"] = lock.get("side_events", 0) + 1
            if lock["side_events"] >= SIDE_BLOCK_FLIP_COUNT:
                lock["direction"] = -corridor["threat_side"]
                lock["side_events"] = 0
                lock["cooldown"] = SIDE_CONE_SWITCH_COOLDOWN  # Cones may not flip it straight back
                turn_direction = lock["direction"]
                corridor["side_blocked"] = False  # Now turning away from it: no counter-turn needed
                corridor["side_turn_given_up"] = True
        lock["side_hold"] = corridor["side_blocked"]

    # Side blocked: turn AWAY from the obstacle's side at SIDE_COUNTER_TURN x the intended turn speed,
    # until clear. The lock keeps its direction, so the intended turn resumes afterwards.
    turn = turn_direction * turn_speed
    if corridor["side_blocked"]:
        turn = -corridor["threat_side"] * turn_speed * SIDE_COUNTER_TURN

    # Heading hold: with no obstacle within TURN_DISTANCE, steer toward target_heading_deg (north-east, or a
    # detour from choose_goal_heading).
    if lock is None and heading_deg is not None and nearest >= HEADING_HOLD_CLEARANCE:
        error = math.radians(wrap_deg(target_heading_deg - float(heading_deg)))
        turn = float(np.clip(HEADING_GAIN * error, -MAX_TURN_SPEED, MAX_TURN_SPEED))
        # Side check for the heading-hold turn too: don't steer the body into something beside it
        # (e.g. a wall corner that is ignored by the corridor because it only grazes the edge).
        if turn != 0.0:
            threat_side = _sweep_threat(points, +1 if turn > 0 else -1)
            if threat_side != 0:
                turn = -threat_side * abs(turn) * SIDE_COUNTER_TURN
                corridor["side_blocked"], corridor["threat_side"] = True, threat_side
    return forward, turn, lock, corridor

def _bearing_sorted(points:np.ndarray, max_range:float) -> np.ndarray:
    '''Nearby scan points sorted by bearing as seen from the LiDAR.'''
    near = points[np.hypot(points[:, 0], points[:, 1]) < max_range]
    return near[np.argsort(np.arctan2(near[:, 1], near[:, 0] - LASER_X_OFFSET))]


def _wall_step(p:np.ndarray) -> np.ndarray:
    '''Largest neighbour spacing still counted as the same wall at each point.'''
    r = np.hypot(p[..., 0] - LASER_X_OFFSET, p[..., 1])
    return np.maximum(GATE_WALL_STEP, GATE_WALL_STEP_PER_M * r)


def _axis_wall_end(near:np.ndarray, i:int, step:int, axes):
    '''Check whether a point is the end of a long, straight, X- or Y-axis wall.'''
    end, j, piece = near[i], i, [near[i]]
    while 0 <= j + step < len(near):
        nxt = near[j + step]
        if np.hypot(*(nxt - near[j])) > _wall_step(near[j]) or np.hypot(*(nxt - end)) > GATE_WALL_LENGTH:
            break
        piece.append(nxt)
        j += step
    if len(piece) < GATE_WALL_MIN_POINTS:
        return None
    piece = np.array(piece)
    from_end = np.hypot(*(piece - end).T)

    def straight(run, k):
        '''Are the first k points of the run on a straight line?'''
        centred = run[:k] - run[:k].mean(axis=0)
        fit_normal = np.linalg.svd(centred, full_matrices=False)[2][1]
        return np.max(np.abs(centred @ fit_normal)) <= GATE_WALL_STRAIGHTNESS

    # The straight run may start up to GATE_WALL_END_SKIP from the end: a real wall has THICKNESS, and seen
    # at a steep angle (e.g. looking back right after passing the gate) its end face shows as a short bent
    # lead-in before the wall's face. The run is kept only as long as it stays straight, so the walk stops
    # at a corner (a short gate wall meeting another wall). A cylinder's edge is never straight.
    for start in range(len(piece)):
        if from_end[start] > GATE_WALL_END_SKIP or len(piece) - start < GATE_WALL_MIN_POINTS:
            break
        run = piece[start:]
        if not straight(run, GATE_WALL_MIN_POINTS):
            continue
        lo, hi = GATE_WALL_MIN_POINTS, len(run)  # straight(lo) holds; find the largest such k (binary search)
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if straight(run, mid):
                lo = mid
            else:
                hi = mid - 1
        run = run[:lo]
        rel = run - run[0]
        for along in axes:
            across = np.array((-along[1], along[0]))
            if np.all(np.abs(rel @ across) <= GATE_WALL_TOLERANCE) and np.ptp(rel @ along) >= GATE_WALL_MIN_LENGTH:
                return run
    return None


def _line_closed(points:np.ndarray, a:np.ndarray, b:np.ndarray) -> bool:
    '''Check that the gate line is a solid wall apart from the gap itself.'''
    lidar = np.array((LASER_X_OFFSET, 0.0))
    centre = (a + b) / 2.0
    width = float(np.hypot(*(b - a)))
    along_dir = (b - a) / width
    normal = np.array((-along_dir[1], along_dir[0]))
    lidar_depth = float((lidar - centre) @ normal)
    if lidar_depth > 0.0:
        normal, lidar_depth = -normal, -lidar_depth  # Normal points away from the LiDAR: "beyond" the line
    if lidar_depth > -1e-3:
        return False  # LiDAR on the line itself: can't tell
    depth = (points - centre) @ normal
    beyond = points[depth > GATE_CLOSED_DEPTH]
    if not beyond.size:
        return True
    # Where each beam (LiDAR -> point) crosses the gate line, as a position along the line from the gap centre
    f = lidar_depth / (lidar_depth - depth[depth > GATE_CLOSED_DEPTH])
    crossing = lidar + f[:, None] * (beyond - lidar)
    u = np.abs((crossing - centre) @ along_dir)
    outside = (u > width / 2.0 + GATE_CLOSED_EDGE_MARGIN) & (u <= GATE_CLOSED_EXTENT)
    return not bool(np.any(outside))


def detect_gaps(points:np.ndarray, heading_deg:float, north_deg:float) -> list:
    '''Find gate candidates in the scan: East-West gaps between two straight wall ends.'''
    th = math.radians(heading_deg)
    c, s = math.cos(th), math.sin(th)
    axes = (np.array((c, -s)), np.array((s, c)))  # World X and Y axes in the robot frame
    east_west = math.radians(north_deg + 90.0) - th
    east_west = np.array((math.cos(east_west), math.sin(east_west)))
    min_cos = math.cos(math.radians(GATE_GAP_ALIGN_DEG))
    near = _bearing_sorted(points, GAP_DETECT_RANGE)
    n = len(near)
    if n < 2 * GATE_WALL_MIN_POINTS:
        return []
    lidar = np.array((LASER_X_OFFSET, 0.0))
    bearing = np.arctan2(near[:, 1], near[:, 0] - LASER_X_OFFSET)
    # Three copies end to end, so pairs and wall walks can run across +-180 deg (straight behind)
    ext = np.concatenate((near, near, near))
    ext_bearing = np.concatenate((bearing - 2 * math.pi, bearing, bearing + 2 * math.pi))
    step_after = np.hypot(*(ext[1:] - ext[:-1]).T)  # step_after[k] = distance ext[k] -> ext[k+1]
    max_step = _wall_step(ext)                       # Largest step still on the same wall, per point
    gaps = []
    for i in range(n, 2 * n):
        if step_after[i] <= max_step[i]:
            continue  # The wall carries on past this point: not an end
        a = ext[i]
        # Candidate other ends: later in bearing order (within 180 deg), starting a piece, right distance away
        hi = int(np.searchsorted(ext_bearing, ext_bearing[i] + math.pi))
        js = np.arange(i + 1, min(hi, 3 * n))
        if js.size == 0:
            continue
        widths = np.hypot(*(ext[js] - a).T)
        starts = step_after[js - 1] > max_step[js - 1]  # Previous point is not on the same wall
        for j in js[(widths >= GAP_MIN_WIDTH) & (widths <= GAP_MAX_WIDTH) & starts]:
            b = ext[j]
            width = float(np.hypot(*(b - a)))
            ab = (b - a) / width
            if abs(float(ab @ east_west)) < min_cos:
                continue  # The gap doesn't run East-West
            center = (a + b) / 2.0
            normal = np.array((-ab[1], ab[0]))
            if normal @ (center - lidar) < 0.0:
                normal = -normal  # Point through the gap, away from the robot
            # Points between the two ends (in bearing order) must be seen THROUGH the gap, beyond its line
            between = ext[i + 1:j]
            if between.size and np.any((between - a) @ normal <= GAP_OPEN_DEPTH):
                continue
            # Both edges must be wall ends (walls along X or Y), the walls running away from the gap
            piece_a, piece_b = _axis_wall_end(ext, i, -1, axes), _axis_wall_end(ext, j, +1, axes)
            if piece_a is None or piece_b is None:
                continue
            # ...and the gate line closed: the only scans beyond it went through the gap itself
            if not _line_closed(points, a, b):
                continue
            # The space just beyond the gap must be empty
            rel = points - center
            along, depth = rel @ ab, rel @ normal
            if np.any((np.abs(along) < width / 2.0 - 0.03) & (depth > 0.02) & (depth < GAP_OPEN_DEPTH)):
                continue
            gaps.append({"center": center, "normal": normal, "width": width, "a": a.copy(), "b": b.copy()})
            break  # One gap per starting end
    return gaps


def _to_world(q, x, y, c, s):
    '''Convert a robot-frame point to the world frame.'''
    return np.array((x + c * q[0] - s * q[1], y + s * q[0] + c * q[1]))


def _new_board_crossing(here:np.ndarray, direction:np.ndarray, east:np.ndarray=None) -> dict:
    '''Initial state for the board crossing.'''
    return {"phase": "SCAN", "origin": here.copy(), "dir": direction / np.linalg.norm(direction), "east": east,
            "tracks": [], "axis": None, "board": 0, "calls": 0, "t": 0.0, "stamp": None, "latest": {},
            "phase_t": 0.0, "side_done": False, "side_start": None}


def _scan_stamp(scan_msg):
    '''Timestamp of a scan in seconds, or None if it has none.'''
    header = getattr(scan_msg, "header", None)
    if header is None:
        return None
    t = float(header.stamp.sec) + 1e-9 * float(header.stamp.nanosec)
    return t if t > 0.0 else None


def _detect_boards(world_pts, lidar_w, origin, d, t, robot_along) -> list:
    '''Find board-like clusters ahead of the robot.'''
    rel = world_pts - origin
    along, lat = rel @ d, rel @ t
    keep = ((along > robot_along + 0.05) & (along < robot_along + BOARD_SCAN_DEPTH)
            & (np.abs(lat) < BOARD_SCAN_HALF_WIDTH))
    if np.count_nonzero(keep) < 3:
        return []
    pts, along, lat = world_pts[keep], along[keep], lat[keep]
    from_lidar = pts - lidar_w
    order = np.argsort(np.arctan2(from_lidar[:, 1], from_lidar[:, 0]))  # Scan order, as seen by the LiDAR
    pts, along, lat = pts[order], along[order], lat[order]
    breaks = np.concatenate(([0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T) > 0.06)))
    boards = []
    for k in range(int(breaks[-1]) + 1):
        a, l = along[breaks == k], lat[breaks == k]
        if len(a) < 3:
            continue
        length, depth = float(l.max() - l.min()), float(a.max() - a.min())
        if 0.05 <= length <= BOARD_MAX_LENGTH and depth <= BOARD_MAX_DEPTH:
            # Near face = the closest points (a bent board's middle sticks out); median of the nearest third
            near_face = float(np.median(np.sort(a)[:max(1, len(a) // 3)]))
            boards.append((near_face, float((l.max() + l.min()) / 2.0), float(l.min()), float(l.max())))
    return boards


def _travel(tr, since=None) -> float:
    '''Sideways distance a board track has covered.'''
    lats = tr["lats"] if since is None else [l for l, ts in zip(tr["lats"], tr["ts"]) if ts >= since]
    return max(lats) - min(lats) if lats else 0.0


def _board_safe_to_cross(b:dict, tr:dict, det, robot_lat:float, half_path:float) -> tuple:
    '''Decide whether it is safe to cross a board now.'''
    now = b["t"]
    waited = now - b["phase_t"]
    recent = [(ts, l, lo, hi) for ts, l, lo, hi in zip(tr["ts"], tr["lats"], tr["los"], tr["his"])
              if ts >= now - BOARD_RECENT_TIME]

    def in_path(lo, hi):
        '''Does this lateral span overlap the robot's path?'''
        return lo - robot_lat < half_path and hi - robot_lat > -half_path

    if det is None:
        # Not seen right now (out of view, or swung out of the LiDAR plane)
        if not recent:
            return (waited > BOARD_LOST_TIME), "board not seen", False
        last_ts, _, lo, hi = recent[-1]
        clearance = max(lo - robot_lat - half_path, -half_path - (hi - robot_lat))
        if now - last_ts > BOARD_LOST_TIME and clearance >= BOARD_LOST_CLEARANCE:
            return True, "board lost beside the path", False
        return False, "board not seen", False

    if in_path(det[2], det[3]):
        return False, "board in front", True

    # It is beside the path. Which side, and where is it heading?
    centre = det[1] - robot_lat
    side = 1.0 if centre > 0.0 else -1.0

    # 1) Stopped (swing died out) beside the path
    if _travel(tr, since=now - 3.0) < BOARD_STILL_TRAVEL and waited > 3.0:
        return True, "board stopped beside the path", False

    # 2) Swing no longer reaches the path: not in the path at all for a whole recent window, always on this side
    if waited >= BOARD_RECENT_TIME and recent and all(not in_path(lo, hi) for _, _, lo, hi in recent) \
            and all((l - robot_lat) * side > 0.0 for _, l, _, _ in recent):
        return True, "swing no longer reaches the path", False

    # 3) Moving away toward the LONGER end of its swing (the end further from the path): go now.
    #    The longer end is judged from its recent swing only (the swing decays).
    reach_pos = max(hi for _, _, _, hi in recent) - robot_lat if recent else det[3] - robot_lat
    reach_neg = robot_lat - min(lo for _, _, lo, _ in recent) if recent else robot_lat - det[2]
    long_side = 1.0 if reach_pos >= reach_neg else -1.0
    window = [(ts, l) for ts, l, _, _ in recent if ts >= now - 0.5]
    if len(window) >= 2 and window[-1][0] > window[0][0]:
        v = (window[-1][1] - window[0][1]) / (window[-1][0] - window[0][0])
        if side == long_side and v * side > BOARD_MOVING_SPEED:
            return True, "moving away to the longer end", False
        if v * side > BOARD_MOVING_SPEED:
            return False, "moving away, but to the shorter end (comes back soon)", False
    return False, "beside the path, coming back", False


def board_crossing_command(scan_msg, state:dict, pose) -> tuple:
    '''One step of the board-crossing state machine (SCAN, SIDE, FACE, APPROACH, WAIT, CROSS).'''
    b = state["boards"]
    b["calls"] += 1
    x, y, heading_deg = float(pose[0]), float(pose[1]), float(pose[2])
    th = math.radians(heading_deg)
    c, s = math.cos(th), math.sin(th)
    points, ranges = scan_to_robot_points(scan_msg)
    corridor = analyse_front_corridor(points, ranges)
    here = np.array((x, y))
    d = b["dir"]
    t = np.array((-d[1], d[0]))
    robot_along = float((here - b["origin"]) @ d)
    robot_lat = float((here - b["origin"]) @ t)
    world_pts = np.column_stack((x + c * points[:, 0] - s * points[:, 1], y + s * points[:, 0] + c * points[:, 1]))
    lidar_w = _to_world((LASER_X_OFFSET, 0.0), x, y, c, s)

    # Clock from the scan stamps. The node's timer runs faster than the LiDAR, so the same scan arrives
    # several times: only a NEW scan updates the board tracks (else speeds and counts are off).
    stamp = _scan_stamp(scan_msg)
    if stamp is None:
        new_scan = True
        b["t"] += 0.1  # No stamps (pocket_test): one scan per call, 10 Hz
    else:
        new_scan = b["stamp"] is None or stamp != b["stamp"]
        if new_scan and b["stamp"] is not None:
            b["t"] += max(0.0, stamp - b["stamp"])
        b["stamp"] = stamp
    now = b["t"]

    # Keep facing the travel direction (straight-line driving only)
    error = math.radians(wrap_deg(math.degrees(math.atan2(d[1], d[0])) - heading_deg))
    turn = float(np.clip(GAP_ALIGN_GAIN * error, -MAX_TURN_SPEED, MAX_TURN_SPEED))
    forward = 0.0
    reason = ""

    # Update the board tracks (grouped by depth along the travel direction)
    if new_scan:
        latest = {}  # track index -> detection seen on this scan
        for det in _detect_boards(world_pts, lidar_w, b["origin"], d, t, robot_along):
            tolerance = max(BOARD_DEPTH_TOLERANCE, BOARD_DEPTH_ERROR * (det[0] - robot_along))
            best, best_err = None, tolerance
            for i, tr in enumerate(b["tracks"]):
                err = abs(float(np.median(tr["depths"])) - det[0])
                if err <= best_err and i not in latest:
                    best, best_err = i, err
            if best is None:
                b["tracks"].append({"depths": [], "lats": [], "los": [], "his": [], "ts": []})
                best = len(b["tracks"]) - 1
            tr = b["tracks"][best]
            for key, value in (("depths", det[0]), ("lats", det[1]), ("los", det[2]), ("his", det[3]), ("ts", now)):
                tr[key].append(value)
                del tr[key][:-300]
            latest[best] = det
        b["latest"] = latest

    def enter(phase):
        '''Switch to a new phase and note when it started.'''
        b["phase"], b["phase_t"] = phase, now

    front_along = robot_along + BODY_FRONT
    rear_along = robot_along - ROBOT_REAR
    half_path = BODY_HALF_WIDTH + BOARD_PATH_MARGIN

    if b["phase"] == "SCAN":
        def depth(i):
            '''Depth of a board track along the travel direction.'''
            return float(np.median(b["tracks"][i]["depths"]))
        ahead = [i for i in range(len(b["tracks"])) if depth(i) > front_along]  # Boards still to cross
        confident = [i for i in ahead
                     if len(b["tracks"][i]["depths"]) >= BOARD_MIN_DETECTIONS and _travel(b["tracks"][i]) >= BOARD_MIN_TRAVEL]
        moving = [i for i in ahead if _travel(b["tracks"][i]) >= BOARD_MOVING_TRAVEL]
        scanning = now - b["phase_t"]
        chosen = None
        if confident:
            chosen = min(confident, key=depth)  # The nearest board only; the next one gets its own scan
        elif scanning > BOARD_SCAN_FALLBACK_TIME and moving:
            chosen = min(moving, key=depth)     # Swing dying out: take the nearest that moved at all
        elif scanning > BOARD_NONE_TIME and not moving:
            state["boards"] = None  # Nothing moving: no (more) boards, back to normal driving
            if b["board"] == 0:
                state["gate_passed"] = False  # No moving board at all: a false gate - keep looking for the real one
                corridor["board_phase"] = "none found (false gate), driving on"
            else:
                corridor["board_phase"] = "done (no more boards)"
            return 0.0, 0.0, state, corridor
        if chosen is not None:
            b["axis"] = chosen
            if not b["side_done"] and BOARD_SIDE_OFFSET > 0.0 and b.get("east") is not None:
                b["side_done"], b["side_start"] = True, here.copy()
                enter("SIDE")
            else:
                enter("APPROACH")

    if b["axis"] is not None:
        tr = b["tracks"][b["axis"]]
        axis = float(np.median(tr["depths"]))  # Depth of the board's near face along the travel direction
        det = b["latest"].get(b["axis"])

        def board_in_path():
            '''Is the board currently in the robot's path?'''
            return det is not None and det[2] - robot_lat < half_path and det[3] - robot_lat > -half_path

        if b["phase"] == "SIDE":
            # Face east, then drive until the east wall is just ahead of the body
            east = b["east"]
            e = math.radians(wrap_deg(math.degrees(math.atan2(east[1], east[0])) - heading_deg))
            ahead_pts = points[(points[:, 0] > 0.0) & (np.abs(points[:, 1]) < BODY_HALF_WIDTH + 0.02)]
            front_gap = float(np.min(ahead_pts[:, 0])) - BODY_FRONT if ahead_pts.size else math.inf
            driven = float(np.hypot(*(here - b["side_start"])))
            if abs(math.degrees(e)) > GAP_FACE_TOLERANCE_DEG:
                turn = math.copysign(float(np.clip(GAP_ALIGN_GAIN * abs(e), MIN_TURN_SPEED, MAX_TURN_SPEED)), e)
            elif (front_gap <= BOARD_SIDE_WALL_CLEARANCE or driven >= BOARD_SIDE_OFFSET
                    or now - b["phase_t"] > BOARD_SIDE_MAX_TIME):
                enter("FACE")
                reason = "at the east wall" if front_gap <= BOARD_SIDE_WALL_CLEARANCE else "went as far east as allowed"
            else:
                turn = float(np.clip(GAP_ALIGN_GAIN * e, -MAX_TURN_SPEED, MAX_TURN_SPEED))
                forward = min(BOARD_SIDE_SPEED, front_gap - BOARD_SIDE_WALL_CLEARANCE)
                reason = f"wall {front_gap:.2f} m ahead" if math.isfinite(front_gap) else "no wall seen yet"

        if b["phase"] == "FACE":
            if abs(math.degrees(error)) < GAP_FACE_TOLERANCE_DEG:
                enter("APPROACH")
            else:
                turn = math.copysign(max(abs(turn), MIN_TURN_SPEED), error)

        if b["phase"] == "APPROACH":
            stop_at = axis - BOARD_WAIT_GAP
            if front_along >= stop_at - 0.01:
                enter("WAIT")
            elif abs(math.degrees(error)) < 10.0:
                # No hold for the board here: it moves along its own axis and the stop point is before that,
                # so it can't be hit while approaching. WAIT decides when to cross (and has the timeouts).
                forward = min(BOARD_APPROACH_SPEED, 2.0 * (stop_at - front_along))

        if b["phase"] == "WAIT":
            cross, reason, blocking = _board_safe_to_cross(b, tr, det, robot_lat, half_path)
            if not cross and now - b["phase_t"] > BOARD_WAIT_MAX_TIME:
                if blocking:
                    state["boards"] = None  # Board stuck in front: give up, normal avoidance drives round it
                    corridor["board_phase"] = "gave up (board stuck in front)"
                    return 0.0, turn, state, corridor
                cross, reason = True, "waited too long, path clear now"
            if cross:
                enter("CROSS")

        if b["phase"] == "CROSS":
            if rear_along >= axis + 2 * BOARD_HALF_THICKNESS + BOARD_CLEAR_MARGIN:
                b["board"] += 1
                if b["board"] >= BOARD_COUNT:
                    state["boards"] = None  # All boards crossed: back to normal driving
                    corridor["board_phase"] = "done"
                    return 0.0, turn, state, corridor
                b["axis"], b["tracks"], b["latest"] = None, [], {}  # Fresh scan for the next board
                enter("SCAN")
            elif front_along < axis - 0.01 and board_in_path():
                enter("WAIT")  # Swung back in before the body got there: stop and wait again
                reason = "board came back, waiting again"
            else:
                forward = BOARD_CROSS_SPEED

    # For logging / drawing: confident axes as world line segments
    axes_world = []
    for i, tr in enumerate(b["tracks"]):
        if len(tr["depths"]) >= BOARD_MIN_DETECTIONS and _travel(tr) >= BOARD_MIN_TRAVEL:
            depth = float(np.median(tr["depths"]))
            p0 = b["origin"] + d * depth + t * min(tr["lats"])
            p1 = b["origin"] + d * depth + t * max(tr["lats"])
            axes_world.append((p0, p1))
    phase_text = b["phase"] + (f" ({reason})" if reason else "")
    corridor.update({"board_phase": phase_text, "board_index": b["board"], "board_axes": axes_world,
                     "board_tracks": sum(1 for tr in b["tracks"] if _travel(tr) >= 0.15)})
    return forward, turn, state, corridor


def _start_boards(state:dict, scan_msg, pose, gate_center, north, east) -> tuple:
    '''Mark the gate as passed and start the board crossing.'''
    here = np.array((float(pose[0]), float(pose[1])))
    state["gate_passed"] = True
    state["gap"] = None
    state["lock"] = None
    state.setdefault("passed_gaps", []).append(gate_center.copy())
    state["boards"] = _new_board_crossing(here, north, east)
    forward, turn, state, corridor = board_crossing_command(scan_msg, state, pose)
    corridor["gate_passed"] = True
    return forward, turn, state, corridor


def _avoidance_core(scan_msg, state:dict, pose=None) -> tuple:
    '''One control step: board crossing, gate manoeuvre, or normal obstacle avoidance.'''
    heading_deg = None if pose is None else float(pose[2])

    # After passing the gate: the moving-board crossing takes over until both boards are crossed
    if pose is not None and state.get("boards"):
        return board_crossing_command(scan_msg, state, pose)

    gap = state.get("gap")

    if pose is not None and not state.get("gate_passed"):
        x, y = float(pose[0]), float(pose[1])
        th = math.radians(heading_deg)
        c, s = math.cos(th), math.sin(th)
        points, ranges = scan_to_robot_points(scan_msg)
        here = np.array((x, y))
        passed = state.setdefault("passed_gaps", [])
        north_deg = state.get("north_deg", NORTH_DEG)
        north = np.array((math.cos(math.radians(north_deg)), math.sin(math.radians(north_deg))))
        east = np.array((north[1], -north[0]))  # North turned 90 deg clockwise

        step = state["step"] = state.get("step", 0) + 1
        skipped = state.setdefault("skipped_gaps", [])  # (centre, step given up) - retried later
        skipped[:] = [(p, k) for p, k in skipped if step - k < GAP_RETRY_CALLS]
        ignore = passed + [p for p, _ in skipped]

        # Front or back is decided by WHERE THE GAP IS relative to the robot (not by the robot's heading):
        #   gap centre in the SOUTH half (south of the robot) -> BACK: start the board crossing right away
        #   gap centre in the NORTH half                      -> FRONT: line up and drive through
        found = []
        # While turning to face the gate and driving through it, the gap is NOT re-detected: the line
        # detected while approaching / aligning is used as it is
        driving_through = gap is not None and gap["phase"] in ("TURN", "GO")
        for g in ([] if driving_through else detect_gaps(points, heading_deg, north_deg)):
            center = _to_world(g["center"], x, y, c, s)
            past = float((here - center) @ north)  # > 0: the gap is south of the robot
            if past > 0.0:
                if gap is not None and gap["phase"] == "GO":
                    continue  # Driving through the front gate: GO decides when it is far enough past
                if any(np.hypot(*(center - p)) <= GAP_SAME_DISTANCE for p in passed):
                    continue  # Already passed (or a false gate with no boards behind it)
                return _start_boards(state, scan_msg, pose, center, north, east)
            if past < 0.0 and (gap is None or gap["phase"] in ("APPROACH", "ALIGN")):
                # The detected gap line: normal perpendicular to it (edge a -> edge b), pointing north
                normal = np.array((c * g["normal"][0] - s * g["normal"][1], s * g["normal"][0] + c * g["normal"][1]))
                if normal @ north < 0.0:
                    normal = -normal
                if all(np.hypot(*(center - p)) > GAP_SAME_DISTANCE for p in ignore):
                    found.append({"center": center, "normal": normal, "width": g["width"]})
        if gap is None and found:
            nearest_gap = min(found, key=lambda g: np.hypot(*(g["center"] - here)))
            gap = {**nearest_gap, "phase": "APPROACH", "calls": 0, "best": math.inf, "stall": 0}
        elif gap is not None and gap["phase"] in ("APPROACH", "ALIGN"):
            for g in found:
                if np.hypot(*(g["center"] - gap["center"])) < 0.2:
                    n_new = g["normal"] if g["normal"] @ gap["normal"] > 0 else -g["normal"]
                    gap["center"] = 0.7 * gap["center"] + 0.3 * g["center"]
                    n = 0.7 * gap["normal"] + 0.3 * n_new
                    gap["normal"] = n / np.linalg.norm(n)

        if gap is not None:
            gap["calls"] += 1
            center, normal = gap["center"], gap["normal"]
            tangent = np.array((-normal[1], normal[0]))
            depth_before = float((center - here) @ normal)  # Distance still to go to the gap line
            lateral = float((here - center) @ tangent)      # Offset from the gap's centre line
            corridor = analyse_front_corridor(points, ranges)
            forward = turn = 0.0
            # Alignment point: on the centre line at the current depth (within the standoff limits)
            target = center - normal * float(np.clip(depth_before, GAP_MIN_STANDOFF, GAP_MAX_STANDOFF))
            to_target = target - here
            target_distance = float(np.hypot(*to_target))

            if gap["phase"] == "APPROACH":
                if depth_before < 0.0:
                    skipped.append((center.copy(), step))  # Past the gap line without GO (behind check takes it)
                    gap = None
                elif target_distance < GAP_ALIGN_START_DISTANCE and corridor["nearest"] > STOP_DISTANCE + 0.10:
                    gap["phase"] = "ALIGN"  # Close and nothing right ahead: precise alignment from here
                    state["lock"] = None
                else:
                    # Give up if no progress toward the alignment point for a while (stuck behind something)
                    if target_distance < gap["best"] - GAP_APPROACH_PROGRESS:
                        gap["best"], gap["stall"] = target_distance, 0
                    else:
                        gap["stall"] += 1
                    if gap["stall"] > GAP_APPROACH_STALL_CALLS:
                        skipped.append((center.copy(), step))  # Stuck: try again later
                        gap = None
                    else:
                        # Normal obstacle avoidance, steering toward the alignment point
                        aim = math.degrees(math.atan2(to_target[1], to_target[0]))
                        forward, turn, state["lock"], corridor = _obstacle_avoidance(
                            scan_msg, state.get("lock"), heading_deg, target_heading_deg=aim)
                        state["gap"] = gap
                        corridor.update({"gap_phase": "APPROACH", "gap_center": center.copy(),
                                         "gap_normal": normal.copy(), "gap_width": gap["width"],
                                         "gap_lateral": lateral})
                        return forward, turn, state, corridor

            if gap is not None and gap["phase"] == "ALIGN":
                if abs(lateral) < GAP_ALIGN_TOLERANCE or target_distance < GAP_ALIGN_TOLERANCE:
                    gap["phase"] = "TURN"
                else:
                    error = math.radians(wrap_deg(math.degrees(math.atan2(to_target[1], to_target[0])) - heading_deg))
                    turn = float(np.clip(GAP_ALIGN_GAIN * error, -MAX_TURN_SPEED, MAX_TURN_SPEED))
                    # Drive only roughly toward the target; slow down for anything in the corridor
                    nearest = corridor["nearest"]
                    safe = float(np.clip((nearest - STOP_DISTANCE) / (SLOWDOWN_DISTANCE - STOP_DISTANCE), 0.0, 1.0))
                    forward = GAP_ALIGN_SPEED * max(0.0, math.cos(error)) * safe
                    forward = min(forward, target_distance)  # Don't overshoot the centre line

            if gap is not None and gap["phase"] == "TURN":
                error = math.radians(wrap_deg(math.degrees(math.atan2(normal[1], normal[0])) - heading_deg))
                if abs(math.degrees(error)) < GAP_FACE_TOLERANCE_DEG:
                    gap["phase"] = "GO"
                else:
                    rate = float(np.clip(GAP_ALIGN_GAIN * abs(error), MIN_TURN_SPEED, MAX_TURN_SPEED))
                    turn = math.copysign(rate, error)

            if gap is not None and gap["phase"] == "GO":
                if -depth_before - ROBOT_REAR >= GATE_GO_PAST:  # Body's rear GATE_GO_PAST beyond the gate line
                    return _start_boards(state, scan_msg, pose, center, north, east)  # Through the gate
                error = math.radians(wrap_deg(math.degrees(math.atan2(normal[1], normal[0])) - heading_deg))
                turn = float(np.clip(GAP_ALIGN_GAIN * error, -MAX_TURN_SPEED, MAX_TURN_SPEED))
                # Only something straight ahead of the body stops it (the gap edges are beside it)
                ahead = ((points[:, 0] > 0.0) & (points[:, 0] < BODY_FRONT + GAP_STOP_DISTANCE)
                         & (np.abs(points[:, 1]) < BODY_HALF_WIDTH + 0.02))
                forward = 0.0 if np.any(ahead) else GAP_SPEED

            if gap is not None and gap["calls"] > GAP_MAX_CALLS:
                skipped.append((gap["center"].copy(), step))  # Taking too long: try again later
                gap = None

            if gap is not None:
                state["gap"] = gap
                corridor.update({"gap_phase": gap["phase"], "gap_center": gap["center"].copy(),
                                 "gap_normal": gap["normal"].copy(), "gap_width": gap["width"],
                                 "gap_lateral": lateral, "locked": False})
                return forward, turn, state, corridor
    state["gap"] = None

    # Normal obstacle avoidance, holding north-east - or a detour toward north / east if north-east is blocked
    target, goal_label = state.get("target_heading_deg", TARGET_HEADING_DEG), None
    if heading_deg is not None:
        target, goal_label = choose_goal_heading(scan_msg, heading_deg, target,
                                                 state.get("north_deg", NORTH_DEG), state)
    forward, turn, state["lock"], corridor = _obstacle_avoidance(
        scan_msg, state.get("lock"), heading_deg, target_heading_deg=target,
        north_deg=state.get("north_deg", NORTH_DEG))
    corridor["goal"] = goal_label
    corridor["gap_phase"] = None
    return forward, turn, state, corridor


def _with_virtual_walls(scan_msg, pose, walls):
    '''Copy of the scan with the virtual dead-end walls added.'''
    ranges = np.asarray(scan_msg.ranges, dtype=float).copy()
    x, y, th = float(pose[0]), float(pose[1]), math.radians(float(pose[2]))
    sx, sy = x + LASER_X_OFFSET * math.cos(th), y + LASER_X_OFFSET * math.sin(th)
    angles = th + LASER_YAW_OFFSET + float(scan_msg.angle_min) + np.arange(ranges.size) * float(scan_msg.angle_increment)
    dx, dy = np.cos(angles), np.sin(angles)
    valid = np.isfinite(ranges) & (ranges > max(float(scan_msg.range_min), 0.01))
    current = np.where(valid, ranges, np.inf)
    for (px, py), (qx, qy) in walls:
        ex, ey = qx - px, qy - py
        den = dx * (-ey) - dy * (-ex)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((px - sx) * (-ey) - (py - sy) * (-ex)) / den
            u = (dx * (py - sy) - dy * (px - sx)) / den
        hit = (np.abs(den) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0) & (t < current)
        current = np.where(hit, t, current)
    out = np.where(np.isfinite(current) & (current < np.inf), current, ranges)  # Keep the driver's no-return values
    import types
    return types.SimpleNamespace(angle_min=scan_msg.angle_min, angle_max=getattr(scan_msg, "angle_max", None),
                                 angle_increment=scan_msg.angle_increment, range_min=scan_msg.range_min,
                                 range_max=scan_msg.range_max, ranges=list(out),
                                 header=getattr(scan_msg, "header", None))


def _update_deadends(state:dict, corridor:dict, pose) -> None:
    '''Add a virtual wall where the robot keeps backing out of the same spot.'''
    backing = bool(corridor.get("backing"))
    started = backing and not state.get("was_backing", False)
    state["was_backing"] = backing
    corridor["deadend_added"] = False
    if not started:
        return
    x, y, heading_deg = float(pose[0]), float(pose[1]), float(pose[2])
    th = math.radians(heading_deg)
    front = np.array((x + BODY_FRONT * math.cos(th), y + BODY_FRONT * math.sin(th)))
    events = state.setdefault("backup_events", [])
    events.append((front, heading_deg))
    del events[:-20]
    same = [(p, h) for p, h in events
            if np.hypot(*(p - front)) < DEADEND_RADIUS and abs(wrap_deg(h - heading_deg)) < DEADEND_HEADING_DEG]
    if len(same) >= DEADEND_BACKUPS:
        centre = np.mean([p for p, _ in same], axis=0)
        mean_heading = math.atan2(np.mean([math.sin(math.radians(h)) for _, h in same]),
                                  np.mean([math.cos(math.radians(h)) for _, h in same]))
        d = np.array((math.cos(mean_heading), math.sin(mean_heading)))
        t = np.array((-d[1], d[0]))
        mid = centre + d * DEADEND_WALL_OFFSET
        walls = state.setdefault("virtual_walls", [])
        walls.append((mid - t * DEADEND_WALL_WIDTH / 2.0, mid + t * DEADEND_WALL_WIDTH / 2.0))
        del walls[:-DEADEND_MAX_WALLS]
        state["backup_events"] = [e for e in events if not any(e is s_ for s_ in same)]
        state["lock"] = None  # Re-decide the turn with the new wall in place
        corridor["deadend_added"] = True


def avoidance_command(scan_msg, state:dict, pose=None) -> tuple:
    '''Main entry point: one control step from a scan and a pose.'''
    walls = state.get("virtual_walls", [])
    if pose is not None and walls:
        scan_msg = _with_virtual_walls(scan_msg, pose, walls)
    forward, turn, state, corridor = _avoidance_core(scan_msg, state, pose)
    if pose is not None:
        _update_deadends(state, corridor, pose)
        corridor["virtual_walls"] = [(a.copy(), b.copy()) for a, b in state.get("virtual_walls", [])]
    return forward, turn, state, corridor


def wrap_deg(angle:float) -> float:
    '''Wrap an angle into [-180, 180) degrees.'''
    return (angle + 180.0) % 360.0 - 180.0

def optitrack_to_maze(x:float, y:float, heading_deg:float, frame:dict) -> tuple:
    '''Convert an OptiTrack pose to the maze frame.'''
    dx, dy = x - frame["origin_x"], y - frame["origin_y"]
    r = math.radians(frame["rotation_deg"])
    c, s = math.cos(r), math.sin(r)
    return (c * dx + s * dy, -s * dx + c * dy,
            wrap_deg(heading_deg - frame["rotation_deg"] - frame["heading_offset_deg"]))

def load_frame(robot_number:int) -> tuple:
    '''Read the OptiTrack frame calibration for this robot.'''
    parser = configparser.ConfigParser()
    path = os.path.join(_PACKAGE_DIR, "optitrack_variables.config")
    if not parser.read(path):
        raise FileNotFoundError(f"Could not read {path}")
    frame = {k: parser.getfloat("frame", k) for k in ("origin_x", "origin_y", "rotation_deg")}
    raw = parser.get("frame", "heading_offset_deg", fallback="auto").strip().lower()
    # Rigid bodies are created facing +y of the maze frame, so Motive's zero heading is maze +y (90 deg)
    frame["heading_offset_deg"] = wrap_deg(-(frame["rotation_deg"] + 90.0)) if raw == "auto" else float(raw)
    if parser.has_section("heading_offsets") and parser.has_option("heading_offsets", str(robot_number)):
        frame["heading_offset_deg"] = parser.getfloat("heading_offsets", str(robot_number))
    return frame

class LidarNode(Node):
    '''Node that only checks whether the scan comes from simulation or the real LiDAR.'''
    def __init__(self):
        '''Subscribe to the scan.'''
        super().__init__("LidarNode")
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # Subscribe to lidar
        self.is_simulation = None

    def sub_scan_callback(self, msg):
        '''Detect simulation from the scan size.'''
        if len(msg.ranges) <= 360: 
            self.is_simulation = True
        else:
            self.is_simulation = False

class ObstacleCourseNode(Node):
    '''Node that drives the obstacle course from the LiDAR scan and the pose.'''
    def __init__(self, is_simulation:bool=True, calibrate:bool=False):
        '''Set up subscriptions, the publisher and the control timer.'''
        super().__init__('obstaclecourse')
        self.get_logger().info("Starting ObstacleCourseNode")

        self.is_simulation = is_simulation
        self.calibrate = calibrate # Real robot: only print poses, never drive
        self.raw_pose = None # Pose exactly as Optitrack reported it (real robot only)
        self.frame = None
        self._ticks = 0

        if is_simulation:
            self.max_translate_velocity = 1.4
        else:
            self.max_translate_velocity = 0.5 # Please keep this in place; 0.3m/s is more than fast enough 

        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # Subscribe to LiDAR scan data

        # Subscribe to the dynamic_pose topic from Gazebo that publishes ground-truth pose data
        if self.is_simulation:
            self.subscription = self.create_subscription(Odometry, 'odom', self.odometer_callback, 2)
        else:
            qos_profile = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
            # On the Bingda robots ROS_DOMAIN_ID is the robot's number, so the Optitrack topic is picked automatically
            domain = os.environ.get("ROS_DOMAIN_ID", "")
            self.robot_number = int(domain) if domain.isdigit() and 1 <= int(domain) <= 99 else 3
            self.frame = load_frame(self.robot_number)
            self.get_logger().info(f"Reading pose from /vrpn_mocap/bingda_{self.robot_number:03d}/pose")

            self.map_sub = self.create_subscription( 
                PoseStamped,
                f'/vrpn_mocap/bingda_{self.robot_number:03d}/pose',
                self.optitrack_callback, 
                qos_profile
                )
            
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # Publish to cmd_vel node       
        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

        self.pose = None
        self.last_scan_msg = None # Full message, needed for the scan angles
        self.avoid_state = {}     # avoidance_command() memory: turn lock and gap manoeuvre
        if not is_simulation:
            self.avoid_state["target_heading_deg"] = REAL_TARGET_HEADING_DEG # Real robot: north-east in Optitrack axes
            self.avoid_state["north_deg"] = REAL_NORTH_DEG # Real robot: north = Optitrack -y
        self.corridor = None      # Latest analyse_front_corridor() result

    def sub_scan_callback(self, msg):
        '''Store the latest scan.'''
        self.last_scan_msg = msg

    def yaw_from_quaternion(self, q):
        '''Yaw angle (rad) from a quaternion.'''
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def optitrack_callback(self, msg:PoseStamped):
        '''Store the OptiTrack pose, converted to the maze frame.'''
        x, y = msg.pose.position.x, msg.pose.position.y
        heading = np.rad2deg(self.yaw_from_quaternion(msg.pose.orientation))
        self.raw_pose = np.array((x,y,heading))
        x, y, heading = optitrack_to_maze(x, y, heading, self.frame) # CA2 maze frame, see optitrack_variables.config
        self.pose = np.array((x,y,heading))
        return self.pose

    def odometer_callback(self, msg):
        '''Store the Gazebo odometry pose.'''
        latest_pose_msg = msg.pose.pose 
        heading = np.rad2deg(self.yaw_from_quaternion(latest_pose_msg.orientation))
        self.pose = np.array((latest_pose_msg.position.x, latest_pose_msg.position.y, heading))
        return self.pose

    def move_2D(self, x:float=0.0, y:float=0.0, turn:float=0.0):
        '''Publish a velocity command, capped to the robot's limits.'''
        twist_msg = Twist()
        x = np.clip(x, -self.max_translate_velocity, self.max_translate_velocity)
        y = np.clip(y, -self.max_translate_velocity, self.max_translate_velocity)
        turn = np.clip(turn, -self.max_translate_velocity*2, self.max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.publisher_.publish(twist_msg)

    def timer_callback(self):
        '''Control loop: run one controller step, publish it and log.'''
        if self.pose is None:
            print("No pose detected")
            return # Does not run if no pose received from Odom or Optitrack

        if self.calibrate and self.raw_pose is not None:
            self._ticks += 1
            if self._ticks % 10 == 0: # twice a second
                ox, oy, oh = self.raw_pose
                self.get_logger().info(f"optitrack ({ox:+.3f}, {oy:+.3f}, {oh:+7.1f} deg)  ->  maze ({self.pose[0]:+.3f}, {self.pose[1]:+.3f}, {self.pose[2]:+7.1f} deg)")
            return # Calibration mode: look, don't drive

        ###### INSERT CODE HERE ######
        if self.last_scan_msg is None:
            self.move_2D(0.0, 0.0, 0.0)
            return # Wait for the first LiDAR scan

        forward, turn, self.avoid_state, corridor = avoidance_command(
            self.last_scan_msg, self.avoid_state, pose=tuple(float(v) for v in self.pose))
        self.corridor = corridor
        nearest = corridor["nearest"]
        self.move_2D(forward, 0.0, turn)

        self._ticks += 1
        if self._ticks % 10 == 0: # twice a second
            nearest_text = "clear" if math.isinf(nearest) else f"{nearest:.2f} m"
            self.get_logger().info(
                f"nearest={nearest_text}, blocking={corridor['blocking']}, "
                f"v={forward:.2f}, turn={'left' if turn > 0 else 'right' if turn < 0 else 'none'}"
                f"{' (side blocked: counter-turning)' if corridor.get('side_blocked') else ''}"
                f"{' (intended turn kept being blocked: switched side)' if corridor.get('side_turn_given_up') else ''}"
                f"{' [locked]' if corridor.get('locked') else ''}"
                f"{' (lock handed to next obstacle, same turn)' if corridor.get('lock_handover') else ' (lock released)' if corridor.get('lock_released') else ''}"
                f"{' BACKING UP' if corridor.get('backing') else ''}"
                f"{' (cones switched turn side)' if corridor.get('cone_switch') else ''}"
                f"{' (slowed after big cone switch)' if corridor.get('switch_slow') else ''}"
                f"{' DEAD END blocked' if corridor.get('deadend_added') else ''}"
                f"{' (rear blocked)' if corridor.get('rear_blocked') else ''}, "
                f"obstacles={corridor['obstacles']}"
                + (f", GATE {corridor['gap_phase']} (width {corridor['gap_width']:.2f} m, "
                   f"off centre {corridor['gap_lateral']:+.2f} m)" if corridor.get('gap_phase') else "")
                + (", GATE PASSED: starting board crossing" if corridor.get('gate_passed') else "")
                + (f", goal: {corridor['goal']}" if corridor.get('goal') else "")
                + (f", BOARDS {corridor['board_phase']}"
                   + (f" (board {corridor['board_index'] + 1}/2, axes found {len(corridor['board_axes'])}, "
                      f"moving tracks {corridor['board_tracks']})" if 'board_axes' in corridor else "")
                   if corridor.get('board_phase') else "")
                + (", (gate passed)" if self.avoid_state.get("gate_passed") else "")
                + f", pose={self.pose}"
            )
        ###### INSERT CODE HERE ######
                

def main(args=None):
    '''Start the node (simulation or real robot is detected from the scan).'''
    arg_parser = argparse.ArgumentParser(description="RB2301 final project")
    arg_parser.add_argument('--calibrate', action='store_true', help="Real robot: only print the Optitrack pose and the maze-frame pose twice a second, never drive")
    cli_args, ros_args = arg_parser.parse_known_args(args=args)
    print("Starting obstacle course")
    rclpy.init(args=ros_args)

    # Use lidar scan length to determine if in simulation or not
    lidar = LidarNode()
    while lidar.is_simulation is None:
        rclpy.spin_once(lidar)
    is_simulation = lidar.is_simulation
    lidar.destroy_node()

    # Start spinning the obstacle node and only stop once SystemExit error is raised within the node callback
    obstacle = ObstacleCourseNode(is_simulation, cli_args.calibrate)
    try:
        rclpy.spin(obstacle)
    except SystemExit:
        print("Shutting down")
    obstacle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()