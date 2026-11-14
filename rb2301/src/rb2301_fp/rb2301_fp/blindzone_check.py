#!/usr/bin/env python3
"""Measure the LiDAR's blind zone (the minimum distance it can still see).

Interactive: it asks you to place an object in front of the robot, then to slide it slowly
toward the robot while it tracks the object's distance. The moment the object disappears from
the scan, the last distance it was seen at is the edge of the blind zone. Repeat a few times
for a reliable value.

Run (robot or Gazebo, with the /scan topic available):
    ros2 run rb2301_fp blindzone_check            # after colcon build
    python3 -m rb2301_fp.blindzone_check          # or directly, from src/rb2301_fp

Tips: use something narrow and solid (a box, a book standing up, a can), keep it straight in
front of the robot, and move it slowly (a few cm per second).
"""
import math
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rb2301_fp.obstacle_course import BODY_FRONT, LASER_X_OFFSET, LASER_YAW_OFFSET

FRONT_CONE_DEG = 15.0      # Only look at beams within +-this of straight ahead (deg)
CLEAR_DISTANCE = 0.60      # Before starting, nothing should be closer than this in the cone (m)
START_MIN, START_MAX = 0.30, 0.80  # The object must first be seen within this distance band (m)
TRACK_JUMP = 0.10          # The object is "lost" when nothing is within last distance + this... (m)
LOST_SCANS = 5             # ...for this many scans in a row
TRIALS = 3                 # Measurements to take


class ScanWatcher(Node):
    def __init__(self):
        super().__init__("blindzone_check")
        self.create_subscription(LaserScan, "scan", self.callback, 5)
        self.lock = threading.Lock()
        self.msg = None
        self.count = 0
        self.first_time = None

    def callback(self, msg):
        with self.lock:
            self.msg = msg
            self.count += 1
            if self.first_time is None:
                self.first_time = time.monotonic()

    def latest(self):
        with self.lock:
            return self.msg, self.count


def front_cone(msg):
    """Raw ranges of the beams within +-FRONT_CONE_DEG of the robot's straight ahead.
    The LiDAR is mounted facing backwards (LASER_YAW_OFFSET), so 'ahead' is computed per beam."""
    ranges = np.asarray(msg.ranges, dtype=float)
    angles = float(msg.angle_min) + np.arange(ranges.size) * float(msg.angle_increment) + LASER_YAW_OFFSET
    angles = (angles + math.pi) % (2.0 * math.pi) - math.pi
    return ranges[np.abs(angles) <= math.radians(FRONT_CONE_DEG)]


def nearest_in_cone(msg):
    """Nearest real return in the front cone (m), or None. Ignores range_min on purpose, so
    readings the driver still reports below its stated minimum are counted too."""
    cone = front_cone(msg)
    valid = cone[np.isfinite(cone) & (cone > 0.01)]
    return float(valid.min()) if valid.size else None


def describe_lost_beams(msg):
    """What the driver reports for the front cone once the object is lost (0, inf, nan...)."""
    cone = front_cone(msg)
    zeros = int(np.count_nonzero(cone == 0.0))
    infs = int(np.count_nonzero(np.isinf(cone)))
    nans = int(np.count_nonzero(np.isnan(cone)))
    below_min = int(np.count_nonzero(np.isfinite(cone) & (cone > 0.0) & (cone < float(msg.range_min))))
    return f"{cone.size} beams in cone: {zeros} zero, {infs} inf, {nans} nan, {below_min} below range_min"


def wait_for_new_scan(node, last_count, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        msg, count = node.latest()
        if count != last_count and msg is not None:
            return msg, count
        time.sleep(0.01)
    return None, last_count


def ask(prompt):
    try:
        return input(prompt)
    except EOFError:
        return ""


def one_trial(node, trial):
    print(f"\n=== Measurement {trial} of {TRIALS} ===")
    # 1. Front must be clear
    while True:
        ask(f"Clear the area in front of the robot (nothing within {CLEAR_DISTANCE:.1f} m straight ahead), then press Enter... ")
        msg, count = node.latest()
        nearest = nearest_in_cone(msg)
        if nearest is None or nearest > CLEAR_DISTANCE:
            break
        print(f"  Something is still {nearest:.2f} m in front. Move it away and try again.")

    # 2. Object in the start band
    while True:
        ask(f"Place the object straight in front, about {START_MIN + 0.1:.1f}-{START_MAX - 0.3:.1f} m away, then press Enter... ")
        msg, count = node.latest()
        nearest = nearest_in_cone(msg)
        if nearest is not None and START_MIN <= nearest <= START_MAX:
            print(f"  Object seen at {nearest:.3f} m from the LiDAR.")
            break
        print("  Can't see an object in that range in front. "
              + ("" if nearest is None else f"(nearest thing: {nearest:.2f} m) ") + "Adjust and try again.")

    # 3. Track while the object is moved closer
    print("Now slide the object SLOWLY straight toward the robot until it touches (or nearly).")
    print("Tracking... (Ctrl+C to abort)")
    last_seen, closest, lost_scans = nearest, nearest, 0
    while True:
        msg, count = wait_for_new_scan(node, count)
        if msg is None:
            print("\n  No scans arriving. Is the LiDAR running?")
            return None
        seen = nearest_in_cone(msg)
        if seen is not None and seen <= last_seen + TRACK_JUMP:
            last_seen, lost_scans = seen, 0
            closest = min(closest, seen)
            sys.stdout.write(f"\r  object at {seen:.3f} m   (closest so far {closest:.3f} m)      ")
            sys.stdout.flush()
        else:
            lost_scans += 1
            sys.stdout.write(f"\r  object NOT seen ({lost_scans}/{LOST_SCANS})   last seen at {last_seen:.3f} m      ")
            sys.stdout.flush()
            if lost_scans >= LOST_SCANS:
                print()
                if last_seen > 0.45:
                    print("  Lost the object while it was still far away (moved sideways out of the cone?). Retrying.")
                    return one_trial(node, trial)
                print(f"  Object disappeared. Last seen at {last_seen:.3f} m from the LiDAR.")
                print(f"  Driver output for the front now: {describe_lost_beams(msg)}")
                return closest


def main(args=None):
    rclpy.init(args=args)
    node = ScanWatcher()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    print("Waiting for /scan ...")
    msg, count = None, 0
    while msg is None:
        msg, count = node.latest()
        time.sleep(0.05)
    time.sleep(1.0)
    msg, count2 = node.latest()
    rate = (count2 - count) / 1.0
    print(f"LiDAR: {len(msg.ranges)} beams, {math.degrees(msg.angle_increment):.2f} deg apart, ~{rate:.0f} scans/s")
    print(f"Driver says: range_min = {msg.range_min:.3f} m, range_max = {msg.range_max:.1f} m")
    print(f"Front cone used: +-{FRONT_CONE_DEG:.0f} deg around straight ahead "
          f"(LiDAR is {LASER_X_OFFSET:.3f} m ahead of the rear axle, the bumper {BODY_FRONT - LASER_X_OFFSET:.3f} m ahead of the LiDAR)")

    results = []
    try:
        for trial in range(1, TRIALS + 1):
            value = one_trial(node, trial)
            if value is not None:
                results.append(value)
    except KeyboardInterrupt:
        print("\nAborted.")

    if results:
        worst = max(results)
        bumper = BODY_FRONT - LASER_X_OFFSET
        print("\n=== Result ===")
        print("Closest distance seen before losing the object, per measurement: "
              + ", ".join(f"{r:.3f} m" for r in results))
        print(f"Blind zone (from the LiDAR): about {worst:.3f} m  (largest of the measurements)")
        print(f"  = about {max(0.0, worst - bumper):.3f} m in front of the bumper")
        print(f"Driver's range_min: {msg.range_min:.3f} m")
        print(f"Suggested: keep STOP_DISTANCE >= {worst + 0.05:.2f} m (blind zone + 5 cm) in obstacle_course.py")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
