import argparse
import configparser
import math
import os
import time
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

# Minimal linear-motion controller and persistent obstacle memory.
LASER_YAW_OFFSET = math.pi
LASER_X_OFFSET = 0.11423
MAP_RESOLUTION = 0.02
MAP_MAXIMUM_SCAN_RANGE = 0.50
LINEAR_SPEED = 0.4
ROBOT_HALF_WIDTH = 0.17
AVOIDANCE_CLEARANCE = 0.2
TRANSITION_COMMIT_DISTANCE = 0.20
# A remembered cell fades out once it has not been seen by the LiDAR for this
# long. Cells still in view are refreshed on every scan.
MAP_MEMORY_SECONDS = 3.0
# Include half a map cell because occupied points are represented by cell
# centres and can shift by up to 0.01 m during 0.02 m quantization.
MAP_CORRIDOR_HALF_WIDTH = ROBOT_HALF_WIDTH + MAP_RESOLUTION / 2.0

def wrap_deg(angle:float) -> float:
    '''Wrap an angle in degrees into [-180, 180).'''
    return (angle + 180.0) % 360.0 - 180.0

def optitrack_to_maze(x:float, y:float, heading_deg:float, frame:dict) -> tuple:
    '''Optitrack-frame pose (x, y, heading in degrees) -> CA2 maze-frame pose (see optitrack_variables.config).'''
    dx, dy = x - frame["origin_x"], y - frame["origin_y"]
    r = math.radians(frame["rotation_deg"])
    c, s = math.cos(r), math.sin(r)
    return (c * dx + s * dy, -s * dx + c * dy,
            wrap_deg(heading_deg - frame["rotation_deg"] - frame["heading_offset_deg"]))

def maze_to_course(x:float, y:float, heading_deg:float, course:dict) -> tuple:
    '''CA2 maze-frame pose -> final-project course frame (origin/rotation of the course in the maze frame, [course] in the config).'''
    dx, dy = x - course["origin_x"], y - course["origin_y"]
    r = math.radians(course["rotation_deg"])
    c, s = math.cos(r), math.sin(r)
    return (c * dx + s * dy, -s * dx + c * dy, wrap_deg(heading_deg - course["rotation_deg"]))

def load_frame(robot_number:int) -> tuple:
    '''Read optitrack_variables.config: returns (frame, course) for this robot.'''
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
    course = {k: parser.getfloat("course", k, fallback=0.0) for k in ("origin_x", "origin_y", "rotation_deg")}
    return frame, course

class LidarNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("LidarNode")
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # Subscribe to lidar
        self.is_simulation = None

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        if len(msg.ranges) <= 360: 
            self.is_simulation = True
        else:
            self.is_simulation = False

class ObstacleCourseNode(Node):
    '''Node to navigate obstacle course, using pose from either gazebo odometer or optitrack and lidar scan data'''
    def __init__(self, is_simulation:bool=True, calibrate:bool=False):
        super().__init__('obstaclecourse')
        self.get_logger().info("Starting ObstacleCourseNode")

        self.is_simulation = is_simulation
        self.calibrate = calibrate # Real robot: only print poses, never drive
        self.raw_pose = None # Pose exactly as Optitrack reported it (real robot only)
        self.frame = self.course = None
        self._ticks = 0

        if is_simulation:
            self.max_translate_velocity = 1.4
        else:
            self.max_translate_velocity = 0.3 # Please keep this in place; 0.3m/s is more than fast enough 

        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # Subscribe to LiDAR scan data

        # Subscribe to the dynamic_pose topic from Gazebo that publishes ground-truth pose data
        if self.is_simulation:
            self.subscription = self.create_subscription(Odometry, 'odom', self.odometer_callback, 2)
        else:
            qos_profile = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
            # On the Bingda robots ROS_DOMAIN_ID is the robot's number, so the Optitrack topic is picked automatically
            domain = os.environ.get("ROS_DOMAIN_ID", "")
            self.robot_number = int(domain) if domain.isdigit() and 1 <= int(domain) <= 99 else 3
            self.frame, self.course = load_frame(self.robot_number)
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
        self.last_scan = None
        self.last_scan_msg = None
        # Integer (grid_x, grid_y) cell -> time it was last seen (time.monotonic()).
        # Cells are forgotten after MAP_MEMORY_SECONDS without being seen.
        self.obstacle_map = {}
        self.desired_movement = "FRONT"
        self.previous_movement = "FRONT"
        self.committed_movement = None
        self.commit_start_position = None
        self.memory_clearances = {
            "front": math.inf,
            "right": math.inf,
            "back": math.inf,
            "left": math.inf,
        }

    def sub_scan_callback(self, msg):
        """Add every valid LiDAR return to persistent world-frame memory."""
        self.last_scan = np.asarray(msg.ranges, dtype=float)
        self.last_scan_msg = msg
        self.update_obstacle_map(msg)
        self.update_avoidance_movement()
        # The scan has been consumed; clear it so it is never reused.
        self.last_scan = None
        self.last_scan_msg = None

    def scan_to_world_coordinates(self, msg) -> np.ndarray:
        """Convert one LaserScan message into world-frame XY coordinates."""
        if self.pose is None:
            return np.empty((0, 2), dtype=float)

        ranges = np.asarray(msg.ranges, dtype=float)
        valid = (
            np.isfinite(ranges)
            & (ranges > 0.0)
            & (ranges < MAP_MAXIMUM_SCAN_RANGE)
        )
        if math.isfinite(float(msg.range_max)):
            valid &= ranges <= float(msg.range_max)
        indices = np.flatnonzero(valid)
        if indices.size == 0:
            return np.empty((0, 2), dtype=float)

        robot_heading = math.radians(float(self.pose[2]))
        scan_angles = (
            float(msg.angle_min)
            + indices * float(msg.angle_increment)
        )
        world_angles = robot_heading + LASER_YAW_OFFSET + scan_angles
        sensor = np.array((
            float(self.pose[0]) + LASER_X_OFFSET * math.cos(robot_heading),
            float(self.pose[1]) + LASER_X_OFFSET * math.sin(robot_heading),
        ))
        valid_ranges = ranges[indices]
        return sensor + np.column_stack((
            valid_ranges * np.cos(world_angles),
            valid_ranges * np.sin(world_angles),
        ))

    def update_obstacle_map(self, msg) -> None:
        """Quantize this scan to 0.02 m cells, refresh their last-seen time,
        and forget cells that have not been seen for MAP_MEMORY_SECONDS."""
        now = time.monotonic()
        world_points = self.scan_to_world_coordinates(msg)
        if len(world_points) > 0:
            grid_cells = np.rint(world_points / MAP_RESOLUTION).astype(np.int64)
            for cell in grid_cells:
                self.obstacle_map[(int(cell[0]), int(cell[1]))] = now

        # Fade out stale cells (runs even when this scan saw nothing).
        cutoff = now - MAP_MEMORY_SECONDS
        stale = [cell for cell, seen in self.obstacle_map.items() if seen < cutoff]
        for cell in stale:
            del self.obstacle_map[cell]

    def remembered_obstacle_points(self) -> np.ndarray:
        """Return remembered occupied-cell centres in world coordinates."""
        if not self.obstacle_map:
            return np.empty((0, 2), dtype=float)
        return np.asarray(tuple(self.obstacle_map), dtype=float) * MAP_RESOLUTION

    def remembered_clearances(self):
        """Return four-direction clearances measured from remembered cells."""
        points = self.remembered_obstacle_points()
        if self.pose is None or len(points) == 0:
            return {
                "front": math.inf,
                "right": math.inf,
                "back": math.inf,
                "left": math.inf,
            }

        heading = math.radians(float(self.pose[2]))
        cosine, sine = math.cos(heading), math.sin(heading)
        relative = points - np.asarray(self.pose[:2], dtype=float)
        forward = relative[:, 0] * cosine + relative[:, 1] * sine
        left = -relative[:, 0] * sine + relative[:, 1] * cosine

        def nearest(along, across):
            inside = (
                (along > 0.0)
                & (np.abs(across) <= MAP_CORRIDOR_HALF_WIDTH)
            )
            return float(np.min(along[inside])) if np.any(inside) else math.inf

        return {
            "front": nearest(forward, left),
            "right": nearest(-left, forward),
            "back": nearest(-forward, left),
            "left": nearest(left, forward),
        }

    def update_avoidance_movement(self) -> None:
        """Select one linear direction after each obstacle-map update."""
        if self.pose is None:
            self.desired_movement = "STOP"
            return

        clearances = self.remembered_clearances()
        self.memory_clearances = clearances
        front_blocked = clearances["front"] < AVOIDANCE_CLEARANCE
        right_blocked = clearances["right"] < AVOIDANCE_CLEARANCE
        back_blocked = clearances["back"] < AVOIDANCE_CLEARANCE
        left_blocked = clearances["left"] < AVOIDANCE_CLEARANCE

        current_movement = self.desired_movement
        decision_movement = (
            self.previous_movement
            if current_movement in ("STOP", "DIAG")
            else current_movement
        )
        blocked = {
            "FRONT": front_blocked,
            "RIGHT": right_blocked,
            "BACK": back_blocked,
            "LEFT": left_blocked,
        }
        blocked_count = sum(blocked.values())

        # A newly selected FRONT, RIGHT or LEFT direction is held so the
        # new corridor is entered before preferred-direction scanning can
        # switch again. A real obstacle cancels the hold early.
        # DIAG is only allowed on the tick a commitment finishes its full
        # 0.3 m without being blocked.
        commit_completed = False
        if self.committed_movement is not None:
            travelled = float(np.linalg.norm(
                np.asarray(self.pose[:2], dtype=float)
                - self.commit_start_position
            ))
            # End the commit early only if an obstacle appears in the
            # direction being travelled.
            commitment_blocked = blocked[self.committed_movement]
            if (
                travelled < TRANSITION_COMMIT_DISTANCE
                and not commitment_blocked
            ):
                self.desired_movement = self.committed_movement
                return
            # Only a finished FRONT or RIGHT hold unlocks DIAG.
            commit_completed = (
                not commitment_blocked
                and self.committed_movement in ("FRONT", "RIGHT")
            )
            self.committed_movement = None
            self.commit_start_position = None

        def same_axis(first, second):
            forward_axis = {"FRONT", "BACK"}
            return (first in forward_axis) == (second in forward_axis)

        # Preferred-direction scanning happens first. FRONT and BACK look
        # right; RIGHT and LEFT look front.
        watched_direction = {
            "FRONT": "RIGHT",
            "RIGHT": "FRONT",
            "BACK": "RIGHT",
            "LEFT": "FRONT",
        }[decision_movement]

        both_preferred_clear = not front_blocked and not right_blocked
        previous_override = None
        if both_preferred_clear and (
            current_movement == "DIAG" or commit_completed
        ):
            next_movement = "DIAG"
        elif both_preferred_clear:
            # Before switching to DIAG, commit 0.3 m along FRONT or RIGHT.
            # Keep going in the current preferred direction; from LEFT/BACK/
            # STOP use the watched preferred direction.
            next_movement = (
                current_movement
                if current_movement in ("FRONT", "RIGHT")
                else watched_direction
            )
            if (
                next_movement != current_movement
                and current_movement not in ("STOP", "DIAG")
            ):
                self.previous_movement = current_movement
            self.committed_movement = next_movement
            self.commit_start_position = np.asarray(
                self.pose[:2],
                dtype=float,
            ).copy()
            self.desired_movement = next_movement
            return
        elif current_movement == "DIAG" and right_blocked and not front_blocked:
            # The right half of the diagonal closed: continue only forward and
            # make RIGHT the logical previous movement for later elimination.
            next_movement = "FRONT"
            previous_override = "RIGHT"
        elif current_movement == "DIAG" and front_blocked and not right_blocked:
            # The front half of the diagonal closed: continue only right and
            # make FRONT the logical previous movement for later elimination.
            next_movement = "RIGHT"
            previous_override = "FRONT"
        elif not blocked[watched_direction]:
            next_movement = watched_direction
        # If the watched route is blocked but current travel remains clear,
        # simply continue current movement.
        elif not blocked[decision_movement]:
            next_movement = decision_movement
        elif blocked_count == 1:
            # With one obstacle, leave the blocked axis using the preferred
            # FRONT/RIGHT directions only.
            next_movement = (
                "RIGHT"
                if decision_movement in ("FRONT", "BACK")
                else "FRONT"
            )
        elif blocked_count == 2:
            clear_directions = [
                direction for direction, is_blocked in blocked.items()
                if not is_blocked
            ]
            front_clear = not front_blocked
            right_clear = not right_blocked

            if front_clear and right_clear:
                # LEFT and BACK are blocked.
                next_movement = "FRONT"
            elif front_clear != right_clear:
                # Exactly one preferred direction is clear; call it X.
                preferred = "FRONT" if front_clear else "RIGHT"
                if same_axis(decision_movement, preferred):
                    next_movement = next(
                        direction for direction in clear_directions
                        if direction != preferred
                    )
                else:
                    next_movement = preferred
            else:
                # FRONT and RIGHT are blocked; LEFT and BACK are clear. Choose
                # the clear direction on the axis opposite current movement.
                next_movement = next(
                    direction for direction in clear_directions
                    if not same_axis(decision_movement, direction)
                )
        elif blocked_count == 3:
            # Exactly one direction remains available.
            next_movement = next(
                direction for direction, is_blocked in blocked.items()
                if not is_blocked
            )
        else:
            # No linear direction is available.
            next_movement = "STOP"

        if previous_override is not None:
            self.previous_movement = previous_override
        elif (
            next_movement != current_movement
            and current_movement not in ("STOP", "DIAG")
        ):
            self.previous_movement = current_movement

        if (
            next_movement in ("FRONT", "RIGHT", "LEFT")
            and next_movement != current_movement
        ):
            self.committed_movement = next_movement
            self.commit_start_position = np.asarray(
                self.pose[:2],
                dtype=float,
            ).copy()
        self.desired_movement = next_movement

    def publish_desired_movement(self) -> None:
        """Publish the direction chosen by the most recent LiDAR scan."""
        commands = {
            "FRONT": (LINEAR_SPEED, 0.0),
            "RIGHT": (0.0, -LINEAR_SPEED),
            "BACK": (-LINEAR_SPEED, 0.0),
            "LEFT": (0.0, LINEAR_SPEED),
            "DIAG": (
                LINEAR_SPEED / math.sqrt(2.0),
                -LINEAR_SPEED / math.sqrt(2.0),
            ),
            "STOP": (0.0, 0.0),
        }
        x, y = commands[self.desired_movement]
        self.move_2D(x, y, 0.0)

    def yaw_from_quaternion(self, q):
        '''Returns yaw angle (in rad) for orientation based on given quaternion input q'''
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def optitrack_callback(self, msg:PoseStamped):
        '''Callback to calculate 2D pose info from Optitrack node. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        x, y = msg.pose.position.x, msg.pose.position.y
        heading = np.rad2deg(self.yaw_from_quaternion(msg.pose.orientation))
        self.raw_pose = np.array((x,y,heading))
        x, y, heading = optitrack_to_maze(x, y, heading, self.frame) # CA2 maze frame, see optitrack_variables.config
        self.pose = np.array((x,y,heading))
        return self.pose

    def to_course(self, pose=None):
        '''Convert a pose [x, y, heading] (default: self.pose) from the CA2 maze frame to the final-project course frame. Optional; see [course] in optitrack_variables.config'''
        pose = self.pose if pose is None else pose
        return np.array(maze_to_course(pose[0], pose[1], pose[2], self.course)) if self.course else np.array(pose)

    def odometer_callback(self, msg):
        '''Callback to calculate 2D pose info from Gazebo odomoter. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        latest_pose_msg = msg.pose.pose 
        heading = np.rad2deg(self.yaw_from_quaternion(latest_pose_msg.orientation))
        self.pose = np.array((latest_pose_msg.position.x, latest_pose_msg.position.y, heading))
        return self.pose

    def move_2D(self, x:float=0.0, y:float=0.0, turn:float=0.0):
        '''Publishes a Twist message to ROS to move a robot. Inputs are x and y linear velocities, as well as turn (z-axis yaw) angular velocity.'''
        twist_msg = Twist()
        x = np.clip(x, -self.max_translate_velocity, self.max_translate_velocity)
        y = np.clip(y, -self.max_translate_velocity, self.max_translate_velocity)
        turn = np.clip(turn, -self.max_translate_velocity*2, self.max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.publisher_.publish(twist_msg)

    def timer_callback(self):
        """Controller loop. Insert path planning and PID control logic here"""
        if self.pose is None:
            print("No pose detected")
            return # Does not run if no pose received from Odom or Optitrack

        if self.calibrate and self.raw_pose is not None:
            self._ticks += 1
            if self._ticks % 10 == 0: # twice a second
                ox, oy, oh = self.raw_pose
                self.get_logger().info(f"optitrack ({ox:+.3f}, {oy:+.3f}, {oh:+7.1f} deg)  ->  maze ({self.pose[0]:+.3f}, {self.pose[1]:+.3f}, {self.pose[2]:+7.1f} deg)")
            return # Calibration mode: look, don't drive

        self.publish_desired_movement()

        self._ticks += 1
        if self._ticks % 20 == 0:
            front = self.memory_clearances["front"]
            right = self.memory_clearances["right"]
            back = self.memory_clearances["back"]
            left = self.memory_clearances["left"]
            front_text = "clear" if math.isinf(front) else f"{front:.2f} m"
            right_text = "clear" if math.isinf(right) else f"{right:.2f} m"
            back_text = "clear" if math.isinf(back) else f"{back:.2f} m"
            left_text = "clear" if math.isinf(left) else f"{left:.2f} m"
            self.get_logger().info(
                f"motion={self.desired_movement}, "
                f"previous={self.previous_movement}, "
                f"commit={self.committed_movement or 'none'}, "
                f"pose={self.pose}, "
                f"map_cells={len(self.obstacle_map)}, "
                f"front={front_text}, right={right_text}, "
                f"back={back_text}, left={left_text}"
            )
                

def main(args=None):
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
