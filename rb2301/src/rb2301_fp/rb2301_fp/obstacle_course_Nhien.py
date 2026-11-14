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
REAL_POSE_HEADING_CORRECTION_DEG = -90.0

#NHIEN's CONSTANT

#SEGMENT INDICES
FIRST_SEGMENT = 0
SECOND_SEGMENT = 1
THIRD_SEGMENT_GAP = 2
THIRD_SEGMENT_SLIDER_1 = 3
THIRD_SEGMENT_SLIDER_2 = 4

VISION_MAP_SIZE = 11    
VISION_MAP_CELL_SIZE = 0.1
VISION_MAP_FURTHEST =  1.32 * VISION_MAP_CELL_SIZE*VISION_MAP_SIZE/2
ROBOT_FREE_HALF_WIDTH_CELLS = 1  # Clear the centre 3x3 cells.
GOAL_TOLERANCE = 0.07
LOOK_AHEAD_DIST = 0.3
MAX_TRANSLATE_VELOCITY = 0.4
MIN_TRANSLATE_VELOCITY = 0.15
MAP_REFRESH_INTERVAL = 10.0
PROGRESS_COOLDOWN = 5.0
ROUTE_SECTOR_HALF_WIDTH_DEG = 45.0
ROUTE_END_DISTANCE = 0.70
REVERSE_ROUTE_END_DISTANCE_BONUS = 0.20
CARROT_SWITCH_COOLDOWN = 5.0
OBSTACLE_SWITCH_COOLDOWN = 2
OBSTACLE_SWITCH_DISTANCE = 0.1
CARROT_REVERSE_ENTER_ANGLE = 100.0
CARROT_FORWARD_ENTER_ANGLE = 80.0
ALIGN_BEFORE_DRIVE_ANGLE = 20.0
HEADING_DEADBAND = 1.0

KP = 0.8
KI = 0.05
KD = 0.15
KPT = 0.7
KIT = 0.15
KDT = 0.1
DISTANCE_INTEGRAL_LIMIT = 1.0
HEADING_INTEGRAL_LIMIT = 1.5
TURN_PENALTY = 1.0  # Extra path cost for a 90-degree turn, in cell-step units.
FAILURE_RECOVERY_OBSTACLE_DISTANCE = 0.2
FAILURE_RECOVERY_SPEED = 0.3    
#HELPER FOR ASTAR

class SearchNode:
    '''SearchNode is a helper class for Astar Function'''
    def __init__(self,cell,g,h,parent=None,order=None,direction=None):

        self.cell = cell # Cell is (i,j) in Grid Coordinates
        self.direction = direction # Direction of the move used to reach this cell
        self.g = g # Cost to go from start to the current cell
        self.h = h # Heuristic cost from the goal to the current cell
        self.f = g + h
        self.parent = parent # This is for path reconstruction
        self.order = order # Sorting
    def cost(self):
        return self.f

    def __lt__(self, other): # Less than inbuilt function, useful for heapq
        return (self.f, self.order) < (other.f, other.order) # this helps for tie breaker situation
    
    def __eq__(self, value): # Equal to inbuilt function, useful for heapq
        return self.f == value.f


def wrap_deg(angle:float) -> float:
    '''Wrap an angle in degrees into [-180, 180).'''
    return (angle + 180.0) % 360.0 - 180.0

def optitrack_to_maze(x:float, y:float, heading_deg:float, frame:dict) -> tuple:
    '''Optitrack-frame pose (x, y, heading in degrees) -> CA2 maze-frame pose (see optitrack_variables.config).'''
    dx, dy = x - frame["origin_x"], y - frame["origin_y"]
    # On the real course, moving in world +X makes raw OptiTrack Y decrease,
    # while moving in world +Y makes raw OptiTrack X increase. Therefore the
    # position mapping is approximately (-raw_y, raw_x), requiring the
    # opposite sign from the legacy configured position rotation.
    r = math.radians(-frame["rotation_deg"])
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

        #Nhien's Properties
        self.progress = FIRST_SEGMENT
        self.bias = "LEFT"
        # States: SCANNING, MOVING, FINISH
        self.state = "SCANNING"
        self.visionMap = np.zeros((VISION_MAP_SIZE, VISION_MAP_SIZE), dtype = int)
        self.vision_map_pose = None
        self.last_scan = None
        self.chosen_path = []
        self.current_goal = ()
        self.failure_recovery_direction = None
        self.path_finished = False
        self.finished_path_route_sector = None
        self.routeHeading = 0
        self.originHeading = 0
        #ASTAR + Carrot Properties
        '''PID Variables'''
        self.error_distance_prev = None
        self.error_heading_prev = None
        self.error_distance_integral = 0.0
        self.error_heading_integral = 0.0
        self.PID_prev_time = self.get_clock().now()
        self.driving_backward = False
        self.last_motion_backward = False
        self.last_carrot_direction_switch_time = None
        self.last_obstacle_direction_switch_time = None
        self.last_progress_change_time = self.get_clock().now()
        '''Carrot Variables'''
        self.path_progress = 0.0 # Initialise as 0
        self.current_segment_idx = 0
        self.map_refresh_timer = self.create_timer(
            MAP_REFRESH_INTERVAL,
            self.request_map_refresh,
        )
        
    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        if len(msg.ranges) <= 360:
            self.last_scan = np.array(msg.ranges)
        else:
            self.last_scan = np.array(msg.ranges)[::2]

    def request_map_refresh(self):
        """Force a fresh scan map and A* path during prolonged movement."""
        if self.state != "MOVING":
            return

        self.last_motion_backward = self.driving_backward
        self.state = "SCANNING"
        self.current_goal = ()
        self.chosen_path = []
        self.move_2D(0, 0, 0)
        print("Periodic 5s map refresh")

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
        # Real pose convention: world +x is 0 degrees and world +y is 90
        # degrees. Remove the legacy 90-degree heading assumption.
        heading = wrap_deg(heading + REAL_POSE_HEADING_CORRECTION_DEG)
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
        # Preserve the last actual translation direction across zero-velocity
        # stops between consecutive A* paths.
        if x < -1e-3:
            self.last_motion_backward = True
        elif x > 1e-3:
            self.last_motion_backward = False
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.publisher_.publish(twist_msg)

    def timer_callback(self):
        """Run one controller tick and dispatch the active course segment.

        This method intentionally contains only top-level orchestration. Each
        segment handler selects its world-frame route, runs navigation, and
        processes the one-shot event produced when an A* path is completed.
        """
        if self.pose is None:
            print("No pose detected")
            return

        if self._handle_calibration_mode():
            return

        if self.progress == FIRST_SEGMENT:
            self._handle_first_segment()
        elif self.progress == SECOND_SEGMENT:
            self._handle_second_segment()

    def _handle_calibration_mode(self):
        """Log raw and converted OptiTrack poses without driving.

        Returns True when calibration mode consumed this controller tick.
        """
        if not self.calibrate or self.raw_pose is None:
            return False

        self._ticks += 1
        if self._ticks % 10 == 0:  # The controller runs at 20 Hz: log at 2 Hz.
            ox, oy, oh = self.raw_pose
            self.get_logger().info(
                f"optitrack ({ox:+.3f}, {oy:+.3f}, {oh:+7.1f} deg)  ->  "
                f"maze ({self.pose[0]:+.3f}, {self.pose[1]:+.3f}, "
                f"{self.pose[2]:+7.1f} deg)"
            )
        return True

    def _handle_first_segment(self):
        """Navigate world +X and switch to segment two at its route end."""
        self._run_route_segment(
            route_heading_deg=0.0,
            next_progress=SECOND_SEGMENT,
        )

    def _handle_second_segment(self):
        """Navigate world -Y and return to segment one at its route end."""
        self._run_route_segment(
            route_heading_deg=-90.0,
            next_progress=FIRST_SEGMENT,
        )

    def _run_route_segment(self, route_heading_deg, next_progress):
        """Run shared A*/carrot navigation for one cardinal course segment.

        Route-end detection is deliberately evaluated only after a path has
        reached its goal. This prevents a nearby obstacle from changing the
        course segment in the middle of an otherwise valid path.
        """
        now = self.get_clock().now()
        self.routeHeading = route_heading_deg
        self.originHeading = (
            self.routeHeading + (180.0 if self.driving_backward else 0.0)
        ) % 360.0

        self.forward_and_dodge()
        if not self.path_finished:
            return

        if self._completed_path_reached_route_end(now):
            self.progress = next_progress
            self.last_progress_change_time = now
            self.state = "SCANNING"

        # path_finished is a one-shot event. A new completed path must be
        # observed before route-end detection is evaluated again.
        self.path_finished = False
        self.finished_path_route_sector = None

    def _completed_path_reached_route_end(self, now):
        """Return whether the saved route sector marks the end of a segment."""
        route_sector = self.finished_path_route_sector
        if route_sector is None:
            return False

        cooldown_elapsed = (
            now - self.last_progress_change_time
        ).nanoseconds >= PROGRESS_COOLDOWN * 1e9
        if not cooldown_elapsed:
            return False

        # When reversing, allow the route boundary to be slightly farther
        # from the rear-facing travel direction.
        end_distance = (
            ROUTE_END_DISTANCE
            + REVERSE_ROUTE_END_DISTANCE_BONUS * float(self.driving_backward)
        )
        return bool(np.all(route_sector < end_distance))

    def forward_and_dodge(self):
        if self.state == "SCANNING":
            if self.last_scan is None:
                self.move_2D(0, 0, 0)
                return
            # The scan grid is centred on this pose but its axes stay aligned
            # with the world frame while the resulting path is followed.
            self.vision_map_pose = self.pose.copy()
            recovery_scan = np.asarray(self.last_scan, dtype=float).copy()
            self.estimateObstacleFromScans()
            
            lowest_cost = 99999
            self.chosen_path = []

            # Choose the map edge along the cardinal world route. In this grid,
            # decreasing row is world +x and decreasing column is world +y.
            robot_cell = VISION_MAP_SIZE // 2
            route_angle = math.radians(self.routeHeading)
            forward_component = math.cos(route_angle)
            left_component = math.sin(route_angle)

            if abs(forward_component) >= abs(left_component):
                layer_indices = (
                    range(1, robot_cell-1)
                    if forward_component >= 0.0
                    else range(VISION_MAP_SIZE - 2, robot_cell-1, -1)
                )
                goal_layers = [
                    [(row, col) for col in range(VISION_MAP_SIZE)]
                    for row in layer_indices
                ]
            else:
                layer_indices = (
                    range(1, robot_cell-1)
                    if left_component >= 0.0
                    else range(VISION_MAP_SIZE - 2, robot_cell-1, -1)
                )
                goal_layers = [
                    [(row, col) for row in range(VISION_MAP_SIZE)]
                    for col in layer_indices
                ]

            for candidate_layer in goal_layers:
                for goal in candidate_layer:
                    if self.visionMap[goal] >= 99:
                        continue
                    path, cost = self.Astar(goal=goal)
                    if cost < lowest_cost:
                        self.chosen_path = path
                        lowest_cost = cost
                if lowest_cost < 99999:
                    break

            map_display = np.empty((VISION_MAP_SIZE, VISION_MAP_SIZE), dtype=str)
            for r in range(VISION_MAP_SIZE):
                for c in range(VISION_MAP_SIZE):
                    if self.visionMap[r,c] == 99:
                        map_display[r,c] = "X"
                    elif self.visionMap[r,c] == 75:
                        map_display[r,c] = "O"
                    else:
                        map_display[r,c] = "-"
            for j in self.chosen_path:
                map_display[j[0], j[1]] = "*"
            print(map_display)

            if lowest_cost == 99999:
                print("A* failed; using front/back obstacle recovery")
                self.recover_from_planning_failure(recovery_scan)
                return

            self.state = "MOVING"
            self.failure_recovery_direction = None
            # Count the next guaranteed refresh interval from this completed
            # map, rather than from node startup or time spent scanning.
            self.map_refresh_timer.reset()
            self.chosen_path = self.simplify_path(self.chosen_path)
            self.current_goal = self.cell_to_world(
                self.vision_map_pose[0], self.vision_map_pose[1],
                self.chosen_path[-1][0], self.chosen_path[-1][1], self.vision_map_pose[2]
            )
            self.path_progress = 0.0
            self.current_segment_idx = 0
            self.driving_backward = self.last_motion_backward
            self.error_distance_prev = None
            self.error_heading_prev = None
            self.error_distance_integral = 0.0
            self.error_heading_integral = 0.0
            self.PID_prev_time = self.get_clock().now()
        elif self.state == "MOVING":
            carrot = self.get_carrot(
                robot_curr_position=self.pose[:2],
                path = self.chosen_path,
                look_ahead_dist=LOOK_AHEAD_DIST
            )

            if carrot is False:
                print("Carrot generation failed; using front/back obstacle recovery")
                self.recover_from_planning_failure(self.last_scan)
                return
            carrot_dx, carrot_dy = self.current_xy_error(carrot)
            carrot_bearing = math.atan2(carrot_dy, carrot_dx)
            heading = math.radians(self.pose[2])
            carrot_heading_error = math.atan2(
                math.sin(carrot_bearing - heading),
                math.cos(carrot_bearing - heading),
            )
            # Hysteresis prevents rapid switching when the carrot is near 90
            # degrees from the car. Enter reverse past 100 degrees, but only
            # return to forward once it is within 80 degrees of the front.
            carrot_is_behind = abs(carrot_heading_error) > math.radians(
                CARROT_REVERSE_ENTER_ANGLE
            )
            carrot_is_in_front = abs(carrot_heading_error) < math.radians(
                CARROT_FORWARD_ENTER_ANGLE
            )

            # The laser link is mounted at yaw=pi: indices near 0/360 face
            # robot-forward and the middle of the scan faces robot-backward.
            # Check a 40-degree sector centred on each travel direction.
            front_obstacle = False
            back_obstacle = False
            if self.last_scan is not None:
                scan = np.asarray(self.last_scan)
                half_sector = len(scan) // 18
                rear_centre = len(scan) // 2
                front_sector = np.concatenate(
                    (scan[-half_sector:], scan[:half_sector])
                )
                back_sector = scan[
                    rear_centre - half_sector:
                    rear_centre + half_sector
                ]
                front_valid = np.isfinite(front_sector) & (front_sector > 0.0)
                back_valid = np.isfinite(back_sector) & (back_sector > 0.0)
                front_obstacle = bool(
                    np.any(
                        front_valid
                        & (front_sector < OBSTACLE_SWITCH_DISTANCE)
                    )
                )
                back_obstacle = bool(
                    np.any(
                        back_valid
                        & (back_sector < OBSTACLE_SWITCH_DISTANCE*1.2)
                    )
                )

            reverse = self.driving_backward
            switch_reason = None
            consume_carrot_switch = False
            consume_obstacle_switch = False
            now = self.get_clock().now()
            carrot_switch_ready = (
                self.last_carrot_direction_switch_time is None
                or (
                    now - self.last_carrot_direction_switch_time
                ).nanoseconds >= CARROT_SWITCH_COOLDOWN * 1e9
            )
            obstacle_switch_ready = (
                self.last_obstacle_direction_switch_time is None
                or (
                    now - self.last_obstacle_direction_switch_time
                ).nanoseconds >= OBSTACLE_SWITCH_COOLDOWN * 1e9
            )
            if self.driving_backward:
                # The rear is the direction of travel while reversing.
                if back_obstacle:
                    if not obstacle_switch_ready:
                        self.move_2D(0, 0, 0)
                        return
                    if front_obstacle:
                        print("Front and back blocked, stopping")
                        self.move_2D(0, 0, 0)
                        return
                    reverse = False
                    switch_reason = "Back blocked, driving forward"
                    consume_obstacle_switch = True
                elif (
                    carrot_is_in_front
                    and carrot_switch_ready
                    and not front_obstacle
                ):
                    reverse = False
                    switch_reason = "Carrot in front, driving forward"
                    consume_carrot_switch = True
            else:
                if front_obstacle:
                    if not obstacle_switch_ready:
                        self.move_2D(0, 0, 0)
                        return
                    if back_obstacle:
                        print("Front and back blocked, stopping")
                        self.move_2D(0, 0, 0)
                        return
                    reverse = True
                    switch_reason = "Front blocked, reversing"
                    consume_obstacle_switch = True
                elif (
                    carrot_is_behind
                    and carrot_switch_ready
                    and not back_obstacle
                ):
                    reverse = True
                    switch_reason = "Carrot behind, reversing"
                    consume_carrot_switch = True

            if reverse != self.driving_backward:
                # Avoid a derivative kick when the target heading flips by 180 degrees.
                self.error_heading_prev = None
                self.error_distance_prev = None
                self.error_distance_integral = 0.0
                self.error_heading_integral = 0.0
                self.driving_backward = reverse
                if consume_carrot_switch:
                    self.last_carrot_direction_switch_time = now
                if consume_obstacle_switch:
                    self.last_obstacle_direction_switch_time = now
                    # An obstacle decision has priority. Prevent the carrot
                    # from immediately selecting the blocked direction again.
                    self.last_carrot_direction_switch_time = now
                if switch_reason is not None:
                    print(switch_reason)
                self.originHeading = (
                    self.routeHeading + (180 if reverse else 0)
                ) % 360

            v_final, vy, w = self.PID(
                carrot, KP, KI, KD, KPT, KIT, KDT, reverse=reverse
            )
            if self.last_obstacle_direction_switch_time is not None:
                obstacle_switch_age = (
                    self.get_clock().now()
                    - self.last_obstacle_direction_switch_time
                ).nanoseconds / 1e9
                if obstacle_switch_age < OBSTACLE_SWITCH_COOLDOWN:
                    # A front/rear blockage switch starts with rotation only.
                    # Translation resumes after the obstacle cooldown.
                    v_final = 0.0
            self.move_2D(v_final, vy, w)
            goal_error = self.current_xy_error(self.current_goal)
            goal_distance = math.hypot(goal_error[0], goal_error[1])
            path_length = self.cumulative_dist_segments[-1]
            path_completed = self.path_progress >= path_length - GOAL_TOLERANCE

            if goal_distance < GOAL_TOLERANCE and path_completed:
                self.finished_path_route_sector = self.world_scan_sector(
                    self.routeHeading,
                    ROUTE_SECTOR_HALF_WIDTH_DEG,
                )
                self.path_finished = True
                self.state = "SCANNING"
                self.current_goal = ()
                print("DONE GOAL")
                self.last_motion_backward = self.driving_backward
                self.move_2D(0,0,0)
                self.originHeading = (
                    self.routeHeading
                    + (180 if self.driving_backward else 0)
                ) % 360
    def simplify_path(self,path:list):
        if not path: # Edge case: Empty list recieved
            return []
        if len(path) == 1: # Edge case: 1 Point only
            return path
        path_idx = 1
        new_path = [path[path_idx-1]]
        
        while path_idx < len(path) - 1:
            prev_node = np.array(path[path_idx-1])
            current_node = np.array(path[path_idx])
            next_node = np.array(path[path_idx+1])
            din = prev_node-current_node
            dout = current_node-next_node
            if not np.array_equal(din,dout): 
            # Compare the change, 
            # if there is a difference in the direction of increment
            # din changes in x, dout changes in y, means its an actual waypoint to go to.
                new_path.append(path[path_idx])
            path_idx += 1
        new_path.append(path[-1])
        return new_path # I also edited the draw_grid and _colors function to reflect the path more accurately        
    def PID(
        self, goal:tuple,
        Kp, Ki, Kd,
        Kp_theta, Ki_theta, Kd_theta,
        reverse=False,
    ):
        e_x_current , e_y_current = self.current_xy_error(goal)
        error_distance = math.sqrt(e_x_current*e_x_current + e_y_current*e_y_current)
        desired_heading = np.arctan2(e_y_current,e_x_current)
        if reverse:
            # Point the front away from the carrot; negative speed moves the rear toward it.
            desired_heading += math.pi
        current_heading = np.deg2rad(self.pose[2])
        error_heading =   desired_heading - current_heading
        error_heading = np.arctan2(
            np.sin(error_heading),
            np.cos(error_heading),

        )
        '''Clock for dt'''  
        current_time = self.get_clock().now() # use only one, this works in nanoseconds so if i call again it will be a different timestamp
        dt = current_time - self.PID_prev_time
        dt_seconds = dt.nanoseconds / 1e9

        if self.error_distance_prev is None:
            distance_derivative = 0.0
        elif dt_seconds > 1e-9:
            distance_derivative = (
                error_distance - self.error_distance_prev
            ) / dt_seconds
        else:
            distance_derivative = 0.0

        if self.error_heading_prev is None:
            heading_derivative = 0.0
        elif dt_seconds > 1e-9:
            # Wrap the difference so crossing +/-pi does not create a large
            # derivative kick.
            heading_delta = math.atan2(
                math.sin(error_heading - self.error_heading_prev),
                math.cos(error_heading - self.error_heading_prev),
            )
            heading_derivative = heading_delta / dt_seconds
        else:
            heading_derivative = 0.0

        # Ignore unusually long callback gaps and clamp accumulated error to
        # prevent integral windup while the output is speed-limited.
        if 1e-9 < dt_seconds < 0.5:
            self.error_distance_integral += error_distance * dt_seconds
            self.error_heading_integral += error_heading * dt_seconds
        self.error_distance_integral = float(np.clip(
            self.error_distance_integral,
            -DISTANCE_INTEGRAL_LIMIT,
            DISTANCE_INTEGRAL_LIMIT,
        ))
        self.error_heading_integral = float(np.clip(
            self.error_heading_integral,
            -HEADING_INTEGRAL_LIMIT,
            HEADING_INTEGRAL_LIMIT,
        ))

        Vx = (
            Kp * error_distance
            + Ki * self.error_distance_integral
            + Kd * distance_derivative
        )
        Vx = np.clip(Vx, 0.0, MAX_TRANSLATE_VELOCITY)
        heading_alignment = math.cos(error_heading)
        if abs(error_heading) > math.radians(ALIGN_BEFORE_DRIVE_ANGLE):
            # Finish the heading correction before translating. Driving and
            # applying a large angular correction together produces an arc.
            Vx = -0.1 if reverse else 0.1
        else:
            Vx = max(
                MIN_TRANSLATE_VELOCITY,
                Vx * max(0.0, heading_alignment),
            )
        if reverse:
            Vx = -Vx
        Vy = 0
        w = (
            Kp_theta * error_heading
            + Ki_theta * self.error_heading_integral
            + Kd_theta * heading_derivative
        )
        if abs(error_heading) < math.radians(HEADING_DEADBAND):
            # Prevent tiny pose noise from continuously steering an otherwise
            # straight run.
            w = 0.0
            self.error_heading_integral = 0.0
        

        self.get_logger().debug(f"Error Distance: {error_distance} Turning Correction Applied: {max(0.0, np.cos(error_heading))} Vx: {Vx}")
        
        self.get_logger().debug(f"Desired Heading: {desired_heading} Current Heading: {current_heading} Error Heading: {error_heading} Angular z: {w}")
        
        '''Save prev errors and prev PID time'''

        self.error_distance_prev = error_distance
        self.error_heading_prev = error_heading
        self.PID_prev_time = current_time

        return Vx, Vy,w 
    def current_xy_error(self,goal):
        '''Calculate Current Error''' 
        robot_point = (self.pose[0],self.pose[1])
        '''Calculate Current Error'''            
        e_x_current = goal[0]-robot_point[0]
        e_y_current = goal[1]-robot_point[1]
        return e_x_current, e_y_current

    def recover_from_planning_failure(self, scan):
        """Move away from a close front or rear obstacle after planning fails."""
        if scan is None:
            self.move_2D(0, 0, 0)
            print("Recovery stopped: no LiDAR scan")
            return

        scan = np.asarray(scan, dtype=float)
        if scan.size == 0:
            self.move_2D(0, 0, 0)
            print("Recovery stopped: empty LiDAR scan")
            return

        # The laser is mounted at yaw=pi: scan ends are robot-front and the
        # middle is robot-rear. Use the same 40-degree sectors as normal
        # obstacle-direction switching.
        half_sector = max(1, len(scan) // 18)
        rear_centre = len(scan) // 2
        front_sector = np.concatenate(
            (scan[-half_sector:], scan[:half_sector])
        )
        back_sector = scan[
            rear_centre - half_sector:
            rear_centre + half_sector
        ]

        def sector_blocked(sector):
            valid = np.isfinite(sector) & (sector > 0.0)
            return bool(np.any(
                valid
                & (sector < FAILURE_RECOVERY_OBSTACLE_DISTANCE)
            ))

        front_blocked = sector_blocked(front_sector)
        back_blocked = sector_blocked(back_sector)

        if front_blocked and not back_blocked:
            self.failure_recovery_direction = -1
            self.driving_backward = True
            self.move_2D(-FAILURE_RECOVERY_SPEED, 0, 0)
            print("Recovery: front blocked, moving backward")
        elif back_blocked and not front_blocked:
            self.failure_recovery_direction = 1
            self.driving_backward = False
            self.move_2D(FAILURE_RECOVERY_SPEED, 0, 0)
            print("Recovery: rear blocked, moving forward")
        elif front_blocked and back_blocked:
            self.failure_recovery_direction = 0
            self.move_2D(0, 0, 0)
            print("Recovery stopped: front and rear blocked")
        elif self.pose is None:
            self.move_2D(0, 0, 0)
            print("Recovery stopped: no pose")
        else:
            # Keep the last obstacle-triggered direction while both ends are
            # clear. Without this latch, clearing a rear obstacle immediately
            # selected reverse again and made the robot oscillate in place.
            if self.failure_recovery_direction not in (-1, 1):
                # No direction has been selected yet. Split the compass into
                # four cardinal sectors and choose forward for +X/+Y.
                heading = math.radians(self.pose[2])
                world_x = math.cos(heading)
                world_y = math.sin(heading)
                front_points_positive = (
                    world_x >= 0.0
                    if abs(world_x) >= abs(world_y)
                    else world_y >= 0.0
                )
                self.failure_recovery_direction = (
                    1 if front_points_positive else -1
                )

            if self.failure_recovery_direction > 0:
                self.driving_backward = False
                self.move_2D(FAILURE_RECOVERY_SPEED, 0, 0)
                print("Recovery: clear, continuing forward")
            else:
                self.driving_backward = True
                self.move_2D(-FAILURE_RECOVERY_SPEED, 0, 0)
                print("Recovery: clear, continuing backward")

    def world_scan_sector(self, world_heading_deg, half_width_deg):
        """Return the LiDAR rays centred on a world-frame direction."""
        if self.last_scan is None or self.pose is None:
            return None

        scan = np.asarray(self.last_scan)
        ray_count = len(scan)
        if ray_count == 0:
            return None

        # The laser is mounted at yaw=pi, so scan index 0 corresponds to the
        # robot's forward direction. Convert the requested world direction to
        # a robot-relative angle, then into its circular scan index.
        relative_heading = wrap_deg(world_heading_deg - self.pose[2]) % 360.0
        centre_index = int(round(relative_heading / 360.0 * ray_count)) % ray_count
        half_width_rays = max(
            0,
            int(round(half_width_deg / 360.0 * ray_count)),
        )
        indices = (
            centre_index
            + np.arange(-half_width_rays, half_width_rays + 1)
        ) % ray_count
        return scan[indices]
    
    def scan_to_cell(self, angle, value):
        """Convert a LiDAR point to the robot-centred, world-aligned grid."""
        robot_cell = VISION_MAP_SIZE // 2

        # LaserScan angles are relative to base_laser_link, which is mounted
        # at yaw=pi. Add the robot heading so the resulting offsets use fixed
        # world axes instead of rotating with the robot.
        world_angle = angle + math.pi + math.radians(self.vision_map_pose[2])
        world_x = math.cos(world_angle) * value
        world_y = math.sin(world_angle) * value

        row = int(round(robot_cell - world_x / VISION_MAP_CELL_SIZE))
        column = int(round(robot_cell - world_y / VISION_MAP_CELL_SIZE))
        return row, column

    def estimateObstacleFromScans(self):
        obs = []
        if self.last_scan is None:
            return
        # Every scan grid has a new robot-centred origin. Do not retain cells
        # from the preceding path segment in this new coordinate frame.
        self.visionMap.fill(0)
        scans = np.clip(self.last_scan,0.05,20)
        angle_increment = 2 * np.pi / max(1, len(scans) - 1)
        for i in range(len(scans)):
            visible = True
            value = scans[i]
            # LaserScan angle_min is -pi in the laser frame. Because the laser
            # is mounted backwards, indices near 0/360 face robot-forward and
            # the middle sample faces robot-backward.
            angle = -np.pi + i * angle_increment
            #Extend the detection until it reach the edge, everythin behind the scan is not visible -> obstacle also
            while value <= VISION_MAP_FURTHEST:
                cellX, cellY = self.scan_to_cell(angle, value)
                if 0<=cellX < VISION_MAP_SIZE and 0 <= cellY < VISION_MAP_SIZE:
                    if visible:
                        if self.visionMap[cellX,cellY] != 99:
                            self.visionMap[cellX, cellY] = 99
                            obs.append((cellX,cellY))
                    else:
                        # Do not let a later ray's occluded region erase an
                        # occupied or inflated cell from an earlier ray.
                        if self.visionMap[cellX, cellY] < 99:
                            self.visionMap[cellX, cellY] = 75
                    value += VISION_MAP_CELL_SIZE
                    visible=False
                else:
                    break

        # Inflate after processing every ray so occlusion cells cannot erase
        # the safety margin. Include horizontal and vertical neighbours as
        # well as diagonals; the old condition inflated diagonals only.
        for cell in obs:
            for row_offset in (-1, 0, 1):
                for column_offset in (-1, 0, 1):
                    if row_offset == 0 and column_offset == 0:
                        continue
                    row = cell[0] + row_offset
                    column = cell[1] + column_offset
                    if (
                        0 <= row < VISION_MAP_SIZE
                        and 0 <= column < VISION_MAP_SIZE
                    ):
                        self.visionMap[row, column] = 99

        # LiDAR returns inside the robot footprint are not navigational
        # obstacles. Keep the requested nine cells centred on the robot free,
        # including any raw hits, occlusion marks, or inflated cells.
        robot_cell = VISION_MAP_SIZE // 2
        free_start = max(0, robot_cell - ROBOT_FREE_HALF_WIDTH_CELLS)
        free_stop = min(
            VISION_MAP_SIZE,
            robot_cell + ROBOT_FREE_HALF_WIDTH_CELLS + 1,
        )
        self.visionMap[free_start:free_stop, free_start:free_stop] = 0
        self.last_scan = None
    

    #ASTAR FROM CA2
    def Astar(self,goal:tuple):
        start = (VISION_MAP_SIZE//2,VISION_MAP_SIZE//2)
        def heuristic(start:tuple,goal:tuple):
            '''Chosen heuristic is manhattan distance'''
            return abs(goal[0]-start[0]) + abs(goal[1]-start[1])

        # The grid axes are fixed to the world, so routeHeading maps directly
        # to the initial cardinal A* direction.
        route_angle = math.radians(self.routeHeading)
        forward_component = math.cos(route_angle)
        left_component = math.sin(route_angle)
        if abs(forward_component) >= abs(left_component):
            start_direction = (-1, 0) if forward_component >= 0.0 else (1, 0)
        else:
            start_direction = (0, -1) if left_component >= 0.0 else (0, 1)
        start_node = SearchNode(
            cell=start, g=0, h=heuristic(start, goal),
            parent=None, direction=start_direction,
        )
        openList = [start_node]
        open_by_state = {(start, start_direction): start_node}
        closed_states = set()

        grid_size_x = self.visionMap.shape[0]
        grid_size_y = self.visionMap.shape[1]

        while openList:
            current = openList.pop(0)
            current_state = (current.cell, current.direction)
            if open_by_state.get(current_state) is not current:
                continue
            del open_by_state[current_state]

            # Check if we have reached the goal
            if current.cell == goal:
                return self.reconstruct_path(current), current.cost()

            closed_states.add(current_state)

            # Check all neighbouring nodes
            '''Check the 4 directions of cell, reject index that are negative or outside of grid size'''
            
            neighbour_down = (current.cell[0] + 1, current.cell[1]) if grid_size_x > (current.cell[0] + 1) >= 0 else None
            neighbour_up = (current.cell[0] - 1, current.cell[1]) if grid_size_x > (current.cell[0] - 1) >= 0 else None
            neighbour_left = (current.cell[0], current.cell[1] + 1) if grid_size_y > (current.cell[1] + 1) >= 0 else None
            neighbour_right = (current.cell[0], current.cell[1] - 1) if grid_size_y > (current.cell[1] - 1) >= 0 else None
            neighbours = [neighbour_up,neighbour_down,neighbour_left,neighbour_right]

            for neighbour in neighbours:
                if neighbour is None:
                    continue # Skip invalid index nodes
                if self.visionMap[neighbour[0], neighbour[1]] >= 99:
                    continue

                direction = (neighbour[0] - current.cell[0], neighbour[1] - current.cell[1])
                state = (neighbour, direction)
                if state in closed_states:
                    continue

                turn_cost = (
                    TURN_PENALTY
                    if current.direction is not None and direction != current.direction
                    else 0.0
                )
                occupancy_cost = float(self.visionMap[neighbour[0], neighbour[1]])
                tentative_g = current.g + 1.0 + occupancy_cost + turn_cost
                existing = open_by_state.get(state)

                if existing is None:
                    node = SearchNode(
                        cell=neighbour,
                        g=tentative_g,
                        h=heuristic(neighbour, goal),
                        parent=current,
                        direction=direction,
                    )
                    open_by_state[state] = node
                    openList.append(node)
                elif tentative_g < existing.g:
                    existing.g = tentative_g
                    existing.f = tentative_g + existing.h
                    existing.parent = current

            openList.sort(key=lambda node: node.cost())
        return [], 99999 # Failure case
    #Return an array of tuple -> CellsIndices
    def reconstruct_path(self,current:SearchNode):
        path = []
        while current is not None:
            path.append(current.cell)
            current = current.parent
        return path[::-1] # The first node is the goal node, the last node is the next point to go to
    def cell_to_world(self,current_posX, current_posY, i, j, heading_deg) -> tuple:
        '''Convert world-aligned scan-grid indices to world (x, y).'''
        robot_cell = VISION_MAP_SIZE // 2
        world_x = VISION_MAP_CELL_SIZE * (robot_cell - i)
        world_y = VISION_MAP_CELL_SIZE * (robot_cell - j)
        return (
            current_posX + world_x,
            current_posY + world_y,
        )
    
    def get_carrot(self,robot_curr_position:tuple,path,look_ahead_dist):
        
        '''
        ### robot_curr_position
            - (x,y) position of the robot, do not give heading
        ### look_ahead_dist: distance of carrot
        ### path: list of points to the goal
        For N waypoints,
            N Path World Coord
            N - 1 Length segments
            N Cumulative Distance Segments
            0 <= self.current_segment_idx <= N - 2
            To adv to next segment, 0 <= self.current_segment_idx < N - 2
        '''
        if len(path) < 2:
            return False

        path_world_coord = [
            self.cell_to_world(
                self.vision_map_pose[0], self.vision_map_pose[1],
                cell[0], cell[1], self.vision_map_pose[2]
            )
            for cell in path
        ]
        length_segments = []
        t = 0.0 
        
        for i in range(len(path_world_coord)):
            if 0 < i:
                length_segments.append(math.dist(path_world_coord[i-1],path_world_coord[i]))
        if sum(length_segments) <= 1e-9:
                return path_world_coord[-1]
        self.cumulative_dist_segments = [0.0]
        for length in length_segments:
            self.cumulative_dist_segments.append(length + self.cumulative_dist_segments[-1])
        
        while True:
        # Calculate AB, AR and t
            A = np.array(path_world_coord[self.current_segment_idx])
            B = np.array(path_world_coord[self.current_segment_idx + 1])
            R = np.array(robot_curr_position)
            AB = B - A
            if np.linalg.norm(AB) <= 1e-9: # If the length segment is 0
                if self.current_segment_idx == len(path_world_coord) - 2 : # If its the maximum segment idx to check
                    t = 0.0 # Ensure t resets to 0
                    break
                self.current_segment_idx += 1 # Move to the next length segment since this one is 0
                continue
            AR = R - A
            t = max(0, min(1,np.dot(AR,AB) / np.dot(AB,AB))) # This ensures that the t given is on the line segment selected

        # If t more than 90% switch
            # When im on line segment 1, it is from point 1 to point 2
            if t >= 0.8 and self.current_segment_idx < len(path_world_coord) - 2: # Maximum segment_idx = N - 2
                self.current_segment_idx += 1 
                continue
            break
        P_projected = np.array(path_world_coord[self.current_segment_idx]) + t * (AB) 
        distance_to_projected = np.linalg.norm(P_projected-np.array(robot_curr_position)) # TO be used for debugging
        s_projected = self.cumulative_dist_segments[self.current_segment_idx] + t *length_segments[self.current_segment_idx] # length projected
        self.path_progress = max(self.path_progress, s_projected)  # S[i] + t * Li # Cumulative progress s of projection


        # Do not clamp the carrot to the final A* waypoint. Once lookahead
        # reaches beyond the path, extrapolate along the last nonzero segment.
        # The actual goal remains unchanged, so completion still occurs at the
        # A* endpoint while the controller retains a useful target distance.
        s_carrot = self.path_progress + look_ahead_dist
        valid_segments = [
            idx for idx, length in enumerate(length_segments)
            if length > 1e-9
        ]
        segment = valid_segments[0]

        if s_carrot >= self.cumulative_dist_segments[-1]:
            segment = valid_segments[-1]
        else:
            for idx in valid_segments:
                if (
                    self.cumulative_dist_segments[idx]
                    <= s_carrot
                    <= self.cumulative_dist_segments[idx + 1]
                ):
                    segment = idx
                    break

        d_in_segment = s_carrot - self.cumulative_dist_segments[segment]
        alpha = d_in_segment / length_segments[segment]
        carrot_A = np.array(path_world_coord[segment])
        carrot_B = np.array(path_world_coord[segment + 1])
        carrot = carrot_A + alpha * (carrot_B - carrot_A)

        self.get_logger().debug(
            f"Robot Segment: {self.current_segment_idx}, "
            f"Carrot Segment: {segment}, "
            f"Progress: {self.path_progress:.3f}, "
            f"Carrot: {tuple(carrot)}"
            f"t value: {t*100}%"
        )

        return tuple(carrot)

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
