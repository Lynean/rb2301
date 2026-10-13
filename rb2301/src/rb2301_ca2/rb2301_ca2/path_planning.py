import argparse
import configparser
import json
import math
import os
import time

import numpy as np
import heapq
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity

from rclpy.qos import (
    ReliabilityPolicy,
    QoSProfile,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from PIL import Image
from geometry_msgs.msg import Twist

# User imports
from dataclasses import dataclass


np.set_printoptions(
    2, suppress=True, threshold=np.inf
)  # Print numpy arrays to specified d.p., suppress scientific notation (e.g. 1e-5), and do not truncate

set_logger_level("waypoint", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG

occupancy_grid_resolution = 0.2 # Sim grid resolution, in metres per cell (the real maze uses the same 0.2m cells, see ca2_irl_layout.json)
max_translate_velocity = 1.4 # Overwritten in main() based on sim vs real-life; 0.4m/s cap for real life, please keep that in place

_PACKAGE_DIR = os.path.dirname(os.path.realpath(__file__))


# --- Coordinate conversion --------------------------------------------------
# A grid index (i, j) represents a CELL, not a point. That cell's world
# coordinate is its CENTER, e.g. cell [0, 0] is centred half a resolution-step
# away from the grid's origin corner, not exactly on it. This matches how the
# Gazebo world and the real maze are physically laid out (goal tape/markers
# sit in the middle of a cell, not on its boundary line).
def grid_to_world(i:int, j:int, origin:tuple, resolution:float=occupancy_grid_resolution) -> tuple:
    '''Convert grid index (i, j) to the world (x, y) coordinate of that cell's centre.'''
    return (origin[0] + (i + 0.5) * resolution, origin[1] + (j + 0.5) * resolution)

def world_to_grid(x:float, y:float, origin:tuple, resolution:float=occupancy_grid_resolution) -> tuple:
    '''Convert a world (x, y) coordinate to the grid index (i, j) of the cell containing it.'''
    return (int(np.floor((x - origin[0]) / resolution)), int(np.floor((y - origin[1]) / resolution)))


# --- Sim / real-life profiles -----------------------------------------------
# Run without arguments for the simulation. For the real maze pass
# --run test1 | test2 | full  (see README). Everything that differs between
# sim and real -- which map to load, the cell resolution, where the grid's
# [0, 0] corner sits in the frame your pose is reported in, the goal points,
# the speed cap -- lives in one of these profiles.
sim_config = {
    "map_file": "ca2_sim_map.npy",
    "origin": (-1.0, -5.0),
    "resolution": occupancy_grid_resolution,
    "goal_list": [(3.5, -3.5), (3.3, 0.3), (2.5, -3.5), (-0.3, -3.7)],
    "max_translate_velocity": 1.4,
}

IRL_RUNS = ("test1", "test2", "full")


# --- Real maze <-> Optitrack frame -------------------------------------------
# The real maze is a fixed layout (ca2_irl_map.npy + ca2_irl_layout.json, in
# "maze-local" metres: (0, 0) is the outer corner of array cell [0, 0], +x runs
# along array axis 0, +y along axis 1 -- exactly like the sim grid). Optitrack
# reports poses in ITS OWN frame, wherever the maze happens to be taped down.
# optitrack_variables.config holds the 3 numbers that relate the two:
#   origin_x, origin_y : Optitrack coordinates of the maze's (0, 0) corner
#   rotation_deg       : angle of the maze's +x axis in the Optitrack frame
#                        (counter-clockwise positive, degrees)
# The pose from Optitrack is converted into the maze frame before your code
# sees it, so self.origin is (0, 0) and everything else (world_to_grid,
# goal_list, ...) works exactly like in simulation.
def wrap_deg(angle:float) -> float:
    '''Wrap an angle in degrees into [-180, 180).'''
    return (angle + 180.0) % 360.0 - 180.0

def optitrack_to_maze(x:float, y:float, heading_deg:float, frame:dict) -> tuple:
    '''Optitrack-frame pose (x, y, heading in degrees) -> maze-frame pose.'''
    dx, dy = x - frame["origin_x"], y - frame["origin_y"]
    r = math.radians(frame["rotation_deg"])
    c, s = math.cos(r), math.sin(r)
    return (c * dx + s * dy, -s * dx + c * dy,
            wrap_deg(heading_deg - frame["rotation_deg"] - frame.get("heading_offset_deg", 0.0))) # offset: the Motive rigid body's "forward" vs the robot's front

def maze_to_optitrack(mx:float, my:float, heading_deg:float, frame:dict) -> tuple:
    '''Inverse of optitrack_to_maze (handy for working out where to place the robot).'''
    r = math.radians(frame["rotation_deg"])
    c, s = math.cos(r), math.sin(r)
    return (frame["origin_x"] + c * mx - s * my, frame["origin_y"] + s * mx + c * my,
            wrap_deg(heading_deg + frame["rotation_deg"] + frame.get("heading_offset_deg", 0.0)))

def robot_frame(config:dict, robot_number:int) -> dict:
    '''The frame for one robot: the shared origin/rotation plus that robot's heading offset (per-robot value from the config if listed).'''
    frame = dict(config["frame"])
    frame["heading_offset_deg"] = config["heading_offsets"].get(robot_number, frame["heading_offset_deg"])
    return frame

def load_irl_config(run:str) -> dict:
    '''Build the real-maze profile for run "test1", "test2" or "full".'''
    if run not in IRL_RUNS:
        raise ValueError(f"--run must be one of {IRL_RUNS}, got {run!r}")
    with open(os.path.join(_PACKAGE_DIR, "ca2_irl_layout.json")) as f:
        layout = json.load(f)
    parser = configparser.ConfigParser()
    config_path = os.path.join(_PACKAGE_DIR, "optitrack_variables.config")
    if not parser.read(config_path):
        raise FileNotFoundError(f"Could not read {config_path}")
    frame = {
        "origin_x": parser.getfloat("frame", "origin_x"),
        "origin_y": parser.getfloat("frame", "origin_y"),
        "rotation_deg": parser.getfloat("frame", "rotation_deg"),
    }
    # Rigid bodies are created in Motive with the robot facing the maze's +y direction, so the rigid body's zero heading IS maze +y
    # (maze heading 90 deg): heading_maze = reported + 90 = reported - rotation - offset  =>  offset = -(rotation + 90). "auto" means that.
    raw_offset = parser.get("frame", "heading_offset_deg", fallback="auto").strip().lower()
    frame["heading_offset_deg"] = wrap_deg(-(frame["rotation_deg"] + 90.0)) if raw_offset == "auto" else float(raw_offset)
    heading_offsets = {int(k): float(v) for k, v in parser.items("heading_offsets")} if parser.has_section("heading_offsets") else {}
    return {
        "map_file": "ca2_irl_map.npy",
        "origin": (0.0, 0.0), # pose is converted into the maze frame, so the grid corner is (0, 0)
        "resolution": float(layout["resolution"]),
        "goal_list": [tuple(g) for g in layout["runs"][run]["goals"]],
        "start": tuple(layout["runs"][run]["start"]),
        "frame": frame,
        "robot_number": parser.getint("robot", "number"),
        "heading_offsets": heading_offsets, # per-robot override of frame["heading_offset_deg"]
        "max_translate_velocity": 0.4, # Please keep this in place
    }


class WaypointNode(Node):
    '''Node to calculate path and move robot towards given goal_coordinates, using pose info from either gazebo odometer or optitrack'''
    def __init__(self, map_array:np.array, goal_list:list, is_simulation:bool=True, origin:tuple=(0.0, 0.0), resolution:float=occupancy_grid_resolution,
                 frame:dict=None, robot_number:int=3, expected_start:tuple=None, calibrate:bool=False):
        super().__init__('waypoint')
        self.get_logger().info("Starting WaypointNode")

        self.is_simulation = is_simulation

        # Subscribe to the dynamic_pose topic from Gazebo that publishes ground-truth pose data
        if self.is_simulation:
            self.subscription = self.create_subscription(Odometry, 'odom', self.odometer_callback, 2)
        else:
            qos_profile = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)

            self.map_sub = self.create_subscription(
                PoseStamped,
                f'/vrpn_mocap/bingda_{robot_number:03d}/pose',
                self.optitrack_callback,
                qos_profile
                )

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # Publish to cmd_vel node
        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

        self.goal_list = goal_list
        self.map_array = map_array
        self.origin = origin # World (x, y) coordinate of the grid's [0, 0] corner. Use with grid_to_world()/world_to_grid()
        self.resolution = resolution # Metres per grid cell (0.2). Use with grid_to_world()/world_to_grid()

        self.frame = frame # Optitrack->maze frame (real robot only), see optitrack_to_maze()
        self.expected_start = expected_start # Where this run says the robot should be placed (maze frame), real robot only
        self.calibrate = calibrate # Real robot: only print poses, never publish cmd_vel
        self.raw_pose = None # Latest pose exactly as Optitrack reported it (real robot only)
        self._start_checked = False
        self._calibrate_ticks = 0

        self.pose = None
        self.path = [] # Set this to your planned route (a list of grid-index tuples, in travel order) once you've computed it -- it'll automatically show up in the terminal map print
        self._last_printed_path = None
        #Nhien's Var
        self.goal_index = 0
        self.tree = None
        self.current_node_index = 0
        #Nhien PID
        self.last_delta_x = 0
        self.last_delta_y = 0
    

    def _print_calibration(self):
        '''--calibrate: twice a second, print the raw Optitrack pose next to the maze-frame pose and grid cell.'''
        self._calibrate_ticks += 1
        if self._calibrate_ticks % 10:
            return
        ox, oy, oh = self.raw_pose
        mx, my, mh = self.pose
        i, j = world_to_grid(mx, my, self.origin, self.resolution)
        self.get_logger().info(f"optitrack ({ox:+.3f}, {oy:+.3f}, {oh:+7.1f} deg)  ->  maze ({mx:+.3f}, {my:+.3f}, {mh:+7.1f} deg)  cell [{i}, {j}]")

    def _check_start_once(self):
        '''Once, on the first pose: warn if the robot isn't near this run's start point (usually a placement or config mistake).'''
        if self._start_checked or self.expected_start is None:
            return
        self._start_checked = True
        dist = math.hypot(self.pose[0] - self.expected_start[0], self.pose[1] - self.expected_start[1])
        msg = f"Robot is at maze ({self.pose[0]:.2f}, {self.pose[1]:.2f}); this run starts at {self.expected_start} -- {dist:.2f} m away"
        if dist > 0.4:
            self.get_logger().warn(msg + ". Check where the robot is placed; if it is on the start point, ask a TA.")
        else:
            self.get_logger().info(msg + " (ok)")

    def print_map(self):
        '''Prints the occupancy grid to the terminal: walls, your current position ('S'), all goal points ('W'/'G'),
        and your planned route (self.path) if you've set one ('*'). Safe to call anytime pose is known; does nothing
        useful before then. Called automatically from timer_callback() whenever self.path changes.'''
        if self.pose is None:
            return
        shape = self.map_array.shape
        clip = lambda cell: (int(np.clip(cell[0], 0, shape[0]-1)), int(np.clip(cell[1], 0, shape[1]-1)))
        current_cell = clip(world_to_grid(self.pose[0], self.pose[1], self.origin, self.resolution))
        goal_cells = [clip(world_to_grid(gx, gy, self.origin, self.resolution)) for gx, gy in self.goal_list]
        grid = Grid(self.map_array, starting_position=current_cell, goal_position=goal_cells[-1])
        grid.draw_grid_map(
            waypoints=goal_cells,
            path=self.path,
            rrt_tree=self.tree,
            origin=self.origin,
            resolution=self.resolution,
            save_path="./map_print.png",
        )

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
        if self.frame is not None:
            x, y, heading = optitrack_to_maze(x, y, heading, self.frame) # Your code works in the maze frame, like in simulation
        self.pose = np.array((x,y,heading))
        return self.pose

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
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.publisher_.publish(twist_msg)

    def set_waypoints(self, waypoints:list):
        '''Set new waypoints when a goal has been reached'''
        self.goal_reached = False
        self.waypoints = waypoints
        self.current_waypoint_idx = 0

    # Preprocess obstacles by giving every free cell next to a 99 a value of 75.
    def preprocess_map(self) -> np.ndarray:
        #Full copy
        original_map = self.map_array.copy()
        processed_map = original_map.copy()

        for obstacle_x, obstacle_y in np.argwhere(original_map == 99):
            for offset_x in (-1, 0, 1):
                for offset_y in (-1, 0, 1):
                    neighbour_x = obstacle_x + offset_x
                    neighbour_y = obstacle_y + offset_y
                    #if Within map, else index -1 will be rotated to last grid
                    if (0 <= neighbour_x < original_map.shape[0]
                            and 0 <= neighbour_y < original_map.shape[1]
                            and original_map[neighbour_x, neighbour_y] == 0):
                        processed_map[neighbour_x, neighbour_y] = 75

        return processed_map

    def timer_callback(self):
        """Controller loop. Insert path planning and PID control logic here"""
        if self.pose is None:
            return # Does not run if no pose received from Odom or Optitrack
        now = time.time()
        if now - getattr(self, "_last_pose_log", 0.0) >= 1.0: # at most once a second, so it does not bury the map / warnings below
            self._last_pose_log = now
            self.get_logger().debug(f"Pose: {self.pose}")

        if self.calibrate:
            self._print_calibration()
            return # Calibration mode: look, don't drive

        self._check_start_once()

        if self.path != self._last_printed_path: # Prints once immediately (map + start + goals), then again each time self.path changes
            self._last_printed_path = list(self.path)

        ###### INSERT CODE HERE ######
        if not self.tree:
            processed_map = self.preprocess_map()
            print("Calculating waypoints...")
            self.tree = Tree()
            self.tree.add_node(TreeNode(
                pos=(self.pose[0], self.pose[1]),
                cost=0,
                parent_id=-1,
                id=0,
                fixed_cost=0,
            ))
            #1000 new points
            while len(self.tree.nodes) < 2000:
                print("Progress:", str(len(self.tree.nodes)) + "/2000")
                #Random inside map expressed in world's coordination
                rand_x = np.random.uniform(
                    self.origin[0],
                    self.origin[0] + processed_map.shape[0] * self.resolution,
                )
                rand_y = np.random.uniform(
                    self.origin[1],
                    self.origin[1] + processed_map.shape[1] * self.resolution,
                )

                # #Try rand_x,rand_y as grid coordinate -> Less uncertainty
                # rand_i = np.random.randint(0, processed_map.shape[0])
                # rand_j = np.random.randint(0, processed_map.shape[1])
                # #Convert to world coordinate
                # rand_x, rand_y = grid_to_world(rand_i, rand_j,self.origin, self.resolution)

                nearest_node = self.tree.NearestNode(rand_x, rand_y)

                #can only be 0.2 away (rectangle to reduce computing compared to circle)from the node
                new_node_pos = (
                    nearest_node.pos[0] + np.clip(rand_x - nearest_node.pos[0], -0.2, 0.2),
                    nearest_node.pos[1] + np.clip(rand_y - nearest_node.pos[1], -0.2, 0.2),
                )

                #Check if obstacle in between
                if self.tree.line_intersects_obstacle(
                    nearest_node.pos,
                    new_node_pos,
                    processed_map,
                    self.origin,
                    self.resolution,
                ):
                    continue
                #Additional Cost for being near obstacle
                new_node_grid = world_to_grid(new_node_pos[0], new_node_pos[1], self.origin, self.resolution)
                additional_cost = processed_map[new_node_grid[0], new_node_grid[1]]

                new_node_cost = nearest_node.cost + self.tree.dist(
                    nearest_node.pos[0], nearest_node.pos[1],
                    new_node_pos[0], new_node_pos[1],
                ) + additional_cost

                new_node = TreeNode(
                    pos=new_node_pos,
                    cost=new_node_cost,
                    fixed_cost=additional_cost,
                    parent_id=nearest_node.id,
                )
                self.tree.add_node(new_node)
                #Rewire to cheapest parent, then rewire nearby nodes to this if it makes them cheaper
                self.tree.rewire(
                    new_node,
                    processed_map,
                    self.origin,
                    self.resolution,
                    r=0.2,
                )

        if not self.path:
            if self.goal_index >= len(self.goal_list):
                print("DONE ALL GOAL")
                self.move_2D(0,0,0)
                return
            print("GOING TO GOAL NO.", self.goal_index+1) 
            
            goal_node = self.tree.NearestNode(self.goal_list[self.goal_index][0], self.goal_list[self.goal_index][1])
            start_node = None
            if self.goal_index == 0:
                #Root node
                start_node = self.tree.nodes[0]
            else:
                start_node = self.tree.NearestNode(self.goal_list[self.goal_index-1][0], self.goal_list[self.goal_index-1][1])
            self.path = self.tree.path_to_goal(start_node, goal_node)
            print("Tree Size:", len(self.tree.nodes))
            print("Route length: ", len(self.path))
            self.print_map()
        else:
            

            distance_to_node = self.tree.dist(
                self.path[self.current_node_index].pos[0],
                self.path[self.current_node_index].pos[1],
                self.pose[0],
                self.pose[1],
            )
            if distance_to_node < 0.1:
                #End of path
                if self.current_node_index >= len(self.path) - 1:
                    print("END OF THIS PATH")
                    self.path = []
                    self.current_node_index = 0
                    self.goal_index += 1
                    self.move_2D(0,0,0)
                    return
                self.current_node_index += 1
                print(
                    "Going to X,Y: ",
                    self.path[self.current_node_index].pos[0],
                    self.path[self.current_node_index].pos[1],
                )

            #PD
            delta_x = self.path[self.current_node_index].pos[0] - self.pose[0]
            delta_y = self.path[self.current_node_index].pos[1] - self.pose[1]
            #PD logic
            Kp = 0.8
            Kd = 0.05
            #set dt = 0.2 for now
            dt = 0.2

            delta_x = Kp*delta_x + Kd*(delta_x-self.last_delta_x)/dt
            delta_y = Kp*delta_y + Kd*(delta_y-self.last_delta_y)/dt

            self.last_delta_x = delta_x
            self.last_delta_y = delta_y
            
            distance = np.hypot(delta_x, delta_y)

            if distance > 0:
                direction_x = delta_x / distance
                direction_y = delta_y / distance
                
                self.move_2D(
                    direction_x * max_translate_velocity/2,
                    direction_y * max_translate_velocity/2,
                    0,
                )
        ###### INSERT CODE HERE ######


@dataclass
class TreeNode:
    pos: tuple[float, float]
    cost: float
    fixed_cost:float
    parent_id: int

    id: int = -1

    # Structure of the k-d tree
    kd_parent_id: int = -1
    kd_smaller: int = -1
    kd_larger: int = -1


@dataclass
class Tree:
    nodes: list[TreeNode]
    root: int

    def __init__(self):
        self.nodes = []
        self.root = -1

    def add_node(self, node:TreeNode):
        # Sequential IDs prevent collisions and allow direct list lookup
        node.id = len(self.nodes)

        # First node becomes the root
        if self.root == -1:
            self.root = node.id
            self.nodes.append(node)
            return

        parent = self.get_node(self.root)
        split = 0  # 0 means x, 1 means y

        while True:
            if node.pos[split] < parent.pos[split]:
                child_id = parent.kd_smaller
            else:
                child_id = parent.kd_larger

            if child_id == -1:
                break

            parent = self.get_node(child_id)
            split = 1 - split

        if node.pos[split] < parent.pos[split]:
            parent.kd_smaller = node.id
        else:
            parent.kd_larger = node.id

        node.kd_parent_id = parent.id
        self.nodes.append(node)

    def get_node(self, id:int) -> TreeNode:
        for node in self.nodes:
            if node.id == id:
                return node
        return None

    def getParentNode(self, id:int) -> TreeNode:
        node = self.get_node(id)
        if node is not None:
            return self.get_node(node.parent_id)
        return None

    def dist(self, x1, y1, x2, y2) -> float:
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    #Preprocess the map_array obstacle to "bubble" up first (grids next to a 99 get set to 75)
    def line_intersects_obstacle(self, pos1:tuple, pos2:tuple, map_array:np.ndarray,
                                 origin:tuple, resolution:float) -> bool:
        '''Return True if the world-coordinate segment from pos1 to pos2 crosses a grid cell containing 99.'''
        start = np.asarray(pos1, dtype=float)
        end = np.asarray(pos2, dtype=float)
        line_length = self.dist(start[0], start[1], end[0], end[1])
        sample_count = max(2, int(np.ceil(line_length / (resolution / 2))) + 1)

        for scanner_pos in np.linspace(start, end, sample_count):
            grid_x, grid_y = world_to_grid(
                scanner_pos[0], scanner_pos[1], origin, resolution
            )
            if (grid_x < 0 or grid_x >= map_array.shape[0]
                    or grid_y < 0 or grid_y >= map_array.shape[1]
                    or map_array[grid_x, grid_y] >= 99):
                return True

        return False

    # Use spatial index / k-d tree later
    def NearestNode(self, x:float, y:float) -> TreeNode:
        nearest_node = None
        min_distance = float('inf')
        for node in self.nodes:
            distance = self.dist(x, y, node.pos[0], node.pos[1])
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        return nearest_node

    def _update_descendant_costs(self, parent_node:TreeNode):
        '''Recalculate all descendant costs after a node is rewired.'''
        for child in self.nodes:
            if child.parent_id == parent_node.id:
                child.cost = parent_node.cost + self.dist(
                    parent_node.pos[0], parent_node.pos[1],
                    child.pos[0], child.pos[1],
                ) + child.fixed_cost
                self._update_descendant_costs(child)

    # RRT* optimizer
    def rewire(self, new_node:TreeNode, map_array:np.ndarray, origin:tuple,
               resolution:float, r:float=0.2):
        nearby_nodes = []
        for node in self.nodes:
            if node.id == new_node.id:
                continue

            distance = self.dist(
                new_node.pos[0], new_node.pos[1], node.pos[0], node.pos[1]
            )
            if distance <= r:
                nearby_nodes.append((node, distance))

        # Replace the nearest parent with the cheapest collision-free parent.
        for node, distance in nearby_nodes:
            candidate_cost = node.cost + distance
            if (candidate_cost < new_node.cost
                    and not self.line_intersects_obstacle(
                        node.pos, new_node.pos, map_array, origin, resolution
                    )):
                new_node.parent_id = node.id
                new_node.cost = candidate_cost

        # Route nearby nodes through the new node when that lowers their cost.
        for node, distance in nearby_nodes:
            candidate_cost = new_node.cost + distance
            if (candidate_cost < node.cost
                    and not self.line_intersects_obstacle(
                        new_node.pos, node.pos, map_array, origin, resolution
                    )):
                node.parent_id = new_node.id
                node.cost = candidate_cost
                self._update_descendant_costs(node)

    def path_to_goal(self, start_node:TreeNode, goal_node:TreeNode) -> list[TreeNode]:
        # Find the start to root path first, then goal to root, then find Lowest Common Ancestor of those 2
        start_to_root = []
        current_node = start_node
        start_to_root_ancestors_id = []
        lowest_common = 0
        goal_to_lowest_common = []

        #Start to root
        while current_node is not None:
            start_to_root_ancestors_id.append(current_node.id)
            start_to_root.append(current_node)
            current_node = self.getParentNode(current_node.id)

        current_node = goal_node
        #Goal_to_common, check for common ancestor on the way to stop
        while current_node is not None:
            goal_to_lowest_common.append(current_node)
            if current_node.id in start_to_root_ancestors_id:
                lowest_common = current_node.id
                lowest_common_index = start_to_root_ancestors_id.index(lowest_common)
                break
            current_node = self.getParentNode(current_node.id)

        start_to_lowest_common = start_to_root[:lowest_common_index]

        start_to_goal = start_to_lowest_common + goal_to_lowest_common[::-1]
        return start_to_goal
        

        
        #Return ancestor chain for LCA
        

class Grid():
    '''
    Grid class to use with occupancy grid. Contains the following functions:
        check_grid_validity : Uses flood fill to check if there's a valid path from start to goal position
        draw_grid_map : Creates a colour image of the grid, as well as waypoints and full solution path if given. Can show it and/or save it to a .png file
        print_grid_map : Prints an ASCII-art version of the same map (grid, waypoints, path) to the terminal
        animate_path : Prints the ASCII-art map frame by frame in the terminal, animating the robot moving along a given path

    __init__ input Args:
        grid_array : 2D numpy array representing the occupancy grid
        starting_position : tuple of starting indices within the numpy array
        goal_position : tuple of goal indices within the numpy array. Works with negative indices as well
    '''
    def __init__(self, grid_array:np.array=np.array([]), starting_position:tuple=(0,0), goal_position:tuple=(-1,-1)):
        self.grid = grid_array
        self.shape = self.grid.shape
        self.starting_position = starting_position

        # If goal position given with negative index, need to convert to +ve
        if goal_position[0] < 0:
            goal_x = self.shape[0] + goal_position[0]
        else:
            goal_x = goal_position[0]
        if goal_position[1] < 0:
            goal_y = self.shape[1] + goal_position[1]
        else:
            goal_y = goal_position[1]

        self.goal_position = (goal_x, goal_y)

    def check_grid_validity(self):
        '''Use flood fill to check if there's a viable path between start and goal positions'''
        grid = self.grid.copy()
        flood_stack = [(self.goal_position[0], self.goal_position[1])]
        while flood_stack:
            tile = flood_stack[0]
            del flood_stack[0]
            try:
                next_tile = (tile[0]+1, tile[1])
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
            try:
                next_tile = (tile[0]-1, tile[1])
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
            try:
                next_tile = (tile[0], tile[1]+1)
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
            try:
                next_tile = (tile[0], tile[1]-1)
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
        if grid[self.starting_position] == 1: # Means the flood is able to reach starting position from the ending position
            return True
        else:
            return False

    def _colour_grid(self, waypoints:list=(), path:list=(), obstacle_threshold:float=50) -> np.array:
        '''Builds the (H, W, 3) colour image array shared by draw_grid_map. Maze walls in blue, empty space in white, path taken in green and waypoints in red'''
        image_grid = np.ones((self.grid.shape[0],self.grid.shape[1],3), dtype=np.uint8)
        image_grid[self.grid <= obstacle_threshold] = (255,255,255)
        image_grid[self.grid > obstacle_threshold] = (0,0,255)

        for x, y in path:
            image_grid[x][y] = (0,255,0)

        for point in waypoints:
            image_grid[point] = (255,0,0)

        return image_grid

    def draw_grid_map(self, waypoints:list=(), path:list=(), rrt_tree:Tree=None,
                      origin:tuple=(0.0, 0.0), resolution:float=occupancy_grid_resolution,
                      obstacle_threshold:float=50, save_path:str=None, show:bool=True):
        '''Creates an image of the maze and path taken. Maze walls in blue, empty space in white, path taken in green and waypoints in red

        Args:
            waypoints : list (or other iterable) of tuple coordinates representing all the grid indices for the waypoints. Will be represented in red, takes precedence over path
            path : RRT TreeNodes forming the solution path. Will be represented in green
            rrt_tree : Tree whose nodes and connections will be drawn
            origin : World coordinate of grid cell [0, 0]'s lower corner
            resolution : Size of each grid cell in world units
            obstacle_threshold : Optional float to indicate threshhold for whether a grid is considered occupied. Not important for ca2
            save_path : Optional file path (e.g. "path_result.png") to save the image to, in addition to/instead of showing it
            show : Whether to pop up the image in a viewer (default True). Set to False if you only want to save it
        '''
        # RRT path nodes are drawn below in continuous world coordinates.
        image_grid = self._colour_grid(waypoints, (), obstacle_threshold)

        image_grid = np.flip(image_grid, axis=1)[::-1]
        img = Image.fromarray(image_grid, 'RGB')

        # Resize image
        base_width = 500
        wpercent = (base_width / float(img.size[0]))
        hsize = int((float(img.size[1]) * float(wpercent)))
        img = img.resize((base_width, hsize), getattr(Image, "Resampling", Image).NEAREST)

        # Draw the RRT as connected dots, with solution-path nodes highlighted.
        if rrt_tree is not None:
            from PIL import ImageDraw

            draw = ImageDraw.Draw(img)
            x_scale = img.width / self.grid.shape[1]
            y_scale = img.height / self.grid.shape[0]

            def image_position(node):
                grid_x = (node.pos[0] - origin[0]) / resolution - 0.5
                grid_y = (node.pos[1] - origin[1]) / resolution - 0.5
                return (
                    (self.grid.shape[1] - grid_y - 0.5) * x_scale,
                    (self.grid.shape[0] - grid_x - 0.5) * y_scale,
                )

            for node in rrt_tree.nodes:
                parent = rrt_tree.getParentNode(node.id)
                if parent is not None:
                    draw.line(
                        (image_position(parent), image_position(node)),
                        fill=(30, 30, 30),
                        width=1,
                    )

            node_radius = 2
            for node in rrt_tree.nodes:
                x, y = image_position(node)
                draw.ellipse(
                    (x - node_radius, y - node_radius,
                     x + node_radius, y + node_radius),
                    fill=(255, 165, 0),
                )

            path_radius = 5
            for node in path:
                x, y = image_position(node)
                draw.ellipse(
                    (x - path_radius, y - path_radius,
                     x + path_radius, y + path_radius),
                    fill=(0, 255, 0),
                    outline=(0, 100, 0),
                    width=1,
                )

        if save_path:
            img.save(save_path)
            print(f"Saved map image to {save_path}")

        if show:
            img.show()

    def print_grid_map(self, waypoints:list=(), path:list=(), obstacle_threshold:float=50):
        '''Prints an ASCII-art version of the maze to the terminal: '#' wall, '.' free, '*' solution path, 'W' waypoint, 'S' start, 'G' goal

        Args:
            waypoints : list (or other iterable) of tuple coordinates representing all the grid indices for the waypoints
            path : list (or other iterable) of tuple coordinates representing all the grid indices forming the solution path
            obstacle_threshold : Optional float to indicate threshhold for whether a grid is considered occupied. Not important for ca2
        '''
        chars = np.where(self.grid > obstacle_threshold, '#', '.').astype('<U1')

        for x, y in path:
            chars[x, y] = '*'
        for point in waypoints:
            chars[point] = 'W'
        chars[self.starting_position] = 'S'
        chars[self.goal_position] = 'G'

        display = np.flip(chars, axis=1)[::-1]
        print('\n'.join(''.join(row) for row in display))

    def animate_path(self, path:list, waypoints:list=(), obstacle_threshold:float=50, delay:float=0.2):
        '''Animates the robot moving along path, one cell at a time, by reprinting the ASCII-art map to the terminal.
        'R' marks the robot's current cell, '*' cells it has already passed through.

        Args:
            path : list of tuple grid indices forming the solution path, in travel order
            waypoints : list (or other iterable) of tuple coordinates representing all the grid indices for the waypoints
            obstacle_threshold : Optional float to indicate threshhold for whether a grid is considered occupied. Not important for ca2
            delay : Seconds to pause between animation frames
        '''
        path = list(path)
        base_chars = np.where(self.grid > obstacle_threshold, '#', '.').astype('<U1')
        for point in waypoints:
            base_chars[point] = 'W'
        base_chars[self.goal_position] = 'G'

        for step in range(len(path)):
            frame = base_chars.copy()
            for x, y in path[:step]:
                frame[x, y] = '*'
            frame[path[step]] = 'R'

            display = np.flip(frame, axis=1)[::-1]
            print("\033c", end="")  # Clear terminal between frames
            print('\n'.join(''.join(row) for row in display))
            time.sleep(delay)


def main(args=None):
    global max_translate_velocity

    arg_parser = argparse.ArgumentParser(description="RB2301 CA2 path planning")
    arg_parser.add_argument(
        '--run', choices=IRL_RUNS, default=None,
        help="Run on the real maze: test1, test2 or full (starts/goals in README). Omit to run in Gazebo simulation."
    )
    arg_parser.add_argument(
        '--robot', type=int, default=None,
        help="Number of your Bingda robot (3 -> /vrpn_mocap/bingda_003/pose). Default: your ROS_DOMAIN_ID (set to the robot number on the robots), else [robot] number in optitrack_variables.config."
    )
    arg_parser.add_argument(
        '--calibrate', action='store_true',
        help="Real robot: only print Optitrack and maze-frame poses (twice a second) and never publish cmd_vel. Use it to set origin/rotation."
    )
    cli_args, ros_args = arg_parser.parse_known_args(args=args)
    run = cli_args.run or ("test1" if cli_args.calibrate else None)
    is_simulation = run is None # Remember: pass --run test1|test2|full to ca2.sh when testing on the real lab setup

    print("Starting path planning")
    rclpy.init(args=ros_args)

    robot_number = 3
    frame = expected_start = None
    if is_simulation:
        config = sim_config
    else:
        config = load_irl_config(run)
        frame, expected_start = config["frame"], config["start"]
        domain = os.environ.get("ROS_DOMAIN_ID", "")
        if cli_args.robot is not None:
            robot_number = cli_args.robot
        elif domain.isdigit() and 1 <= int(domain) <= 99: # on the Bingda robots ROS_DOMAIN_ID is the robot number
            robot_number = int(domain)
        else:
            robot_number = config["robot_number"]
        frame = robot_frame(config, robot_number)
        expected_start = config["start"]
        print(f"Running on the real maze, run={run}, robot=bingda_{robot_number:03d}, resolution={config['resolution']}")
        print(f"  maze origin in Optitrack frame = ({frame['origin_x']}, {frame['origin_y']}), rotation = {frame['rotation_deg']} deg, robot heading offset = {frame['heading_offset_deg']} deg")
        print(f"  start={expected_start}, goals={config['goal_list']}")
        if cli_args.calibrate:
            print("  CALIBRATE MODE: printing poses only, the robot will not be commanded")

    max_translate_velocity = config["max_translate_velocity"]

    map_array = np.load(os.path.join(_PACKAGE_DIR, config["map_file"]), allow_pickle=True)
    waypoint = WaypointNode(map_array, config["goal_list"], is_simulation, config["origin"], config["resolution"],
                            frame=frame, robot_number=robot_number, expected_start=expected_start, calibrate=cli_args.calibrate)

    # Start spinning the waypoint node and only stop once SystemExit error is raised within the node callback
    try:
        rclpy.spin(waypoint)
    except SystemExit:
        print("Shutting down")

    waypoint.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
