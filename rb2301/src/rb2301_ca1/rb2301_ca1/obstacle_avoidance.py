import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan



np.set_printoptions(
    2, suppress=True
)  # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)

max_translate_velocity = 0.4 # Can be implemented as parameter
max_turn_velocity = max_translate_velocity * 2 # Can be implemented as parameter
set_logger_level("obstacle_avoidance", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  





class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.declare_parameter("algo", "nhien")
        self.get_logger().info("Starting Obstacle Avoidance")
        self.algo = None
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message
        self.going = False # A flag to indicate if the robot is moving forward or not. If it is moving forward, then it should stop if an obstacle is detected. If it is not moving forward, then it should start moving forward if there are no obstacles detected.
        self.scanner = 0
        self.scanner_dir = -2
        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.
        self.last_invalid_algo = None
        self.tempo_blind_left = 0
        self.tempo_blind_right = 0
        #MAX's var
        # Save the latest gap decision as a JPEG. The same file is overwritten
        # so that debugging does not create an unlimited number of images.
        self.last_debug_plot_time = 0.0
        self.debug_plot_interval = 1.0
        self.debug_clusters = []
        self.debug_candidate_gaps = []

        robot_width = 0.23
        safety_margin = 0.05
        self.required_gap = robot_width + 2 * safety_margin
        self.stop_distance = 0.6 # og:0.9
        self.look_ahead = 0.75

        self.avoiding = False
        self.clear_count = 0
        self.gap_heading = None

        self.gap_side = -1   # -1 = start with right, +1 = start with left
        self.side = 0        # emergency sidestep direction

        self.side_lock_count = 0
        self.missing_count = 0
        self.last_command = (0.0, 0.0) 

        #MAX's var
        self.last_angles = None
    
        # Store each gap decision beside the can layout for this environment.

        # Automatically evaluate every run from the first odometry sample.

        self.robot_width = 0.23
        self.robot_length = 0.23
        self.safety_margin = 0.05
        self.required_gap = self.robot_width + 2 * self.safety_margin
        self.stop_distance = 0.6 # og:0.9
        self.look_ahead = 0.75

        self.avoiding = False
        self.clear_count = 0
        self.gap_heading = None

        self.gap_side = -1   # -1 = start with right, +1 = start with left
        self.side = 0        # emergency sidestep direction

        self.side_lock_count = 0
        self.missing_count = 0
        self.last_command = (0.0, 0.0)

    def move_2D(self, x: float = 0.0, y: float = 0.0, turn: float = 0.0):
        """Publishes a twist command to move in 2D space. +ve x is forwards, +ve y is left, and +ve turn is anticlockwise"""
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.pub_cmd_vel.publish(twist_msg)

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        algo = self.get_parameter("algo").value
        
        if algo == "nhien":
            self.last_scan = np.array(msg.ranges)[::2]
        elif algo == "max":
            self.last_scan = np.array(msg.ranges)[::4]  
        else:
            self.last_scan = np.array(msg.ranges)[::2]
    def nhien(self):
        """Controller loop"""
        if self.last_scan is None:
            return # Does not run if the laser message is not received.
        
        ######################## MODIFY CODE HERE ########################
        #self.get_logger().debug(str(self.last_scan))
        def boundary_distance(angle_degrees):
            """Map boundary angular width to distance, quartering every 30 degrees."""
            return 20.0 * 4.0 ** (-(angle_degrees - 90.0) / 30.0)

        #use self.scanner to go through every group of 6 scans and check if any of them are blocked. If not, then the robot should move forward.
        delta_x = 0
        delta_y = 0



        self.last_scan = np.clip(self.last_scan, 0, 20) # Clip the scanned values to avoid inf and 0

        last_scan_copy = self.last_scan.copy()
            #Split into 2 parts to prevent over detecting edge (>180)
            #left first
        window_size = 90
        window_start = 0
        while window_start <= len(self.last_scan)/2 - window_size:
            window_end = window_start + window_size
            if np.all(self.last_scan[window_start:window_end] == 20):
                while window_end < len(self.last_scan)/2 and self.last_scan[window_end] == 20:
                    window_end += 1
                angle_degrees = window_end - window_start
                self.last_scan[window_start:window_end] = boundary_distance(
                    angle_degrees)
                window_start = window_end
                #print("Zeroed left")
                continue
            window_start += 1
        
        #Right
        while window_start <= len(self.last_scan) - window_size:
            window_end = window_start + window_size
            if np.all(self.last_scan[window_start:window_end] == 20):
                while window_end < len(self.last_scan) and self.last_scan[window_end] == 20:
                    window_end += 1
                angle_degrees = window_end - window_start
                self.last_scan[window_start:window_end] = boundary_distance(
                    angle_degrees)
                window_start = window_end
                #print("Zeroed right")
                continue
            window_start += 1
        #Second Layer
        #Mask the back
        self.last_scan[100:260:5] = 0.1 

        #Blind left/ right side
        if self.tempo_blind_left > 0:
            self.tempo_blind_left -= 1
            self.last_scan[0:180:5] = 0.1
            self.tempo_blind_right = 0
            print("Blind left")
        
        if self.tempo_blind_right > 0:
            self.tempo_blind_right -= 1
            self.last_scan[-180::5]  = 0.1 
            self.tempo_blind_right = 0
            print("Blind right")
        
        self.last_scan = np.clip(np.concatenate((self.last_scan, self.last_scan)), 0, 20)
        window_size = 20
        window_start = 0
        dir  = 0
        best_gap_score = 0
        best_gap_center = 0
        while window_start <= len(self.last_scan) - window_size:
            window_end = window_start + window_size
            if np.all(self.last_scan[window_start:window_end] >= 0.5):
                while (window_end < len(self.last_scan)) and (self.last_scan[window_end] >= 0.5):
                    window_end += 1
                gap_width = window_end - window_start
                gap_distance_values = self.last_scan[window_start:window_end]
                mean_distance = float(np.mean(gap_distance_values)) if gap_distance_values.size > 0 else 0.0
                self.last_scan[window_start:window_end] = mean_distance 
                if gap_width >= 20:
                    if mean_distance >= 2:
                        gap_score = (gap_width + mean_distance) / 2
                        if gap_score > best_gap_score:
                            best_gap_score = gap_score
                            best_gap_center = window_start + (window_end - window_start) / 2
                window_start = window_end
                window_size = 20
                continue
            else:
                self.last_scan[window_start:window_end] = 0
                window_start = window_end
            
        if best_gap_score > 0:
            dir = best_gap_center
        else:
            dir = 0


        dir = dir*np.pi/180
        if dir >= 360:
            dir -= 360

        delta_x = max_translate_velocity*np.cos(dir)
        delta_y = max_translate_velocity*np.sin(dir)
        #Emergency

        coke_near_right = np.any(last_scan_copy[-180:-60]<0.2) and False
        bound_near_right = np.all(last_scan_copy[-200:] >= 20) and np.all(last_scan_copy[:20] >= 20)
        coke_near_left = np.any(last_scan_copy[60:180]<0.2) and False
        bound_near_left = np.all(last_scan_copy[:200] >= 20) and np.all(last_scan_copy[20:] >= 20)

        coke_near_front = (np.any(last_scan_copy[-30:] < 0.2) or np.any(last_scan_copy[:30] < 0.2)) and False

        min_dis_right = np.min(last_scan_copy[-180:-60]) if not coke_near_right else 0.2
        min_dis_left = np.min(last_scan_copy[60:180]) if not  coke_near_left else 0.2

        # #if sth on right
        # if bound_near_right:
        #     self.current_bound.add("right")
        #     self.tempo_blind_right += 20
            
        # #if sth on left
        # elif bound_near_left:
        #     self.current_bound.add("left")
        #     self.tempo_blind_left += 20

        #go left
        if np.any(last_scan_copy[90:181]<0.25):
            self.tempo_blind_left += 10
            self.tempo_blind_right = 0
            print("LEFTDOWN")
        elif np.any(last_scan_copy[181:271]<0.25):
            self.tempo_blind_right += 10
            self.tempo_blind_left = 0
            print("RIGHTDOWN")

        if (np.any(last_scan_copy[-180:]<0.25) and self.tempo_blind_left == 0):
            print("Emergency left")
            self.tempo_blind_right += 10
            delta_x = delta_x
            delta_y = (delta_y if delta_y > 0 else 0.2)
        #Go right
        elif (np.any(last_scan_copy[:180]<0.25) and self.tempo_blind_right == 0):
            print("Emergency right")
            self.tempo_blind_left += 10
            delta_x = delta_x
            delta_y = (delta_y if delta_y < 0 else -0.2)

        
    
        #GO BACK
        if np.any(last_scan_copy[-90:] < 0.15) or np.any(last_scan_copy[:90] < 0.15):
            print("Emergency BACK")
            delta_x = (delta_x if delta_x < 0 else 0)
            delta_y = delta_y

        self.move_2D(delta_x,delta_y)
        self.last_scan = None

    #MAXX
    """
    LiDAR index 0       approximately -180  robot front
    LiDAR index 45      approximately  -90  robot left
    LiDAR index 90                      0   robot back
    LiDAR index 135     approximately  +90  robot right
    LiDAR index 179     approximately +178  robot front
    
    """
    def max(self):

        if self.last_scan is None:
            return

        scan = np.asarray(self.last_scan, dtype=float).reshape(-1)
        if len(scan) < 4:
            self.move_2D()
            return

        points, indices, angles, ranges = self.get_front_scan(scan)
        targets = []

        # Check the space directly in front of the robot.
        front_clearance = 0.12
        blocked = np.any(
            (points[:, 0] > 0)
            & (points[:, 0] < self.stop_distance)
            & (np.abs(points[:, 1]) < front_clearance)
        )

        if not blocked:
            if self.avoiding:
                self.clear_count += 1

            if self.avoiding and self.clear_count < 8:
                # Wait for a few clear readings before going straight again.
                vx, vy = self.last_command
                target = np.array([vx, vy])
            else:
                self.avoiding = False
                self.clear_count = 0
                self.gap_heading = None
                
                self.side = 0
                self.side_lock_count = 0
                self.missing_count = 0
                self.last_command = (0.0, 0.0)
                vx, vy = 0.3, 0.0
                target = np.array([self.look_ahead, 0.0])

        else:
            self.avoiding = True
            self.clear_count = 0
            targets = self.find_gap_targets(points, indices)
            target = self.choose_gap(targets)

            if target is not None:

                # Move towards the middle of the selected gap.
                direction = target / np.linalg.norm(target)
                vx = 0.18 * direction[0]
                vy = 0.18 * direction[1]
                self.gap_heading = np.arctan2(target[1], target[0])
                self.missing_count = 0

                if self.gap_heading > np.deg2rad(5):
                    self.side = 1
                elif self.gap_heading < np.deg2rad(-5):
                    self.side = -1
            else:
                vx, vy = self.get_sidestep_command(points, angles, ranges)
                target = np.array([vx, vy])

            self.last_command = (vx, vy)

        
        print(f"Moving to x={vx}, y={vy}")
        self.move_2D(x=vx, y=vy, turn=0.0)

    def get_front_scan(self, scan):
        # Keep the original scan order: right side -> front -> left side.
        quarter = len(scan) // 4
        front = np.concatenate((scan[3 * quarter:], scan[:quarter + 1]))
        angles = np.linspace(-np.pi / 2, np.pi / 2, len(front))

        ranges = np.nan_to_num(front, nan=0.0, posinf=self.look_ahead, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.look_ahead)

        valid = np.isfinite(front) & (front >= 0.05) & (front < self.look_ahead)
        indices = np.flatnonzero(valid)
        x = front[valid] * np.cos(angles[valid])
        y = front[valid] * np.sin(angles[valid])
        points = np.column_stack((x, y))
        return points, indices, angles, ranges
    
    def required_clearance_for_direction(self, direction):
        """Project the fixed-heading chassis across a travel direction."""
        return (
            self.robot_width * abs(direction[0])
            + self.robot_length * abs(direction[1])
            + 2.0 * self.safety_margin
        )



    def find_gap_targets(self, points, indices):
        # Put neighbouring readings from the same can into one group.
        cans = []
        for i in range(len(points)):
            if i == 0:
                cans.append([points[i]])
                continue

            distance = np.linalg.norm(points[i] - points[i - 1])
            if indices[i] != indices[i - 1] + 1 or distance > 0.12:
                cans.append([points[i]])
            else:
                cans[-1].append(points[i])

        self.debug_clusters = [np.asarray(can) for can in cans]
        self.debug_candidate_gaps = []

        targets = []
        for i in range(len(cans) - 1):
            right_edge = cans[i][-1]
            left_edge = cans[i + 1][0]
            gap = left_edge - right_edge
            midpoint = right_edge + 0.5 * gap
            distance = np.linalg.norm(midpoint)

            if midpoint[0] <= 0 or distance == 0:
                continue

            heading = np.arctan2(midpoint[1], midpoint[0])

            # Don't consider gaps that are mostly beside the robot
            # this was the change that helped it to get out of the loop
            if abs(heading) > np.deg2rad(85): # originally 60
                continue

            direction = midpoint / distance
            # The robot keeps its +x heading, so the opening it can pass through
            # is the lateral (+/-y) surface-to-surface separation. The diagonal
            # vector to the midpoint is not the robot heading.
            lateral_width = abs(gap[1])
            if lateral_width < self.required_gap:
                continue

            # Separately check the swept corridor along the diagonal translation.
            corridor_required = self.required_clearance_for_direction(direction)
            along = points @ direction
            across = np.abs(
                direction[0] * points[:, 1]
                - direction[1] * points[:, 0]
            )
            blocked = np.any(
                (along > 0.05)
                & (along < distance - 0.05)
                & (across < corridor_required / 2)
            )
            if not blocked:
                heading = abs(np.arctan2(midpoint[1], midpoint[0]))
                targets.append((heading, midpoint))
                self.debug_candidate_gaps.append(
                    (right_edge.copy(), left_edge.copy(), midpoint.copy())
                )

        return targets
    def choose_gap(self, targets):
        if not targets:
            return None

        # First check if there is a gap almost straight ahead
        for _, target in targets:
            if abs(target[1]) < 0.15:
                self.side = 0
                print(f"Straight ahead target: {target}")
                return target

        # If no straight gap, choose the opposite side
        wanted_side = self.gap_side

        possible = []

        for _, target in targets:
            if target[1] * wanted_side > 0:
                possible.append(target)

        if possible:
            # Pick the gap closest to straight ahead
            best_target = possible[0]

            for target in possible:
                if abs(np.arctan2(target[1], target[0])) < abs(
                    np.arctan2(best_target[1], best_target[0])
                ):
                    best_target = target

            #self.gap_side *= -1
            print(f"Best Target: {best_target}")
            return best_target

        return None

    def get_sidestep_command(self, points, angles, ranges):
        right = np.median(ranges[angles < np.deg2rad(-30)])
        left = np.median(ranges[angles > np.deg2rad(30)])

        if self.side_lock_count > 0:
            self.side_lock_count -= 1

        if self.side == 0:
            if left > right:
                self.side = 1
            else:
                self.side = -1
        elif (
            self.side_lock_count == 0
            and self.side == 1
            and right > left + 0.20
        ):
            self.side = -1
        elif (
            self.side_lock_count == 0
            and self.side == -1
            and left > right + 0.20
        ):
            self.side = 1

        # Check the swept corridor in the current sideways direction. This is
        # independent of the generated scenario: any obstacle close to the
        # side of the chassis causes the robot to reverse before contact.
        half_robot_clearance = self.required_gap / 2
        side_stop_distance = 0.30
        distance_in_side_direction = points[:, 1] * self.side
        side_blocked = np.any(
            (distance_in_side_direction > 0.0)
            & (distance_in_side_direction < side_stop_distance)
            & (np.abs(points[:, 0]) < half_robot_clearance)
        )

        if side_blocked:
            opposite_distance = points[:, 1] * -self.side
            opposite_blocked = np.any(
                (opposite_distance > 0.0)
                & (opposite_distance < side_stop_distance)
                & (np.abs(points[:, 0]) < half_robot_clearance)
            )

            if not opposite_blocked:
                self.side *= -1
                # Hold the escape direction long enough to move away from the
                # detected side wall instead of immediately switching back.
                self.side_lock_count = 12
            else:
                return -0.08, 0.0

        if self.missing_count >= 4:
            self.gap_heading = None

        # Reverse slightly if a can is very close in front.
        front_points = points[
            (points[:, 0] > 0)
            & (np.abs(points[:, 1]) < self.required_gap / 2)
        ]
        vx = 0.0
        if len(front_points) > 0 and np.min(front_points[:, 0]) < 0.20:
            vx = -0.08

        vy = 0.25 * self.side
        return vx, vy
    ##MAXX
    def timer_callback(self):
        algo = self.get_parameter("algo").value

        if algo == "nhien":
            self.algo = "nhien"
            self.last_invalid_algo = None
            self.nhien()
        elif algo == "max":
            self.algo = "max"
            self.last_invalid_algo = None
            self.max()
        else:
            # Stop rather than continuing with a stale velocity command when
            # the selected controller does not exist.
            self.move_2D()
            if algo != self.last_invalid_algo:
                self.get_logger().error(
                    f"Unknown obstacle-avoidance algorithm {algo!r}; "
                    "expected 'nhien' or 'max'"
                )
                self.last_invalid_algo = algo


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()