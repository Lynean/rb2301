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
        self.get_logger().info("Starting Obstacle Avoidance")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message
        self.current_angle = 0
        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

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
        self.last_scan = np.array(msg.ranges)[::20] # Slices the 721 scan array to return only 36 scans. Feel free to edit

    def timer_callback(self):
        """Controller loop"""
        if self.last_scan is None:
            return # Does not run if the laser message is not received.
        
        ######################## MODIFY CODE HERE ########################
        self.get_logger().debug(str(self.last_scan))
        #shift the scan array clockwise
        angle_per_element = 10 * np.pi / 180  # radians per scan element
        shifts = int(self.current_angle / angle_per_element) % len(self.last_scan)
        self.last_scan = np.roll(self.last_scan, shift=shifts)

        # Only get the first 9 and last 9 scans, as the rest are behind the robot

        front_scans = np.concatenate((self.last_scan[:9], self.last_scan[-9:]))
        front_scans[front_scans == np.inf] = 20

        

        # frontscan = first and last 3
        front_scan = np.concatenate((self.last_scan[:3], self.last_scan[-3:]))
        front_blocked = np.any(front_scan < 1) # Check if any of the front scans are blocked
        front_inf = (max(front_scans) == 20) # Check if any of the front scans are inf
        if front_blocked: # If the front is blocked or inf, turn left or right depending on which side is more open
            north_east_scan = self.last_scan[:9] # Get the scans to the left of the robot
            north_west_scan = self.last_scan[-9:] # Get the scans to the right of the robot
            if np.min(north_east_scan) > np.min(north_west_scan):
                delta_x = 0.0
                delta_y = max_translate_velocity
            elif np.min(north_east_scan) < np.min(north_west_scan):
                delta_x = 0.0
                delta_y = -max_translate_velocity
            else:
                #Kinda locate where the map is left or right
                left_average = np.mean(self.last_scan[:18])
                right_average = np.mean(self.last_scan[-18:])
                delta_x = 0.0
                delta_y = max_translate_velocity if left_average > right_average else -max_translate_velocity
        else:
            delta_x = max_translate_velocity
            delta_y = 0.0
        self.get_logger().debug(f"Delta x: {delta_x:.2f}, Delta y: {delta_y:.2f}")
        #turn the delta vector to handle the current frame 
        delta_x_new = delta_x * np.cos(self.current_angle) - delta_y * np.sin(self.current_angle)
        delta_y_new = delta_x * np.sin(self.current_angle) + delta_y * np.cos(self.current_angle)
        delta_x, delta_y = delta_x_new, delta_y_new
        deg_to_rad_10 = 10 * np.pi / 180
        self.move_2D(delta_x, delta_y, max_turn_velocity) 
        self.current_angle += max_turn_velocity
        if self.current_angle >= np.pi * 2:
            self.current_angle -= np.pi * 2
        ######################## MODIFY CODE HERE ########################


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()