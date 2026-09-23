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
        self.going = False # A flag to indicate if the robot is moving forward or not. If it is moving forward, then it should stop if an obstacle is detected. If it is not moving forward, then it should start moving forward if there are no obstacles detected.
        self.scanner = 0
        self.scanner_dir = -2
        self.timer = self.create_timer(0.1, self.timer_callback)  # Runs at 20Hz. Can be changed.
        self.current_bound = set(["left",])
        self.last_dir = 0
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
        self.last_scan = np.array(msg.ranges)[::2] # Slices the 721 scan array to return only 360 scans. Feel free to edit

    def timer_callback(self):
        """Controller loop"""
        if self.last_scan is None:
            return # Does not run if the laser message is not received.
        
        ######################## MODIFY CODE HERE ########################
        #self.get_logger().debug(str(self.last_scan))


        #use self.scanner to go through every group of 6 scans and check if any of them are blocked. If not, then the robot should move forward.
        delta_x = 0
        delta_y = 0

    
        #How close are we to the boundary
        self.last_scan = np.clip(self.last_scan, 0.05, 20) # Clip the scanned values to avoid inf and 0
        
        # print("Scans:", self.last_scan)
        #Is at left boundary
        if all(self.last_scan[0:100] >= 19) and all(self.last_scan[-10:] >= 19):
            self.current_bound.add("left")
            if "right" in self.current_bound:
                self.current_bound.remove("right")
            
        #Is at right boundary
        if all(self.last_scan[-100:] >= 19) and all(self.last_scan[:10] >= 19):
            self.current_bound.add("right")
            if "left" in self.current_bound:
                self.current_bound.remove("left")


        if all(self.last_scan[90:270]>=19):
            self.current_bound.add("back")
        elif "back" in self.current_bound:
            self.current_bound.remove("back")

        # if all(np.concatenate((self.last_scan[-90:],self.last_scan[:90]))>=19):
        #     self.current_bound.add("front")
        # elif "front" in self.current_bound:
        #     self.current_bound.remove("front")

        if not "front" in self.current_bound:
            self.current_bound.add("back")
        
        #Second Layer
        if "left" in self.current_bound:
            self.last_scan[30:150] = 0.3
            print("LEFT BOUNDARY")
        if "right" in self.current_bound:
            self.last_scan[-150:-30]  = 0.3
            print("RIGHT BOUNDARY")
        if "back" in self.current_bound:
            print("BACK BOUNDARY")
            self.last_scan[95:265] = 0.3
        if "front" in self.current_bound:
            print("FRONT BOUNDARY")
            self.last_scan[-85:] = 0.3
            self.last_scan[:85] = 0.3

        last_scan_copy = self.last_scan.copy()
        window_size = 80
        window_start = 0
        while window_start <= len(self.last_scan) - window_size:
            window_end = window_start + window_size
            if np.all(self.last_scan[window_start:window_end] == 20):
                while window_end < len(self.last_scan) and self.last_scan[window_end] == 20:
                    window_end += 1
                self.last_scan[window_start:window_end] = 3
            window_start += 1
        
        self.last_scan = np.clip(self.last_scan, 0, 3)
        
        
        
        self.last_scan = np.concatenate((self.last_scan, self.last_scan))
        window_size = 10
        window_start = 0
        while window_start <= len(self.last_scan) - window_size:
            window_end = window_start + window_size
            if np.all(self.last_scan[window_start:window_end] >= 0.7):
                while (window_end < len(self.last_scan)) and (self.last_scan[window_end] >= 0.7):
                    window_end += 1
                self.last_scan[window_start:window_end] = 1
                window_start = window_end
                window_size = 10
                continue
            else:
                self.last_scan[window_start:window_end] = 0
                window_start += window_size
            
        window_size = 1
        window_start = 0
        dir  = 0
        best_gap_score = -1e9
        best_gap_center = 0
        while window_start <= len(self.last_scan) - window_size:
            window_end = window_start + window_size
            if np.all(self.last_scan[window_start:window_end] == 1):
                while window_end < len(self.last_scan) and self.last_scan[window_end] == 1:
                    window_end += 1
                    window_size += 1
                gap_width = window_end - window_start
                if gap_width >= 20:
                    circular_scan = np.concatenate((last_scan_copy, last_scan_copy))
                    gap_distance_values = circular_scan[window_start:window_end]
                    mean_distance = float(np.mean(gap_distance_values)) if gap_distance_values.size > 0 else 0.0
                    if mean_distance >= 2:
                        gap_score = (gap_width + mean_distance) / 2
                        if gap_score > best_gap_score:
                            best_gap_score = gap_score
                            best_gap_center = window_start + (window_end - window_start) / 2
                window_start += window_size
                window_size = 1
                continue
            window_start += window_size

        if best_gap_score > -1e9:
            dir = best_gap_center
        else:
            dir = 0

        #print("Filtered:", self.last_scan[:360])
        #print("Window Size:", window_size)

        dir = dir*np.pi/180
        if dir >= 360:
            dir -= 360
        dir = (7*dir + self.last_dir) / 8
        self.last_dir = dir
        print("Dir:", dir*180/np.pi)
        delta_x = max_translate_velocity*np.cos(dir)
        delta_y = max_translate_velocity*np.sin(dir)
        print("X,Y Original:", delta_x, delta_y)
        #Emergency
        #if sth in front
        #print("Original scan:", last_scan_copy)
        if np.any(last_scan_copy[-150:]<0.3) and np.any(last_scan_copy[:150]<0.3):
            if np.min(last_scan_copy[-180:-30]) - np.min(last_scan_copy[30:180]) > 0.2:
                self.current_bound.add("right")
                if "left" in self.current_bound:
                    self.current_bound.remove("left")
                #go left
                delta_x = delta_x
                delta_y = (delta_y if delta_y > 0 else 0)
            elif np.min(last_scan_copy[-180:-60]) - np.min(last_scan_copy[60:180]) < -0.2:
                self.current_bound.add("left")
                if "right" in self.current_bound:
                    self.current_bound.remove("right")
                #go right
                delta_x = delta_x
                delta_y = (delta_y if delta_y < 0 else 0)
            else:
                delta_x = delta_x
                delta_y = 0
        elif np.any(last_scan_copy[-180:-30]<0.3):
            self.current_bound.add("right")
            if "left" in self.current_bound:
                self.current_bound.remove("left")
            #go left
            delta_x = delta_x
            delta_y = (delta_y if delta_y > 0 else 0)
        #if sth on left
        elif np.any(last_scan_copy[30:180]<0.3):
            self.current_bound.add("left")
            if "right" in self.current_bound:
                self.current_bound.remove("right")
            #go right
            delta_x = delta_x
            delta_y = (delta_y if delta_y < 0 else 0)
        if np.any(last_scan_copy[-80:] < 0.3) and np.any(last_scan_copy[:80] < 0.3):
            self.current_bound.add("front")
            if "back" in self.current_bound:
                self.current_bound.remove("back")
            delta_x = (delta_x if delta_x < 0 else -delta_x)
            delta_y = delta_y
        elif "front" in self.current_bound:
            self.current_bound.remove("front")
            self.current_bound.add("back")
            delta_x = delta_x
            delta_y = delta_y

        print(self.current_bound)
        print("X,Y New:", delta_x, delta_y)
        self.move_2D(delta_x,delta_y)
        self.last_scan = None
        

        ######################## MODIFY CODE HERE ########################


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()