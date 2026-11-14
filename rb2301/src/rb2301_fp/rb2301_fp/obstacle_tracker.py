"""Print obstacle directions and nearest distances from a LaserScan."""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


DEFAULT_GROUP_DISTANCE = 0.10
DEFAULT_MAXIMUM_OBSTACLE_DISTANCE = 0.20


def wrap_degrees(angle):
    """Wrap degrees into [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


class ObstacleTracker(Node):
    """Group continuous LiDAR returns and print one bearing/range per group."""

    def __init__(self):
        super().__init__('obstacle_tracker')
        self.group_distance = float(self.declare_parameter(
            'group_distance', DEFAULT_GROUP_DISTANCE).value)
        self.maximum_obstacle_distance = float(self.declare_parameter(
            'maximum_obstacle_distance',
            DEFAULT_MAXIMUM_OBSTACLE_DISTANCE,
        ).value)
        # Direction is reported in the robot frame. The nanocar's scanner is
        # mounted backward, so its scan angle needs a 180-degree correction.
        self.lidar_yaw = math.radians(float(self.declare_parameter(
            'lidar_yaw_deg', 180.0).value))

        qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos,
        )
        self.get_logger().info(
            'Obstacle tracker started: adjacent-point threshold '
            f'{self.group_distance:.2f} m, reporting obstacles nearer than '
            f'{self.maximum_obstacle_distance:.2f} m'
        )

    @staticmethod
    def point_spacing(first_range, second_range, angle_difference):
        """Euclidean distance between two polar scan returns."""
        distance_squared = (
            first_range * first_range
            + second_range * second_range
            - 2.0 * first_range * second_range * math.cos(angle_difference)
        )
        return math.sqrt(max(0.0, distance_squared))

    def group_scan_indices(self, msg, ranges):
        """Return ordered lists of adjacent indices belonging to one object."""
        maximum_range = float(msg.range_max)
        valid = np.isfinite(ranges) & (ranges > 0.0)
        if math.isfinite(maximum_range):
            valid &= ranges <= maximum_range

        valid_indices = np.flatnonzero(valid)
        if valid_indices.size == 0:
            return []

        groups = [[int(valid_indices[0])]]
        for raw_index in valid_indices[1:]:
            index = int(raw_index)
            previous = groups[-1][-1]
            adjacent = index == previous + 1
            spacing = math.inf
            if adjacent:
                spacing = self.point_spacing(
                    float(ranges[previous]),
                    float(ranges[index]),
                    abs(float(msg.angle_increment)),
                )

            if adjacent and spacing < self.group_distance:
                groups[-1].append(index)
            else:
                groups.append([index])

        # A 360-degree scan is circular: index N-1 and index 0 are neighbors.
        full_circle = (
            len(ranges) * abs(float(msg.angle_increment))
            >= 2.0 * math.pi - 0.05
        )
        if (
            full_circle
            and len(groups) > 1
            and groups[0][0] == 0
            and groups[-1][-1] == len(ranges) - 1
        ):
            seam_spacing = self.point_spacing(
                float(ranges[groups[-1][-1]]),
                float(ranges[groups[0][0]]),
                abs(float(msg.angle_increment)),
            )
            if seam_spacing < self.group_distance:
                groups = [groups[-1] + groups[0]] + groups[1:-1]

        return groups

    def scan_callback(self, msg):
        """Print the middle-beam direction and minimum range of every object."""
        ranges = np.asarray(msg.ranges, dtype=float)
        groups = self.group_scan_indices(msg, ranges)

        obstacles = []
        for group in groups:
            middle_index = group[len(group) // 2]
            scan_angle = (
                float(msg.angle_min)
                + middle_index * float(msg.angle_increment)
            )
            robot_angle = scan_angle + self.lidar_yaw
            angle_degrees = wrap_degrees(math.degrees(robot_angle))
            nearest_distance = float(np.min(ranges[group]))
            if nearest_distance < self.maximum_obstacle_distance:
                obstacles.append((angle_degrees, nearest_distance))

        obstacles.sort(key=lambda obstacle: obstacle[0])
        if not obstacles:
            self.get_logger().info(
                'Obstacles: none nearer than '
                f'{self.maximum_obstacle_distance:.2f} m'
            )
            return

        text = '\n'.join(
            f'obstacle: angle={angle:+.1f} deg, nearest={distance:.2f} m'
            for angle, distance in obstacles
        )
        self.get_logger().info('Obstacles:\n' + text)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
