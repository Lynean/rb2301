"""Measure the time and path length taken to leave the obstacle maze."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .obstacle_generator import height, size_div


# The global maze top is its final grid row; use one finish for every scenario.
# Put the finish line just beyond that row.
MAZE_TOP_X = (height - 1) * 2.0 / size_div
FINISH_OFFSET = 0.1
FINISH_LINE_X = MAZE_TOP_X + FINISH_OFFSET


class AlgoGraderNode(Node):
    """Track a robot run from the first odometry message to the finish line."""

    def __init__(self):
        super().__init__("algo_grader")

        self.latest_position = None
        self.previous_position = None
        self.start_time = None
        self.elapsed_time = 0.0
        self.path_length = 0.0
        self.finished = False

        self.odom_subscription = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10,
        )
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f"Waiting for odometry; finish line is x = {FINISH_LINE_X:.3f} m "
            f"(maze top x = {MAZE_TOP_X:.3f} m)"
        )

    def odom_callback(self, msg: Odometry) -> None:
        """Save the newest planar position for the timer to process."""
        position = msg.pose.pose.position
        self.latest_position = (position.x, position.y)

    def timer_callback(self) -> None:
        """Update distance/time and report the result after crossing the line."""
        if self.finished or self.latest_position is None:
            return

        now = self.get_clock().now()
        current_position = self.latest_position

        if self.start_time is None:
            self.start_time = now
            self.previous_position = current_position
        else:
            delta_x = current_position[0] - self.previous_position[0]
            delta_y = current_position[1] - self.previous_position[1]
            self.path_length += math.hypot(delta_x, delta_y)
            self.previous_position = current_position
            self.elapsed_time = (now - self.start_time).nanoseconds / 1e9

        if current_position[0] >= FINISH_LINE_X:
            self.finished = True
            self.get_logger().info(
                "Finish line reached! "
                f"Path length: {self.path_length:.3f} m, "
                f"time: {self.elapsed_time:.3f} s"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlgoGraderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

