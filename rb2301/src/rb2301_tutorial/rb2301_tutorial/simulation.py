# 1. IMPORT
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class Simulation(Node):
    def __init__(self):
        super().__init__('simulation')
        self.odom_msg = None
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', callback=self.sub_callback_odom, qos_profile=10)
        self.timer = self.create_timer(0.2, self.timer_callback)

    def sub_callback_odom(self, msg):
        self.odom_msg = msg
    def timer_callback(self):
        if self.odom_msg:
            print(f"Robot X: {self.odom_msg.pose.pose.position.x} Robot Y = {self.odom_msg.pose.pose.position.y}")


# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = Simulation() # same name as the class above.
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()