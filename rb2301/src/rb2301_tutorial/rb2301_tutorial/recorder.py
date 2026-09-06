import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class Recorder(Node):
    def __init__(self):
        super().__init__('recorder')
        self.x = None
        self.y = None
        self.theta = None
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", callback=self.sub_pose_callback, qos_profile=10)
        self.f = open('data.txt', 'w')
    def timer_callback(self):
        pass

    def sub_pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.f.write(f'{self.x}\t{self.y}\t{self.theta}\n')
# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = Recorder() # same name as the class above.
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
