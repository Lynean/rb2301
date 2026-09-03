# 1. IMPORT
import rclpy
from rclpy.node import Node
#Message Types
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
'''
* Do not remove the previous codes. Now, create a **service client**.
  - The client will request to spawn a new turtle over the service `/spawn`.
  - Using the ROS2 CLI and a running `turtlesim` node, determine the service interface type for the service, and its data fields.
  - From the **timer callback**, send the service request **only once** throughout the entire run.
  - The request should spawn a new turtle at the coordinates $(1, 1)$ at any orientation. The name should be left blank using an empty string `''`.
  - Using the future object, print out the automatically assigned name of the new turtle in the **timer callback** after the response is received.
'''
# NODE CLASS
class Tutorial(Node):
    def __init__(self):
        super().__init__('tutorial')
        self.count = 0
        self.poseMsg = None
        self.velCmd = 1.0
        self.spawn_response_future = None

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.sub_pose = self.create_subscription(Pose,"/turtle1/pose", callback = self.sub_callback, qos_profile=10)
        self.pub_vel = self.create_publisher(Twist,"/turtle1/cmd_vel", qos_profile=10)
        self.client_spawn = self.create_client(Spawn, "/spawn")

    def sub_callback(self, msg):
        self.poseMsg = msg

    def pub_fw_vel(self, speed):
        msg = Twist()
        msg.linear.x = speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.pub_vel.publish(msg)

    def client_spawnUnamedTurtle(self, x, y, theta):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = ''
        self.spawn_response_future = self.client_spawn.call_async(request)

    def timer_callback(self):
        self.count += 1
        print(self.count)
        #Sub
        if self.poseMsg:
            x = self.poseMsg.x
            y = self.poseMsg.y
            print(f'({x}, {y})')
        #Pub
        self.pub_fw_vel(self.velCmd)
        self.velCmd *= -1
        #Spawn Client
        #async -> only proceed when response available
        if not self.spawn_response_future:
            self.client_spawnUnamedTurtle(1.0, 1.0, 0.0)
        else:
            if self.spawn_response_future.done():
                print(self.spawn_response_future.result().name)
# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = Tutorial() # same name as the class above.
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()