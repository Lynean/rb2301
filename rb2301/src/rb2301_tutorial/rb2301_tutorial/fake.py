# 1. IMPORT
import rclpy
from rclpy.node import Node
#Message Types
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from nav_msgs.srv import GetPlan

# NODE CLASS
class Controller(Node):
    def __init__(self):
        #super init with node's name
        super().__init__('controller')
        # 2. NODE PROPERTIES
        self.odom_msg = None
        # Declare a node param -> can be changed from outside the node
        self.speed_param = self.declare_parameter('speed', 2.0)
        # 3. NODE HANDLES
        #Timer
        self.timer = self.create_timer(0.5, self.timer_callback)
        #Sub - What to do when receiving data from the topic
        #10 is the Qos Profile ~ queue size
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_sub_callback, 10)
        #Pub
        self.point_pub = self.create_publisher(Point, '/goal', 10)
        #Service Server - Give sth when called
        self.get_plan_srv = self.create_service(GetPlan, '/get_plan', self.get_plan_callback) #When called the run the callback that returns {response}
        #Service Client - Request sth from a Service
        self.get_plan_cli = self.create_client(GetPlan, '/get_plan')

    # 4. NODE CALLBACKS
    def timer_callback(self):
        time_now = self.get_clock().now().seconds_nanoseconds()
        print(f'sec: {time_now[0]}, nanosec: {time_now[1]}')

    def odom_sub_callback(self, msg):
        self.odom_msg = msg

    # 5. HOW TO USE
    def odo_info(self):
        print(self.odom_msg.pose.pose.position.x)

    #Server -> {request} & {response} are structs
    def get_plan_callback(self, request, response):
        path = "PlaceHolder: A->C->B"
        response.plan = path
        return response

    #Publish - used in main to control when to put
    def pub_point(self):
        point_msg = Point()
        point_msg.x = 1.0
        point_msg.y = -1.5
        point_msg.z = 0.0
        self.point_pub.publish(point_msg)
    # Can only read params from ownself
    # Nodes from other parameters can be accessed, but require service calls
    def expose_speed_param(self):  
        speed = self.speed_param.get_parameter_value().double_value
    #Client
    def function_to_send_request(self, start, goal):    
        if self.get_plan_future is None:
            request = GetPlan.Request()
            request.start = start
            request.goal = goal
            request.tolerance = 0.0
            self.get_plan_future = self.get_plan_cli.call_async(request)

    def function_to_use_response(self):
        path = []
        if self.get_plan_future is not None:
            if self.get_plan_future.done():
                path = self.get_plan_future.result().plan
        return path
    
# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = Controller() # same name as the class above.
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()