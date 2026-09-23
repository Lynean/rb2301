# 1. IMPORT
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy import Parameter as Param

# NODE CLASS
class Parameters(Node):
    def __init__(self):
        super().__init__('parameters')
    
        self.declare_parameter('values.bool_v', False)
        self.declare_parameter('values.int_v', 0)
        self.declare_parameter('values.dbl_v', 0.0)
        self.declare_parameter('values.str_v', '')
        self.declare_parameter('bool_arr_v', [False])
        self.declare_parameter('int_arr_v', [0])
        self.declare_parameter('dbl_arr_v', [0.0])
        self.declare_parameter('str_arr_v', [''])
        
        self.timer = self.create_timer(1, self.timer_callback)
        self.add_on_set_parameters_callback(self.set_parameters_callback)

    def set_parameters_callback(self, param_list):
        for i in param_list:
            param = i.name
            new = i.value
            print(f'Setting "{param}": {new}')
        return SetParametersResult(successful=True)

    def timer_callback(self):
        self.get_logger().info(f'{self.get_parameter("values.str_v").value}')

# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = Parameters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()