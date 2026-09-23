# 1. IMPORT
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy import Parameter as Param
from  rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter as ParamType
from rcl_interfaces.msg import ParameterValue as ParamVal
from rcl_interfaces.msg import ParameterType as ParamEnums
# NODE CLASS
class Parameters(Node):
    def __init__(self):
        super().__init__('parameters')
        
        self.timer = self.create_timer(1, self.timer_callback)
        self.get_client = self.create_client(GetParameters, "/turtlesim/get_parameters")
        self.set_client = self.create_client(SetParameters, "/turtlesim/set_parameters")
        self.get_params_future = None
        self.set_params_future = None
        self.r = None
        self.g = None
        self.ParamSet = False
    def set_parameters_callback(self, param_list):
        for i in param_list:
            param = i.name
            new = i.value
            print(f'Setting "{param}": {new}')
        return SetParametersResult(successful=True)

    def timer_callback(self):
        if self.get_params_future is None:
            self.get_params(["background_r", "background_g"])
        else:
            if self.get_params_future.done():
                news = self.get_params_future.result().values
                self.r = news[0].integer_value
                self.g = news[1].integer_value
                print(f'Current Red({self.r:3d}) and Green({self.g:3d})')

                if self.set_params_future is None:
                    self.set_params(["background_r", "background_g"], [ParamEnums.PARAMETER_INTEGER, ParamEnums.PARAMETER_INTEGER], [(self.r + 50)%255, (self.g - 40)%255])
                else:
                    if self.set_params_future.done() and not self.ParamSet:
                        print(f'Setting Red({(self.r + 50)%255:3d}) and Green({(self.g - 40)%255:3d})')
                        self.ParamSet = True
                        #Prompt the GET client 1 more time
                        self.get_params_future = None


    def get_params(self, name_list):
        request = GetParameters.Request()
        request.names = name_list
        self.get_params_future = self.get_client.call_async(request)

    def set_params(self, name_list, type_list, value_list):
        request = SetParameters.Request()
        for i in range(len(name_list)):
            param = ParamType()
            param.name = name_list[i]
            param.value = ParamVal()
            param.value.type = type_list[i]
            if type_list[i] == ParamEnums.PARAMETER_INTEGER:
                param.value.integer_value = value_list[i]
            #... Expand this when needed, currently only INTEGERs
            request.parameters.append(param)
        self.set_params_future = self.set_client.call_async(request)

# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = Parameters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()