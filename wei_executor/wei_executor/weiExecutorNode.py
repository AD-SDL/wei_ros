import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import String

from wei_services.srv import WeiActions
from std_msgs.msg import String


class weiExecNode(Node):
    def __init__(self):
        super().__init__('weiExecNode')
                                                

    def send_wei_command(self,service,action_handle, action_vars={}):
        weiExecutor = self.create_client(WeiActions,service)
        while not weiExecutor.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(service + ' service not available, waiting again...')
        weiReq = WeiActions.Request()
        weiReq.action_handle=str(action_handle)
        weiReq.vars=str(action_vars)

        future = weiExecutor.call_async(weiReq)
        rclpy.spin_until_future_complete(self, future)  


