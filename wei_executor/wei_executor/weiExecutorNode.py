import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from wei_services.srv import WeiActions
from std_msgs.msg import String
from sensor_msgs.msg import Image  
from rclpy.qos import qos_profile_sensor_data

import cv2  # OpenCV library
from cv_bridge import CvBridge 

class weiExecNode(Node):
    def __init__(self):
        super().__init__('weiExecNode')
        self.image_path = ""
        self.image_name = ""
                                                

    def send_wei_command(self,service,action_handle, action_vars={}):
        weiExecutor = self.create_client(WeiActions,service)
        while not weiExecutor.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(service + ' service not available, waiting again...')
        weiReq = WeiActions.Request()
        weiReq.action_handle=str(action_handle)
        weiReq.vars=str(action_vars)

        future = weiExecutor.call_async(weiReq)
        rclpy.spin_until_future_complete(self, future)  
    
    def get_description(self, node_name):
        pass

    def capture_image(self, image_stream, image_name, path):
        self.image_name = image_name
        self.image_path = path
        rclpy.init(args=None)
        node = rclpy.create_node('minimal_subscriber')
        cameraSub = node.create_subscription(Image, "Camera_Publisher_Node/video_frames", self.save_image_callback, qos_profile_sensor_data)
        cameraSub # prevent unused variable warning

        rclpy.spin_once(node)
        node.destroy_node()
        rclpy.shutdown() 

    def save_image_callback(self, data):
        br = CvBridge()
        current_frame = br.imgmsg_to_cv2(data)
        if current_frame.any(): 
            cv2.imwrite(self.image_path + self.image_name, current_frame)

        # Display image
        # cv2.imshow("camera", current_frame)
        # cv2.waitKey(1)

    def get_log(self, node_name):
        pass


