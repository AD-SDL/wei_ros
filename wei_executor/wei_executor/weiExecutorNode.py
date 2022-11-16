import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from wei_services.srv import WeiActions
from std_msgs.msg import String
from sensor_msgs.msg import Image  
from rclpy.qos import qos_profile_sensor_data

import cv2  # OpenCV library
from cv_bridge import CvBridge 

import os

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
        self.image_path = os.path.join(path,image_name)
#        node = rclpy.create_node('image_subscriber')
        print('Naaa')
        self.create_subscription(Image, image_stream, self.save_image_callback, qos_profile_sensor_data)
        rclpy.spin_once(self,timeout_sec=10)

    def save_image_callback(self, data):
        print('Savanah')
        br = CvBridge()
        print('badabin')
        current_frame = br.imgmsg_to_cv2(data)
        if current_frame.any(): 
            print('Badaaabaa')
            cv2.imwrite(self.image_path, current_frame)

        # Display image
        # cv2.imshow("camera", current_frame)
        # cv2.waitKey(1)

    def get_log(self, node_name):
        pass


