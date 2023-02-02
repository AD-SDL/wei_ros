import rclpy
from rclpy.node import Node

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

    async def capture_image(self, image_stream, image_name, path):
        self.image_path = os.path.join(path,image_name)
        print('Naaa')
        self.create_subscription(Image, image_stream, self.save_image_callback, 1 )#, qos_profile_sensor_data)
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


#rclpy.init()
#test = weiExecNode()