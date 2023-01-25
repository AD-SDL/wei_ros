import rclpy
from rclpy.node import Node

from wei_services.srv import WeiActions
from wei_services.srv import WeiDescription
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
                                                

    def send_wei_command(self,ros_node,action_handle, action_vars={}):
        weiActionClient = self.create_client(WeiActions,ros_node+'/action_handler')
        while not weiActionClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(ros_node + ' service not available, waiting again...')
        weiReq = WeiActions.Request()
        weiReq.action_handle=str(action_handle)
        weiReq.vars=str(action_vars)

        future = weiActionClient.call_async(weiReq)
        rclpy.spin_until_future_complete(self, future)  
        res = future.result()
        print(res.action_response)
        print(res.action_msg)
        return res.action_response, res.action_msg
    
    def get_description(self, node_name):
        weiDescClient = self.create_client(WeiDescription,node_name+'/get_description')
        while not weiDescClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(node_name + ' service not available, waiting again...')
        future = weiDescClient.call_async()
        rclpy.spin_until_future_complete(self, future)  
        res = future.result()
        print(res.description)
        return res.description

<<<<<<< HEAD
    def capture_image(self, node_name = "CameraPublisher1", image_name = "camera_image.png", path = "~/"):
=======
    def capture_image(self, image_name = "camera_image.png", path = "~/", node_name = "CameraPublisher1"):
>>>>>>> b0b038f2ac5ca515341121166e396a19efd48f13
        self.image_path = os.path.join(path, image_name)
        self.create_subscription(Image, "/std_ns/" + node_name + "/video_frames", self.save_image_callback, qos_profile_sensor_data)
        rclpy.spin_once(self,timeout_sec=10)

    def save_image_callback(self, data):
        br = CvBridge()
        current_frame = br.imgmsg_to_cv2(data)
        if current_frame.any(): 
            self.get_logger().info("Received an image!")
            cv2.imwrite(self.image_path, current_frame)
            self.get_logger().info("Image is saved to " + str(self.image_path))

        # Display image
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

    def get_log(self, node_name):
        pass


