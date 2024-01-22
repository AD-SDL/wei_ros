import rclpy
from rclpy.node import Node

from wei_services.srv import WeiActions

from std_msgs.msg import String
from sensor_msgs.msg import Image  

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default

import cv2  # OpenCV library
from cv_bridge import CvBridge 
import json

import os
import time

class weiExecNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.camera_sub = None
        self.image_path = ""
        self.image_name = ""
        self.image_rotation = 0
        self.state = {}
                                                

    def send_wei_command(self,ros_node,action_handle, action_vars={}):
        weiActionClient = self.create_client(WeiActions,ros_node+'/action_handler')
        while not weiActionClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(ros_node + ' service not available, waiting again...')
        weiReq = WeiActions.Request()
        weiReq.action_handle=str(action_handle)
        weiReq.vars=json.dumps(action_vars)
        #print(weiReq.vars)
        future = weiActionClient.call_async(weiReq)
        # self.get_logger().info('retrying')
        # self.get_logger().info(str(rclpy.utilities.ok()))
        # self.get_logger().info("chachanged")
        # self.get_logger().info("chachanged")
        # while not future.done():    
        #     self.get_logger().info(str(rclpy.utilities.ok()))
        #     self.get_logger().info(str(weiActionClient.wait_for_service(timeout_sec=1.0)))
        #     self.get_logger().info(str(future.done()))
        
        rclpy.spin_until_future_complete(self, future=future)  
           
        #res = weiActionClient.call(weiReq)
        res = future.result()
        print(res.action_response)
        print(res.action_msg)
        print(res.action_log)
        return res.action_response, res.action_msg, res.action_log
    
    def get_state(self, node_name):
        self.state_sub = self.create_subscription(String, node_name+"/state", lambda data: self.save_state_callback(data, node_name), qos_profile_system_default)
        s = 1
        self.state[node_name] = ""
        while self.state[node_name] == "" and s < 5:
            rclpy.spin_once(self, timeout_sec=0.5)
            s += 1
        return self.state[node_name]
    
    def get_about(self, node_name):
        weiDescClient = self.create_client(String,node_name+'/get_description')
        while not weiDescClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(node_name + ' service not available, waiting again...')
        future = weiDescClient.call_async()
        rclpy.spin_until_future_complete(self, future)  
        res = future.result()
        print(res)
        return res

    def capture_image(self, node_name, image_name = "camera_image.png", path = "~/",rotation=0):
        self.image_path = os.path.join(path, image_name)
        image_stream = node_name + "/video_frames"
        self.image_rotation = rotation
        self.get_logger().info('Image from: ' + image_stream)
        self.camera_sub = self.create_subscription(Image, image_stream, self.save_image_callback, qos_profile_sensor_data)
        self.camera_sub

        while not os.path.exists(self.image_path):
            rclpy.spin_once(self,timeout_sec=10)

        return self.image_path

    def save_image_callback(self, data):
        br = CvBridge()
        current_frame = br.imgmsg_to_cv2(data)
        if current_frame.any(): 
            self.get_logger().info("Received an image!")
            current_frame = cv2.rotate(current_frame, self.image_rotation)
            cv2.imwrite(self.image_path, current_frame)
            self.get_logger().info("Image is saved to " + str(self.image_path))
    def save_state_callback(self, data, node_name):
       self.state[node_name] = data
       return data

    def get_log(self, node_name):
        pass


