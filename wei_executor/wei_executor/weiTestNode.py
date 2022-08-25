#! /usr/bin/env python3
"""Peeler node"""

from typing import List, Tuple

import rclpy  # import Rospy
from rclpy.node import Node  # import Rospy Node
from wei_services.srv import WeiActions, WeiDescription
from std_msgs.msg import String


class testNode(Node):
    """
    """

    def __init__(self, NODE_NAME='testNode'):
        """
        The init function is neccesary for the peelerNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        """

        super().__init__(NODE_NAME)

        self.state = "READY" 

        self.description = {
            'name': NODE_NAME,
            'type':'',
            'actions':
            {
                'peel':'%d %d'
            }
            }

        timer_period = 1  # seconds
        self.stateTimer = self.create_timer(timer_period, self.stateCallback)   # Callback that publishes to peeler state

        self.actionSrv = self.create_service(WeiActions, NODE_NAME + "/action_handler", self.actionCallback)

        self.descriptionSrv = self.create_service(WeiDescription, NODE_NAME + "/description_handler", self.descriptionCallback)

    def descriptionCallback(self, request, response):
        """The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.

        Parameters:
        -----------
        request: str
            Request to the robot to deliver actions
        response: str
            The actions a robot can do, will be populated during execution

        Returns
        -------
        str
            The robot steps it can do
        """
        response.description_response = str(self.description)

        return response
        
        
    def actionCallback(self, request, response):
        """The actions the robot can perform, also performs them

        Parameters:
        -----------
        request: str
            Request to the robot to perform an action
        respone: bool
            If action is performed

        Returns
        -------
        None
        """

        self.state = "BUSY"
        self.stateCallback()

        print(request) 
        response.action_response = True

        self.state = "COMPLETED"


        return response

    def stateCallback(self):
        """The state of the robot, can be ready, completed, busy, error"""

        msg = String()

        msg.data = "State: %s" % self.state

        self.statePub.publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.data)

        self.state = "READY"


def main(args=None):  # noqa: D103

    rclpy.init(args=args)       # initialize Ros2 communication

    node = testNode()

    rclpy.spin(node)            # keep Ros2 communication open for action node

    rclpy.shutdown()            # kill Ros2 communication


if __name__ == "__main__":

    main()