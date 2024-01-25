from typing import Any, Dict, Tuple

from wei.config import Config
from wei.core.data_classes import Interface, Module, Step

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from wei_interfaces.action import WeiAction

import json

import os

class weiExecNode(Node):
    """Basic Interface for interacting with WEI modules using ROS2"""

    def __init__(self, node_name = 'weiExecNode'):
        super().__init__(node_name)

        self.goal = None
        self.feedback = None
        self.result = None
   
    def send_wei_goal(self, ros_node, wei_goal) -> None:
        """Send action goal"""
        
        self._action_client = ActionClient(self, WeiAction, ros_node + '/wei_action')
        while not self._action_client.wait_for_server():
            self.get_logger().info(ros_node + 'Action server not available, waiting ...')

        self.goal = json.dumps(wei_goal)
        
        weiGoal = WeiAction.Goal()
        weiGoal.wei_goal = self.goal
        print(weiGoal)
        self._send_goal_future = self._action_client.send_goal_async(weiGoal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future) 
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self.result["action_response"], self.result["action_log"]
    
    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.get_logger().info("Action started: " + self.goal)
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        self.result = future.result().result
        self.get_logger().info('Result: {0}'.format(self.result.robot_response))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg) -> None:
        self.feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback:' + self.feedback.robot_feedback)
    
    #-----------                                             
    
    def send_wei_command(self, ros_node, action_handle, action_vars={}):
        # weiActionClient = self.create_client(WeiAction,ros_node+'/action_handler')
        # while not weiActionClient.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info(ros_node + ' service not available, waiting again...')
        # weiReq = WeiAction.Request()
        # weiReq.action_handle=str(action_handle)
        # weiReq.vars=json.dumps(action_vars)
        # print(weiReq.vars)
        # future = weiActionClient.call_async(weiReq)
        # rclpy.spin_until_future_complete(self, future)  
        # res = future.result()
        # print(res.action_response)
        # print(res.action_msg)
        # print(res.action_log)
        # return res.action_response, res.action_msg, res.action_log

        pass
        
    
    def get_description(self, node_name):
        # weiDescClient = self.create_client(WeiDescription,node_name+'/get_description')
        # while not weiDescClient.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info(node_name + ' service not available, waiting again...')
        # future = weiDescClient.call_async()
        # rclpy.spin_until_future_complete(self, future)  
        # res = future.result()
        # print(res.description)
        # return res.description

        pass

    def get_log(self, node_name):
        pass

#----Interface
class ROS2Interface(Interface):

    @staticmethod
    def __init_rclpy(name="'weiExecNode')", args = None) -> Any:
        if not rclpy:
            raise ImportError("ROS2 environment not found")
        if not rclpy.utilities.ok():
            rclpy.init(args=args)
            print("Started RCLPY")
            wei_execution_node = weiExecNode(name)
        else:
            print("RCLPY OK ")
            wei_execution_node = weiExecNode(name)
        return wei_execution_node

    @staticmethod
    def config_validator(config: Dict[str, Any]) -> bool:
        """Validates the configuration for the interface

        Parameters
        ----------
        config : Dict
            The configuration for the module

        Returns
        -------
        bool
            Whether the configuration is valid or not
        """
        for key in ["ros_node_address"]:
            if key not in config:
                return False
        return True

    @staticmethod
    def send_action(step: Step, module: Module, **kwargs: Any) -> Tuple[str, str, str]:
        """Executes a single step from a workflow 

        Parameters
        ----------
        step : Step
            A single step from a workflow definition

        Returns
        -------
        action_response: StepStatus
            A status of the step (in theory provides async support with IDLE, RUNNING, but for now is just SUCCEEDED/FAILED)
        action_log: str
            A record of the execution of the step

        """
        wei_execution_node = ROS2Interface.__init_rclpy(
            Config.workcell_name + "_exec_node"
        )
        
        msg = {
            "node": module.config["ros_node_address"],
            "action_goal": {step.action:step.args}
        }
        print(msg)

        if kwargs.get("verbose", False):
            print("\n Callback message:")
            print(msg)
            # print()

        action_response = ""
        action_log = ""
        (action_response, action_log) = wei_execution_node.send_wei_goal(msg["node"], msg["action_goal"])
        
        if action_response and kwargs.get("verbose", False):
            print(action_response)
        
        rclpy.spin_once(wei_execution_node)
        wei_execution_node.destroy_node()
        rclpy.shutdown()
        
        return action_response, action_log

    @staticmethod
    def get_state(module: Module, **kwargs: Any) -> str:
        """Gets the state of the module and returns it."""
        wei_execution_node = ROS2Interface.__init_rclpy(
            Config.workcell_name + "_wei_exec_node"
        )
        state = wei_execution_node.get_state(module.config["ros_node_address"])
        rclpy.spin_once(wei_execution_node)
        wei_execution_node.destroy_node()
        rclpy.shutdown()
        return str(state)
    
if __name__ == "__main__":
    rclpy.init()
    # node = weiExecNode()
    # node.send_wei_goal("/ur_module","pick")
    node = ROS2Interface(name="WeiActionExecuter")
    name = "run demo"
    module = "ur_node"
    action = "pick"
    step = Step(name=name, module=module,action=action)
    module = Module(name=name, module=module,action=action, interface="wei_ros_node", config={"ros_node_address":"ur_node"})
    module
    node.send_action(step,module)
