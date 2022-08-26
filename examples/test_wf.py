#!/usr/bin/env python3

from hudson_driver.sciclops_driver import sciclops_driver
import rclpy
from wei_executor.weiExecutorNode import weiExecNode


rclpy.init()
wei_execution_node = weiExecNode()


##module name 
# for now this will be the full action_handler service address
pf400_node = '/PF400_Joint_Node/action_handler'
sciclops_node = ''
sealer_node = ''
peeler_node = ''


##Positions for PF400
peelerPos = [264.584, -29.413,	284.376, 372.338, 0.0, 651.621]
sealerPos = [231.788, -27.154, 313.011, 342.317, 0.0, 683.702]
cyclops_ext = [262.550, 20.608, 119.290, 662.570, 0.0, 574.367]

## Static flow definition 
flow_def = [
    {'node':sciclops_node,'action_handle':'get_plate','action_vars':{'pos':'tower1'}},
    {'node':pf400_node,'action_handle':'transfer','action_vars':{'pos1':cyclops_ext,'pos2':sealerPos}},
#    {'node':sealer_node,'action_handle':'seal','action_vars':{'time':175,'temp':3}},
    {'node':pf400_node,'action_handle':'transfer','action_vars':{'pos1':sealerPos,'pos2':peelerPos}},
#    {'node':peeler_node,'action_handle':'peel','action_vars':{}},
    {'node':pf400_node,'action_handle':'transfer','action_vars':{'pos1':peelerPos,'pos2':cyclops_ext}}
    ]


for step in flow_def:
    wei_execution_node.send_wei_command(step['node'],step['action_handle'],step['action_vars'])

