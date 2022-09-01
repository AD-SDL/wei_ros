#!bin/bash

source ~/wei_ws_dev_kyle/install/setup.bash &
ros2 launch sp_module_client sp_module.launch.py &
ros2 launch pf400_client pf400_client.launch.py &
ros2 launch sciclops_module_client sciclops_module.launch.py &
wait
