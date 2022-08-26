


source ~/wei_ws/install/setup.sh && 
ros2 launch sp_module_client sp_module.launch.py && 
ros2 run pf400_description pf400_Joint_Control && 
ros2 run sciclops_client sciclops_client 