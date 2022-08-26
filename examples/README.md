# WEI Examples


## How to run it:
  1 - Make sure sealer, peeler and hudson are all connected to parker
  2 - For sanity sake. I also rebooted parker to make sure there were no other running nodes.
  3 - Install packages
  4 - Run ros nodes 
  5 - Run the script

## Installing the example
On parker:
```
mkdir ~/wei_ws/src
cd ~/wei_ws/src
git clone https://github.com/AD-SDL/azenta_driver 
git clone https://github.com/AD-SDL/hudson_driver
git clone https://github.com/AD-SDL/pf400_driver
git clone https://github.com/AD-SDL/wei_ros
cd ..
colcon build 
```
## Running the example
```
On parker start the ROS2 nodes:
source ~/wei_ws/install/setup.sh && 
ros2 launch sp_module_client sp_module.launch.py && 
ros2 run pf400_description pf400_Joint_Control && 
ros2 run sciclops_client sciclops_client &&
```
And finally run the scripts with the steps you want to test
```
./~/wei_ws/src/wei_ros/examples/test_wf.py
```