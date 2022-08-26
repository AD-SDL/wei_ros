



ROS_WS=~/wei_ws

mkdir $ROS_WS/src
cd $ROS_WS/src
git clone https://github.com/AD-SDL/azenta_driver 
git clone https://github.com/AD-SDL/hudson_driver
git clone https://github.com/AD-SDL/pf400_driver
git clone https://github.com/AD-SDL/wei_ros
cd ..
colcon build 
