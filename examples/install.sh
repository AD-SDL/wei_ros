ROS_WS=~/wei_ws_dev_kyle

mkdir $ROS_WS
mkdir $ROS_WS/src
cd $ROS_WS/src
git clone https://github.com/AD-SDL/azenta_driver
cd $ROS_WS/src/azenta_driver/
git checkout 55d4598
cd $ROS_WS/src
git clone https://github.com/AD-SDL/hudson_driver
git clone https://github.com/AD-SDL/pf400_driver
git clone https://github.com/AD-SDL/wei_ros
git clone https://github.com/AD-SDL/rpl_wei
cd $ROS_WS/src/rpl_wei
pip3 install -r requirements/requirements.txt
pip3 install -e .
cd $ROS_WS/src
git clone https://github.com/AD-SDL/rpl_workcell
cd ..
colcon build
