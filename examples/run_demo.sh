#!bin/bash

source ~/wei_ws_dev_kyle/install/setup.bash &
python3 ~/wei_ws_dev_kyle/src/wei_ros/examples/test_wei_wf.py -wc ~/wei_ws_dev_kyle/src/rpl_workcell/pcr_workcell/pcr_workcell.yaml -wf ~/wei_ws/src/rpl_workcell/pcr_workcell/workflows/week_demo_workflow.yaml
