#!/bin/bash

# Laurent LEQUIEVRE
# laurent.lequievre@uca.fr
# UMR 6602 - Institut Pascal

cd /home/hand/git_project/platform-sigma
source devel/setup.bash
export ROS_MASTER_URI=http://ifma-kuka-test:11311 
roslaunch force_torque_sensor FTsensor_13953.launch
