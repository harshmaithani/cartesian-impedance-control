#!/bin/bash

# Laurent LEQUIEVRE
# laurent.lequievre@uca.fr
# UMR 6602 - Institut Pascal

export ROS_MASTER_URI=http://ifma-kuka-test:11311
cd /home/ifma/git_project/platform-sigma
source devel/setup.bash
roslaunch double_lwr_robot double_arms.launch use_right_arm:=false
