#!/bin/bash

# Laurent LEQUIEVRE
# laurent.lequievre@uca.fr
# UMR 6602 - Institut Pascal


source /opt/ros/indigo/setup.bash
export ROS_MASTER_URI=http://ifma-kuka-test:11311
if [ $1 = "left" ]
	then
		namespace="kuka_lwr_left"
	else
		namespace="kuka_lwr_right"
	fi
rostopic pub -1 /$namespace/kuka_group_command_controller_fri/command std_msgs/Float64MultiArray "data: [0.9,0.9,0.5,0.5,0.3,0.5,0.8]"





