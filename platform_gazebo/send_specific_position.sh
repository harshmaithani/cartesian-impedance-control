#!/bin/sh

if [ $1 = "left" ]
then
	namespace="kuka_lwr_left"
else
	namespace="kuka_lwr_right"
fi

rostopic pub -1 /$namespace/kuka_group_command_controller_fri/command std_msgs/Float64MultiArray "data: $2"
