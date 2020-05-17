#!/bin/sh


if [ $1 = "left" ]
then
	namespace="kuka_lwr_left"
else
	namespace="kuka_lwr_right"
fi

rosservice call /$namespace/controller_manager/switch_controller "{start_controllers: ['kuka_group_command_controller_fri'], stop_controllers: [], strictness: 2}"

rostopic pub -1 /$namespace/kuka_group_command_controller_fri/command std_msgs/Float64MultiArray "data: $2"

sleep 7

rosservice call /$namespace/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['kuka_group_command_controller_fri'], strictness: 2}"
