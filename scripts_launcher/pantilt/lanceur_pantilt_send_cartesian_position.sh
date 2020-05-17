#!/bin/bash

# Laurent LEQUIEVRE
# laurent.lequievre@uca.fr
# UMR 6602 - Institut Pascal


cd /home/pantilt/projects/ros_sigma_platform_fri_ws
source devel/setup.bash
export ROS_MASTER_URI=http://ifma-kuka-test:11311
if [ $1 = "left" ]
	then
		namespace="kuka_lwr_left"
	else
		namespace="kuka_lwr_right"
	fi
rostopic pub -1 /$namespace/kuka_one_task_inverse_kinematics/command kuka_lwr_controllers/PoseRPY "{id: 1, position: {x: -0.4, y: 0.3, z: 0.9}}"





