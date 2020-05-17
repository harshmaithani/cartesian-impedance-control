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
rosservice call /$namespace/controller_manager/list_controllers

echo "Appuyer la touche <Entrée> pour continuer..."
read touche




