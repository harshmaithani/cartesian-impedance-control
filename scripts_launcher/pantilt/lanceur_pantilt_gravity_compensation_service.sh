#!/bin/bash

# Laurent LEQUIEVRE
# laurent.lequievre@uca.fr
# UMR 6602 - Institut Pascal

do_start()
{
	source /opt/ros/indigo/setup.bash
	export ROS_MASTER_URI=http://ifma-kuka-test:11311
	if [ $1 = "left" ]
	then
		namespace="kuka_lwr_left"
	else
		namespace="kuka_lwr_right"
	fi
	rosservice call /$namespace/controller_manager/switch_controller "{start_controllers: ['kuka_gravity_compensation_controller'], stop_controllers: [], strictness: 2}"
}

do_stop()
{
	source /opt/ros/indigo/setup.bash
	export ROS_MASTER_URI=http://ifma-kuka-test:11311
	if [ $1 = "left" ]
	then
		namespace="kuka_lwr_left"
	else
		namespace="kuka_lwr_right"
	fi
	rosservice call /$namespace/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['kuka_gravity_compensation_controller'], strictness: 1}"
}

case "$1" in
   start)
      do_start $2
      ;;
   stop)
      do_stop $2
      ;;
   *)
      echo "--> Usage: $0 {start|stop} {left|right}"
      exit 1
esac

exit 0

