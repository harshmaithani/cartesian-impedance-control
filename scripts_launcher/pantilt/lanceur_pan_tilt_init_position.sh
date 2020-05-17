#!/bin/bash

# Laurent LEQUIEVRE
# laurent.lequievre@univ-bpclermont.fr
# UMR 6602 - Institut Pascal

source /home/pantilt/git_project/platform-sigma/devel/setup.bash
 rostopic pub -1 /pantilt/pan_tilt_position_controller/command std_msgs/Float64MultiArray "data: [1.5,0.0,0.0,0.0]"

