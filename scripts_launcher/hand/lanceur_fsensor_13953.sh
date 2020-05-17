#!/bin/bash

# Laurent LEQUIEVRE
# laurent.lequievre@uca.fr
# UMR 6602 - Institut Pascal
# Used for the FT Sensor n° 13953

cd /home/hand/git_project/platform-sigma
source devel/setup.bash
roslaunch force_torque_sensor FTsensor_13953.launch
