# cartesian-impedance-control
Impedance Controllers for pHRI on KUKA LWR 4+ using Stanford FRI

Institut Pascal
MACCS Team (http://ip.univ-bpclermont.fr/index.php/fr/maccs)
UMR6602 Clermont Ferrand

In Real Mode
============
In real mode, the controllers are launched in separate computers.

-> On arms computer
(FRI - Fast Research Interface need to be run in sudo)
sudo -s su
source devel/setup.bash
roslaunch double_lwr_robot double_arms.launch

-> For left arm only
(FRI - Fast Research Interface need to be run in sudo)
sudo -s su
source devel/setup.bash
roslaunch double_lwr_robot double_arms.launch use_right_arm:=false

-> On pantilt computer
source devel/setup.bash
roslaunch pan_tilt_real pan_tilt_real.launch

You can move/control the robots like in simulation mode, so let's have a look to the simulation part.


In Simulation/Gazebo Mode
=========================

How to launch the simulation of Sigma Platform (arms + pantilt) :
---------------------------------------------------------------
source devel/setup.bash
roslaunch platform_gazebo platform_gazebo.launch use_pantilt:=true

If you don't need the pantilt -> roslaunch platform_gazebo platform_gazebo.launch use_pantilt:=false


How to move the pantilt in position :
-----------------------------------
rostopic pub -1 /pantilt/pantilt_group_position_controller/command std_msgs/Float64MultiArray "data: [1.5,-0.5,0.5,0.5]"

How to see the image_raw of each camera :
-----------------------------------------
rosrun image_view image_view image:=/pantilt/camera1/image_raw
rosrun image_view image_view image:=/pantilt/camera2/image_raw



How to move the right or left kuka lwr arm :
------------------------------------------

The namespace 'kuka_lwr_right' is used to control the right arm.
The namespace 'kuka_lwr_left' id used to control the left arm.

-> Get a list of ros services (ros controllers) available for the right arm (so need to use the 'kuka_lwr_right' namespace) :
rosservice call /kuka_lwr_right/controller_manager/list_controllers

you can do the same for the left arm -> rosservice call /kuka_lwr_left/controller_manager/list_controllers

An example of available controllers (loaded but stopped by default): (of course you can write your own controller !)

controller: 
  - 
    name: joint_state_controller
    state: running
    type: joint_state_controller/JointStateController
    hardware_interface: hardware_interface::JointStateInterface
    resources: []
  - 
    name: kuka_group_command_controller_fri
    state: stopped
    type: kuka_lwr_controllers/GroupCommandControllerFRI
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['kuka_lwr_right_0_joint', 'kuka_lwr_right_1_joint', 'kuka_lwr_right_2_joint', 'kuka_lwr_right_3_joint', 'kuka_lwr_right_4_joint', 'kuka_lwr_right_5_joint', 'kuka_lwr_right_6_joint']
  - 
    name: kuka_one_task_inverse_kinematics
    state: stopped
    type: kuka_lwr_controllers/OneTaskInverseKinematics
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['kuka_lwr_right_0_joint', 'kuka_lwr_right_1_joint', 'kuka_lwr_right_2_joint', 'kuka_lwr_right_3_joint', 'kuka_lwr_right_4_joint', 'kuka_lwr_right_5_joint', 'kuka_lwr_right_6_joint']
  - 
    name: cartesian_velocity_control
    state: stopped
    type: kuka_lwr_controllers/CartesianVelocityControl
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['kuka_lwr_right_0_joint', 'kuka_lwr_right_1_joint', 'kuka_lwr_right_2_joint', 'kuka_lwr_right_3_joint', 'kuka_lwr_right_4_joint', 'kuka_lwr_right_5_joint', 'kuka_lwr_right_6_joint']

-> How to Start the position controller 'kuka_group_command_controller_fri' for the 'kuka_lwr_right' arm :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['kuka_group_command_controller_fri'], stop_controllers: [], strictness: 2}"

The same for the left arm -> rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: ['kuka_group_command_controller_fri'], stop_controllers: [], strictness: 2}"


-> How to Send positions with the controller 'kuka_group_command_controller_fri' for the 'kuka_lwr_right' arm :
rostopic pub -1 /kuka_lwr_right/kuka_group_command_controller_fri/command std_msgs/Float64MultiArray "data: [0.9,0.9,0.5,0.5,0.3,0.5,0.8]"

The same for the left arm -> rostopic pub -1 /kuka_lwr_left/kuka_group_command_controller_fri/command std_msgs/Float64MultiArray "data: [0.9,0.9,0.5,0.5,0.3,0.5,0.8]"

-> How to Stop the position controller 'kuka_group_command_controller_fri' for the 'kuka_lwr_right' arm :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['kuka_group_command_controller_fri'], strictness: 1}"

-> How to Start 'cartesian inverse kinematic' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['kuka_one_task_inverse_kinematics'], stop_controllers: [], strictness: 2}"

-> How to Send cartesian position :
rostopic pub -1 /kuka_lwr_right/kuka_one_task_inverse_kinematics/command kuka_lwr_controllers/PoseRPY '{id: 1, position: {x: -0.4, y: 0.3, z: 0.9}}'

-> How to Stop 'cartesian inverse kinematic' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['kuka_one_task_inverse_kinematics'], strictness: 1}"


-> How to start 'kuka_gravity_compensation_controller' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['kuka_gravity_compensation_controller'], stop_controllers: [], strictness: 2}"

-> How to stop 'kuka_gravity_compensation_controller' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['kuka_gravity_compensation_controller'], strictness: 2}"

-> How to start 'torque_based_position_controller' controller :
rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: ['torque_based_position_controller'], stop_controllers: [], strictness: 2}"

-> How to stop 'torque_based_position_controller' controller :
rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['torque_based_position_controller'], strictness: 2}"


RVIZ only
=========

roslaunch platform_rviz platform_rviz.launch

Cartesian computed torque controller
====================================

In simulation Mode ->

-> Launch ros controllers and gazebo
roslaunch platform_gazebo platform_gazebo.launch

In real Mode ->
left arm only -> 
roslaunch double_lwr_robot double_arms.launch use_right_arm:=false

right arm only ->
roslaunch double_lwr_robot double_arms.launch use_left_arm:=false

-> Go to a specific position to avoid singularity :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['kuka_group_command_controller_fri'], stop_controllers: [], strictness: 2}"
rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: ['kuka_group_command_controller_fri'], stop_controllers: [], strictness: 2}"

rostopic pub -1 /kuka_lwr_right/kuka_group_command_controller_fri/command std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]"
rostopic pub -1 /kuka_lwr_left/kuka_group_command_controller_fri/command std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]"

rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['kuka_group_command_controller_fri'], strictness: 2}"
rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['kuka_group_command_controller_fri'], strictness: 2}"

-> You can do the same actions with a specific bash script (launch from the workspace root) :
./src/platform_gazebo/start_specific_position.sh right "[0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]"
./src/platform_gazebo/start_specific_position.sh left "[0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]"

-> Start/Stop 'cartesian_computed_torque_controller' :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['cartesian_computed_torque_controller'], stop_controllers: [], strictness: 2}"
rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: ['cartesian_computed_torque_controller'], stop_controllers: [], strictness: 2}"

rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['cartesian_computed_torque_controller'], strictness: 2}"
rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['cartesian_computed_torque_controller'], strictness: 2}"


-> Publish Pose :
rostopic pub -1 /kuka_lwr_right/cartesian_computed_torque_controller/command kuka_lwr_controllers/PoseRPY '{id: 1, position: {x: -0.5, y: 0.0, z: 0.9}}'
rostopic pub -1 /kuka_lwr_left/cartesian_computed_torque_controller/command kuka_lwr_controllers/PoseRPY '{id: 1, position: {x: -0.5, y: 0.0, z: 0.9}}'

rostopic pub -1 /kuka_lwr_right/cartesian_computed_torque_controller/command kuka_lwr_controllers/PoseRPY '{id: 0, position: {x: -0.5, y: 0.0, z: 0.9}, orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}}'
rostopic pub -1 /kuka_lwr_left/cartesian_computed_torque_controller/command kuka_lwr_controllers/PoseRPY '{id: 0, position: {x: -0.5, y: 0.0, z: 0.9}, orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}}'

-> Set PID gains :
rostopic pub -1 /kuka_lwr_right/cartesian_computed_torque_controller/set_gains std_msgs/Float64MultiArray "data: [3000,1800,1800,1400,100,100,100,181,62,53,54,10,10,10]"
rostopic pub -1 /kuka_lwr_left/cartesian_computed_torque_controller/set_gains std_msgs/Float64MultiArray "data: [3000,1800,1800,1400,100,100,100,181,62,53,54,10,10,10]"

For Moveit!
===========

-> Launch controllers + moveit! + gazebo
roslaunch platform_gazebo platform_gazebo.launch load_moveit:=true 

-> Launch rviz plugin for moveit!
roslaunch platform_gazebo show_moveit_rviz.launch use_left_arm:=true 

-> Launch a test node using move group
roslaunch test_move_group test_cartesian_position.launch group_name:=kuka_lwr_left

Launch rqt plugin to start/stop controllers graphically
=======================================================

rosrun rqt_gui rqt_gui

Menu Plugins / Robot Tools / Controller Manager


Simple Cartesian Controller :
===========================

-> How to start 'kuka_simple_cartesian_impedance_controller' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['kuka_simple_cartesian_impedance_controller'], stop_controllers: [], strictness: 2}"

-> How to stop 'kuka_simple_cartesian_impedance_controller' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['kuka_simple_cartesian_impedance_controller'], strictness: 2}"

-> Set Cartesian stiffness or damping values
rostopic pub -1 /kuka_lwr_right/kuka_simple_cartesian_impedance_controller/setCartesianStiffness std_msgs/Float64MultiArray "data: [800.0, 800.0, 800.0, 50.0, 50.0, 50.0]"
rostopic pub -1 /kuka_lwr_right/kuka_simple_cartesian_impedance_controller/setCartesianDamping std_msgs/Float64MultiArray "data: [ 0.8, 0.8, 0.8, 0.8, 0.8, 0.8]"


-> Set Cartesian pose or wrench values
-> Pose data to send -> [RXX, RXY, RXZ, TX, RYX, RYY, RYZ, TY, RZX, RZY, RZZ, TZ]
rostopic pub -1 /kuka_lwr_right/kuka_simple_cartesian_impedance_controller/setCartesianPose std_msgs/Float64MultiArray "data: [RXX, RXY, RXZ, TX, RYX, RYY, RYZ, TY, RZX, RZY, RZZ, TZ]"
-> Wrench data to send -> [fx, fy, fz, tx, ty, tz]
rostopic pub -1 /kuka_lwr_right/kuka_simple_cartesian_impedance_controller/setCartesianWrench std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.01, 0.0, 0.0, 0.0]"
