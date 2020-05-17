Laurent LEQUIEVRE
Institut Pascal umr6602
laurent.lequievre@uca.fr

0- Add Ip address and name of the user's laptop to /etc/hosts of the pan tilt computer:
192.168.100.105 HP-8570w
(In that case the name of my laptop is HP-8570w and its IP address is always 192.168.100.105)

1- Launch the network configuration from the pan tilt computer :
Menu Applications/ Network / Start Network Config

2- Test the pan tilt robot's network address from the pan tilt computer :
start a ping 192.168.100.252 in a terminal window. 
Or
Menu Applications / Network / Test Pan Tilt connexion

3- Test Camera acquisition by using coriander software.
The name of the 2 cameras are :
Imaging Source DFx 31BF03-Z2
Imaging Source DFx 21BF04-Z2

4- Start ROS launch
Menu Applications / PanTilt / Start Drivers with cameras
or
Menu Applications / PanTilt / Start Drivers without cameras

5- Set Cameras to initial Pose
Menu Applications / PanTilt / Set to Init Pose

6- Launch the network configuration from the user's laptop :
see and adapt the script sample net_user_pan_tilt_sigma.h saved in the directory :
/home/pantilt/projects/catkin_sigma_ws/src/scripts_net

You just have to adapt the hardware address of eth0 -> 
for example : hw_ethX_associate_switch="2c:44:fd:68:fb:58"

7- Add Ip address and name of pan tilt computer to /etc/hosts of user's laptop :
192.168.100.103 pantilt-OptiPlex-7040

8- On each new terminal window on user's laptop, set the ROS MASTER URI :
export ROS_MASTER_URI=http://pantilt-OptiPlex-7040:11311

rostopic list
rostopic pub -1 /pantilt/pan_tilt_position_controller/command std_msgs/Float64MultiArray "data: [1.5,0.0,0.0,0.0]"




