Laurent LEQUIEVRE
Institut Pascal umr6602
laurent.lequievre@uca.fr

0- Add Ip address and name of the user's laptop to /etc/hosts of the kuka computer:
192.168.100.105 HP-8570w
(In that case the name of my laptop is 'HP-8570w' and its IP address is always 192.168.100.105)

1- Launch the network configuration from hand computer :
Menu Applications/ Network / Start Network Config

2- Launch the network configuration from the user's laptop :
see and adapt the script sample net_user_kuka_sigma.h saved in the directory :
git_project/platform_sigma/src/scripts_net/kuka

You just have to adapt the hardware address of eth0 -> 
for example : hw_ethX_associate_switch="2c:44:fd:68:fb:58"

3- Add Ip address and name of kuka computer to /etc/hosts of user's laptop :
192.168.100.123 ????

4- On each new terminal window on user's laptop, set the ROS MASTER URI :
export ROS_MASTER_URI=http://ifma-kuka-test:11311

rostopic list
rostopic pub ....




