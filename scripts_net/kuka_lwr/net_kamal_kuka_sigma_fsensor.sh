#!/bin/sh

# Laurent LEQUIEVRE
# laurent.lequievre@univ-bpclermont.fr
# UMR 6602 - Institut Pascal

# Ip address of Kuka Computer -> 192.168.100.123
# Ip address Of User Computer -> 192.168.100.105

# eth0 -> network card used to communicate with the pan tilt computer by using the switch.

# NETWORK INFORMATIONS
# ====================
# sudo lshw -C network
# on my computer :
# eth0 -> 82579LM Gigabit Network Connection, pci@0000:00:19.0, 2c:44:fd:68:fb:58, e1000e


# Table de routage actuelle
# route -n

# Associate ethX
# eth0 -> switch
ethX_associate_switch="eth0"
ethX_associate_fsensor="eth1"


# Define IP address for ethX associate to switch and ethX associate to pan tilt
ip_address_associate_kuka_computer="192.168.100.123"
ip_address_associate_switch="192.168.100.105"
ip_address_associate_fsensor="192.168.1.100"

ip_address_associate_sub_net_switch="192.168.100.0/24"
ip_address_fsensor="192.168.1.1"

# Define Netmask
netmask_associate_switch="255.255.255.0"
netmask_associate_computer="255.255.255.0"
netmask_associate_fsensor="255.255.255.255"

# Define hw of each ethX associated
hw_ethX_associate_switch="30:e1:71:18:ab:c6"
hw_ethX_associate_fsensor="74:da:38:9f:fa:49"

# path of ax88179_178a linux module
path_of_ax88179_178a="/home/kmohyeldin/git_projects/AX88179_178A/ax88179_178a.ko"

do_start()
{
	echo "load usb and ax88179_178a linux modules !"
	sudo modprobe usbnet
	sleep 3

	sudo insmod $path_of_ax88179_178a
	sleep 3

	echo "Start net config for user computer !"
	
	echo "ifconfig down ethX associated to switch !"
	sudo ifconfig $ethX_associate_switch down

	echo "ifconfig down ethX associated to fsensor !"
	sudo ifconfig $ethX_associate_fsensor down

	sleep 3

	echo "Start ethX associate to switch !"
	sudo ifconfig $ethX_associate_switch up $ip_address_associate_switch netmask $netmask_associate_switch hw ether $hw_ethX_associate_switch

	echo "Start ethX associate to fsensor !"
	sudo ifconfig $ethX_associate_fsensor up $ip_address_associate_fsensor netmask $netmask_associate_fsensor hw ether $hw_ethX_associate_fsensor 
        
	echo "Delete all routes of ethX associate to switch !"
	sleep 5
	sudo route del -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch

	echo "Add host route (kuka computer) for switch !"
	sleep 5
	sudo route add -host $ip_address_associate_kuka_computer dev $ethX_associate_switch

	echo "Add (non RT) route to fsensor"
	sleep 5
	sudo route add -host $ip_address_fsensor dev $ethX_associate_fsensor
}

do_stop()
{
 	echo "unload usb and ax88179_178a linux modules !"
 	sudo rmmod ax88179_178a
	sleep 3

 	sudo rmmod usbnet
	sleep 3

	echo "ifconfig down ethX associated to switch !"
	sudo ifconfig $ethX_associate_switch down

	echo "ifconfig down (non RT) ethX associated to fsensor !"
	sudo ifconfig $ethX_associate_fsensor down

	echo "Delete all routes of ethX associate to switch !"
	sleep 5
	sudo route del -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch

	echo "Delete all routes of ethX associate to fsensor !"
	sleep 5
	sudo route del -host $ip_address_fsensor dev $ethX_associate_fsensor
	
}


case "$1" in
   start)
      do_start
      ;;
   stop)
      do_stop
      ;;
   *)
      echo "--> Usage: $0 {start|stop}"
      exit 1
esac

exit 0
