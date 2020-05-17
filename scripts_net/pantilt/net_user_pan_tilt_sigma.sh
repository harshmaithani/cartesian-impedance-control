#!/bin/sh

# Laurent LEQUIEVRE
# laurent.lequievre@univ-bpclermont.fr
# UMR 6602 - Institut Pascal

# Ip address of Pan Tilt Computer -> 192.168.100.103
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

# Define IP address for ethX associate to switch and ethX associate to pan tilt
ip_address_associate_pan_tilt_computer="192.168.100.103"
ip_address_associate_switch="192.168.100.105"

ip_address_associate_sub_net_switch="192.168.100.0/24"

# Define Netmask
netmask_associate_switch="255.255.255.0"
netmask_associate_computer="255.255.255.0"

# Define hw of each ethX associated
hw_ethX_associate_switch="2c:44:fd:68:fb:58"

do_start()
{
	echo "Start net config for user computer !"
	
	echo "ifconfig down ethX associated to switch !"
	sudo ifconfig $ethX_associate_switch down

	echo "Start ethX associate to switch !"
	sudo ifconfig $ethX_associate_switch up $ip_address_associate_switch netmask $netmask_associate_switch hw ether $hw_ethX_associate_switch
        
	echo "Delete all routes of ethX associate to switch !"
	sleep 5
	sudo route del -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch

	echo "Add host route (pan tilt computer) for switch !"
	sleep 5
	sudo route add -host $ip_address_associate_pan_tilt_computer dev $ethX_associate_switch
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
