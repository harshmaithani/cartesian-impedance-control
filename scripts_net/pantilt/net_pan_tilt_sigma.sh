#!/bin/sh

# Laurent LEQUIEVRE
# laurent.lequievre@univ-bpclermont.fr
# UMR 6602 - Institut Pascal

# Ip address of Pan Tilt -> 192.168.100.252

# NETWORK INFORMATIONS
# ====================
# sudo lshw -C network
# eth2 -> 82571EB Gigabit Ethernet Controller, pci@0000:07:00.0, 68:05:ca:1c:d7:c8, e1000e
# eth3 -> 82571EB Gigabit Ethernet Controller, pci@0000:07:00.1, 68:05:ca:1c:d7:c9, e1000e
# eth1 -> Ethernet Connection (2) I219-LM, pci@0000:00:1f.6, 48:4d:7e:ac:cf:9c, e1000e

# eth1 -> On board, connexion internet
# eth3 -> connexion pan tilt
# eth2 -> connexion switch

# Table de routage actuelle
# route -n

# Associate ethX
# eth3 -> pan tilt
# eth2 -> switch
ethX_associate_switch="eth2"
ethX_associate_pan_tilt="eth3"

# Define IP address for ethX associate to switch and ethX associate to pan tilt
ip_address_associate_switch="192.168.100.103"
ip_address_associate_sub_net_switch="192.168.100.0/24"

ip_address_associate_pan_tilt="192.168.100.104"
ip_address_associate_user_computer="192.168.100.105"

# Define IP address for pan tilt
ip_address_pan_tilt="192.168.100.252"

# Define Netmask for switch card and pan tilt card
netmask_associate_switch="255.255.255.0"
netmask_associate_pan_tilt="255.255.255.0"
netmask_associate_user_computer="255.255.255.0"

# Define hw of each ethX associated
hw_ethX_associate_switch="68:05:ca:1c:d7:c8"
hw_ethX_associate_pan_tilt="68:05:ca:1c:d7:c9"

do_start()
{
	echo "Start net config for switch and pan tilt !"
	
	echo "ifconfig down ethX associated to switch !"
	sudo ifconfig $ethX_associate_switch down
	
	echo "ifconfig down ethX associated to and pan tilt  !"
	sudo ifconfig $ethX_associate_pan_tilt down
	
	echo "Start ethX associate to switch !"
	sudo ifconfig $ethX_associate_switch up $ip_address_associate_switch netmask $netmask_associate_switch hw ether $hw_ethX_associate_switch
        
	echo "Start ethX associate to pan tilt !"
	sudo ifconfig $ethX_associate_pan_tilt up $ip_address_associate_pan_tilt netmask $netmask_associate_pan_tilt hw ether $hw_ethX_associate_pan_tilt
	
	echo "Delete all routes of ethX associate to switch and pan tilt !"
	sleep 5
	sudo route del -net $ip_address_associate_sub_net_switch dev $ethX_associate_pan_tilt
	sudo route del -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch

	echo "Add route to host pan tilt (using ip address) !"
	sleep 5
	sudo route add -host $ip_address_pan_tilt dev $ethX_associate_pan_tilt

	echo "Add sub network for switch !"
	sleep 5
	sudo route add -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch
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
