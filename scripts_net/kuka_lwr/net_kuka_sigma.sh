#!/bin/sh

# Laurent LEQUIEVRE
# laurent.lequievre@univ-bpclermont.fr
# UMR 6602 - Institut Pascal

# NETWORK INFORMATIONS
# ====================
# sudo lshw -C network
# eth0 -> pci=0000:04:00.0, hw=68:05:ca:3e:3d:35, card=82574L Gigabit Network Connection, driver=e1000e
# eth1 -> pci=0000:06:00.0, hw=f0:4d:a2:32:39:d3, card=NetXtreme BCM5761 Gigabit Ethernet PCIe, driver=tg3
# eth2 -> pci=0000:22:00.0, hw=00:1b:21:b3:ae:27, card=82574L Gigabit Network Connection, driver=e1000e
# eth3 -> pci=0000:23:00.0, hw=68:05:ca:3e:3b:52, card=82574L Gigabit Network Connection, driver=e1000e
# eth4 -> pci=0000:24:00.0, hw=68:05:ca:3e:3d:36, card=82574L Gigabit Network Connection, driver=e1000e


# eth2 -> kuka left
# eth0 -> kuka right
# eth3 -> switch
ethX_associate_kuka_left="eth2"
ethX_associate_kuka_right="eth0"
ethX_associate_switch="eth3"
ethX_associate_fsensor="eth4"

# PCI address of each ethX associated
pci_address_associate_kuka_left="0000:22:00.0"
pci_address_associate_kuka_right="0000:04:00.0"
pci_adresse_associate_fsensor="0000:24:00.0"

# Define IP address for RT eth0 and eth1
ip_address_associate_kuka_left="192.168.100.102"
ip_address_associate_kuka_right="192.168.100.120"
ip_address_associate_switch="192.168.100.123"
ip_address_associate_sub_net_switch="192.168.100.0/24"
ip_address_associate_fsensor="192.168.1.100"

# Define Netmask for RT eth0 and eth1
netmask_associate_kuka_left="255.255.255.255"
netmask_associate_kuka_right="255.255.255.255"
netmask_associate_switch="255.255.255.0"
netmask_associate_fsensor="255.255.255.255"

# Define hw of each ethX associated
hw_ethX_associate_kuka_left="00:1b:21:b3:ae:27"
hw_ethX_associate_kuka_right="68:05:ca:3e:3d:35"
hw_ethX_associate_switch="68:05:ca:3e:3b:52"
hw_ethX_associate_fsensor="68:05:ca:3e:3d:36"

# Define IP address of kuka arms
ip_address_kuka_right="192.168.100.253"
ip_address_kuka_left="192.168.100.254"
ip_address_fsensor="192.168.1.1"

# Define IP address for local loopback
ip_address_loopback="127.0.0.1"

# Define UDP port for kuka arms
udp_port_kuka_right=49938
udp_port_kuka_left=49939

do_start()
{
	echo "Start net script (RT and non RT) of kuka right arm and also switch !"
	
	echo "ifconfig down (non RT) ethX associate to kuka right !"
	sudo ifconfig $ethX_associate_kuka_right down

	echo "ifconfig down (non RT) ethX associate to kuka left !"
	sudo ifconfig $ethX_associate_kuka_left down
	
	echo "ifconfig down (non RT) ethX associated to switch !"
	sudo ifconfig $ethX_associate_switch down

	echo "ifconfig down (non RT) ethX associated to fsensor !"
	sudo ifconfig $ethX_associate_fsensor down
	
	echo "ifconfig up (non RT) ethX associate to kuka right !!"
	sudo ifconfig $ethX_associate_kuka_right up $ip_address_associate_kuka_right netmask $netmask_associate_kuka_right hw ether $hw_ethX_associate_kuka_right

	echo "ifconfig up (non RT) ethX associate to kuka left !!"
	sudo ifconfig $ethX_associate_kuka_left up $ip_address_associate_kuka_left netmask $netmask_associate_kuka_left hw ether $hw_ethX_associate_kuka_left
	
	echo "Start ethX (non RT) associate to switch !"
	sudo ifconfig $ethX_associate_switch up $ip_address_associate_switch netmask $netmask_associate_switch hw ether $hw_ethX_associate_switch

	echo "Start ethX (non RT) associate to fsensor !"
	sudo ifconfig $ethX_associate_fsensor up $ip_address_associate_fsensor netmask $netmask_associate_fsensor hw ether $hw_ethX_associate_fsensor 
      
	echo "Add (non RT) route to kuka right arm"
	sleep 5
	sudo route add -host $ip_address_kuka_right dev $ethX_associate_kuka_right

	echo "Add (non RT) route to kuka left arm"
	sleep 5
	sudo route add -host $ip_address_kuka_left dev $ethX_associate_kuka_left

	echo "Add (non RT) route to fsensor"
	sleep 5
	sudo route add -host $ip_address_fsensor dev $ethX_associate_fsensor


	echo "Add iptables for kuka right port"
	sudo iptables -A INPUT -p udp -i eth0 --dport $udp_port_kuka_right -j ACCEPT
	sudo iptables -A OUTPUT -p udp --dport $udp_port_kuka_right -j ACCEPT

	echo "Add iptables for kuka left port"
	sudo iptables -A INPUT -p udp -i eth0 --dport $udp_port_kuka_left -j ACCEPT
	sudo iptables -A OUTPUT -p udp --dport $udp_port_kuka_left -j ACCEPT

	
    	#echo "Delete all routes of ethX (non RT) associate to switch !"
	#sleep 5
	#sudo route del -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch
        
	#echo "Add sub network (non RT) for switch !"
	#sleep 5
	#sudo route add -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch
}

do_stop()
{
	echo "Stop net script (RT) !"
	
	#sleep 1    
    	#echo "Stop ethX (non RT) associate to kuka right !"
    	#sudo ifconfig $ethX_associate_kuka_right down

	#sleep 1    
    	#echo "Stop ethX (non RT) associate to kuka left !"
    	#sudo ifconfig $ethX_associate_kuka_left down
    
    	#echo "ifconfig up ethX associate to kuka right arm !"
  
    	#sudo ifconfig $ethX_associate_kuka_right up $ip_address_associate_kuka_right netmask $netmask_associate_kuka_right hw ether $hw_ethX_associate_kuka_right
	
    	#echo "Add route (non RT) to kuka right arm !"
    	#sleep 5
    	#sudo route add -host $ip_address_kuka_right dev $ethX_associate_kuka_right
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
