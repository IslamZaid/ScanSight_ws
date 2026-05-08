#!/bin/bash
#sudo ip addr add 192.168.50.100/24 dev enx30d0420a74c9
#sudo ip link set enx30d0420a74c9 up
#ping -c 2 192.168.50.110


#!/bin/bash

IFACE="enx30d0420a74c9"
IP="192.168.50.100/24"
ROBOT_IP="192.168.50.110"

sudo ip addr flush dev "$IFACE"
sudo ip addr add "$IP" dev "$IFACE"
sudo ip link set "$IFACE" up

ping -c 4 "$ROBOT_IP"
