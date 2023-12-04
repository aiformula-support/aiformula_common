#!/bin/sh

sudo modprobe kvaser_usb
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

#simulation
# sudo modprobe vcan
# sudo ip link add dev vcan0 type vcan
# sudo ip link set vcan0 up