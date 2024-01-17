#!/bin/sh

sudo modprobe kvaser_usb
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
