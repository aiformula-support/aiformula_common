#!/bin/bash

# IMU
CURRENT=$(cd $(dirname $0);pwd)
sed -i 's/tf2_geometry_msgs\.hpp/tf2_geometry_msgs.h/g' ${CURRENT%/*/*}/sensing/vectornav/vectornav/src/vn_sensor_msgs.cc
