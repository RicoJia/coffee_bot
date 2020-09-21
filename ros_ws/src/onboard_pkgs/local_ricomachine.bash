#!/bin/bash
export ROS_HOSTNAME=192.168.1.11    #this machine
export ROS_MASTER_URI=http://192.168.1.11:11311    #this machine
exec "$@"