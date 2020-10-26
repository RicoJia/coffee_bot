#!/bin/bash
export DEFAULT_WS=/home/coffeebot/default_coffeebot_pkgs
source $DEFAULT_WS/devel/setup.bash
export ONBOARD_PKGS_WS=/home/coffeebot/onboard_pkgs_ws
source $ONBOARD_PKGS_WS/devel/setup.bash
export ROS_MASTER_URI=http://192.168.1.11:11311 #use hostname -I to check local IP
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$DEFAULT_WS
export PATH=$ROS_ROOT/bin:$PATH
export ROS_HOSTNAME=192.168.1.29  #this should be your node
sudo chown ${USER} /dev/gpiomem
export DISPLAY=:0
exec "$@"

