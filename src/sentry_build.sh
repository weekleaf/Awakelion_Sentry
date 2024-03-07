#!/bin/bash

./livox_ros_driver2/build.sh ROS1
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_lio"
catkin_make -DCATKIN_WHITELIST_PACKAGES="serial"
catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_driver"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""

