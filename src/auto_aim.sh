#!/bin/bash

gnome-terminal -e 'bash -c "source /opt/ros/noetic/setup.bash; roscore; exec bash"'
sleep 3
gnome-terminal -e 'bash -c "source ../devel/setup.bash; roslaunch robot_driver robot_driver.launch; exec bash"'
gnome-terminal -e 'bash -c "./auto_aim_node; exec bash"'
#gnome-terminal -e 'bash -c "source ../devel/setup.bash; rqt_plot; exec bash"'
