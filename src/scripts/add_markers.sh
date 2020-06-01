#!/bin/sh
xterm  -e  " roslaunch turtlebot turtlebot_world.launch " &
sleep 3
xterm  -e  " roslaunch turtlebot amcl_demo.launch " &
sleep 3
xterm  -e  " roslaunch turtlebot view_navigation.launch " &
sleep 3
xterm  -e  " rosrun add_markers add_markers_node " &
sleep 3