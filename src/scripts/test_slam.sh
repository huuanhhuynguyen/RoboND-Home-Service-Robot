#!/bin/sh
xterm  -e  " roslaunch turtlebot turtlebot_world.launch " &
sleep 3
xterm  -e  " roslaunch turtlebot gmapping_demo.launch " &
sleep 3
xterm  -e  " roslaunch turtlebot view_navigation.launch " &
sleep 3
xterm  -e  " rosrun teleop_twist_keyboard teleop_twist_keyboard.py " &
sleep 3