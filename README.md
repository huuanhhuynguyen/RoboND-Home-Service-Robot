sudo apt-get install xterm
chmod +x all scripts
clone mapping
ros install gmapping
rosdep -i install turtlebot_gazebo
pip install rospkg
install mapserver, amcl and movebase
to create map rosrun map_server map_saver -f src/map/my_map
edit name of map file in turtlebot_world.launch

