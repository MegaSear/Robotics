#!/bin/bash
#colcon build #--symlink-install
#. install/setup.bash
#ros2 launch crispr robot_display.launch.py #model:=src/description/robot.urdf.xacro


colcon build
. install/setup.bash
ros2 launch crispr sdf_parser.launch.py rviz:=True
