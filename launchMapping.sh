#!/bin/bash

xterm -hold -e "roscore" &
sleep 1; xterm -hold -e "cd catkin_ws; roslaunch robot.launch" &
sleep 2; xterm -hold -e "cd catkin_ws; roslaunch hokuyo.launch" &
sleep 3; xterm -hold -e "cd catkin_ws; roslaunch gmapping.launch"

