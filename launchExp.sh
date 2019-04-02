#!/bin/bash

xterm -hold -e "roscore" &
sleep 1; xterm -hold -e "cd catkin_ws; roslaunch robot.launch" & #RosAria
sleep 2; xterm -hold -e "cd catkin_ws; roslaunch hokuyo.launch" &
sleep 3; xterm -hold -e "cd catkin_ws; rosrun robot_setup_tf tf_broadcaster" &
sleep 4; xterm -hold -e "cd catkin_ws; rosrun rviz rviz" &
sleep 5; xterm -hold -e "cd catkin_ws; rosrun map_server map_server map7.yaml" &
sleep 6; xterm -hold -e "cd catkin_ws; roslaunch AMCL.launch" &
sleep 7; xterm -hold -e "cd catkin_ws; rostopic echo /amcl_pose"&
sleep 8; xterm -hold -e "cd catkin_ws; rosrun joy joy_node"&
sleep 9; xterm -hold -e "cd catkin_ws; rostopic echo joy"

