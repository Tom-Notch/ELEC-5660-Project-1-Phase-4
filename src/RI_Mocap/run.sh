#!/bin/bash

roslaunch mavros px4.launch &
sleep 6
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 &
sleep 1
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 &
sleep 1
roslaunch vrpn_client_ros sample.launch server:=10.1.1.198 &
sleep 3
roslaunch ekf nokov.launch &
sleep 3
roslaunch odom_visualization odom_visualization.launch &
sleep 2
#roslaunch px4ctrl run_ctrl.launch & sleep 2;
#roslaunch traj_server traj_server.launch & sleep 3;
#roslaunch traj_server traj_server.launch & sleep 3;
wait
