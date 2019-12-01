#!/bin/bash
rosrun velma_common reset_shm_comm
roscore >>/dev/null &
sleep 5
roslaunch stero_velma offline_octo.launch &
sleep 20
roslaunch stero_velma cab_rviz_gazebo.launch &
sleep 25
rosrun velma_task_cs_ros_interface initialize_robot.py
rosrun stero_velma moveObjectInv.py >> "$1"
pkill -P $$
wait
