#!/bin/bash
rosrun velma_common reset_shm_comm
roscore >>/dev/null &
sleep 10
roslaunch stero_velma offline_octo.launch &
sleep 15
roslaunch stero_velma cab_rviz_gazebo.launch &
sleep 15
rosrun velma_task_cs_ros_interface initialize_robot.py
rosrun stero_velma moveObject.py >> "$1"
pkill -P $$
wait
