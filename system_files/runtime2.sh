#!/bin/bash
rosrun velma_common reset_shm_comm
roscore >>/dev/null &
sleep 5
roslaunch stero_velma offline_octo.launch &
sleep 10
roslaunch stero_velma rviz_gazebo.launch &
sleep 10
timeout 20 rosrun velma_task_cs_ros_interface initialize_robot.py
timeout 250 rosrun stero_velma moveObjectInv.py >> "$1"
pkill -P $$
wait
