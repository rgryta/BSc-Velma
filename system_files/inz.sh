#!/bin/bash
for i in {1..10}
do
   start=$SECONDS
   rosrun velma_common reset_shm_comm
   timeout 340s roscore &
   sleep 10
   timeout 330 roslaunch stero_velma offline_octo.launch &
   sleep 30
   timeout 300 roslaunch stero_velma rviz_gazebo.launch &
   sleep 30
   timeout 20 rosrun velma_task_cs_ros_interface initialize_robot.py
   timeout 250 rosrun stero_velma moveObject.py >> "${i}.txt"
   end=$SECONDS
   tosleep=$(( 400 - end + start ))
   sleep $tosleep
   pkill -P $$
done
clear
echo "1st PART done"
for i in {1..10}
do
   start=$SECONDS
   rosrun velma_common reset_shm_comm
   timeout 340s roscore &
   sleep 10
   timeout 330 roslaunch stero_velma offline_octo.launch &
   sleep 30
   timeout 300 roslaunch stero_velma rviz_gazebo.launch &
   sleep 30
   timeout 20 rosrun velma_task_cs_ros_interface initialize_robot.py
   timeout 250 rosrun stero_velma moveObjectInv.py >> "${i} - inverted.txt"
   end=$SECONDS
   tosleep=$(( 400 - end + start ))
   sleep $tosleep
   pkill -P $$
done
clear
echo "2nd part done"
