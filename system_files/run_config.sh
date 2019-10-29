#!/bin/bash
cp velma_sim_gazebo/$1/velma_gazebo_re.launch /home/rgryta/ws/ws_velma_os/src/velma_sim_gazebo/launch/

cp velma_sim_gazebo/lwr_gazebo.cpp2 $HOME/ws/ws_velma_os/src/velma_sim_gazebo/src/lwr_gazebo.cpp

cd $HOME/ws/ws_velma_os

catkin clean
catkin build
