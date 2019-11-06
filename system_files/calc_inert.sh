#!/bin/bash
cp velma_robot\ -\ velma_description\ -\ robots/$1/velma.urdf.xacro $HOME/ws/ws_velma_os/src/velma_robot/velma_description/robots/

cp velma_sim_gazebo/lwr_gazebo.cpp1 $HOME/ws/ws_velma_os/src/velma_sim_gazebo/src/lwr_gazebo.cpp

cd $HOME/ws/ws_velma_os

catkin clean
catkin build
