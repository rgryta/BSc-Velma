#!/bin/bash
cd $HOME/velma/src/stero_velma/system_files

cp velma_robot\ -\ velma_description\ -\ robots/$1/velma.urdf.xacro $HOME/ws/ws_velma_os/src/velma_robot/velma_description/robots/

cp velma_sim_gazebo/$1/velma_gazebo_re.launch /home/rgryta/ws/ws_velma_os/src/velma_sim_gazebo/launch/

cp velma_sim_gazebo/lwr_gazebo.cpp2 $HOME/ws/ws_velma_os/src/velma_sim_gazebo/src/lwr_gazebo.cpp

cd $HOME/ws/ws_velma_os

catkin clean -y
sudo rm /opt/ros/melodic/share/cmake_modules/cmake/Modules/FindUUID.cmake
catkin build
