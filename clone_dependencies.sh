#!/bin/bash

git clone https://github.com/catkin/catkin_simple ../catkin_simple
git clone https://github.com/ethz-asl/mav_comm ../mav_comm
git clone https://github.com/OctoMap/octomap_msgs ../octomap_msgs
git clone https://github.com/cirpote/mav_control_rw ../mav_control_rw
git clone https://github.com/cirpote/rotors_simulator ../rotors_simulator
cd ../rotors_simulator
git checkout feature/gazebo9
cd ../rvb_mpc

