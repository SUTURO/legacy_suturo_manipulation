#!/bin/bash
source ~/.bashrc
roscd suturo_manipulation_gazebo
export GAZEBO_MODEL_PATH=$(pwd)/models
roslaunch suturo_manipulation_gazebo gazebo_pr2_test_simulation_with_3_obj.launch

