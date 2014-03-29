#!/bin/zsh
source $(rospack find suturo_manipulation_gazebo)/../../../devel/setup.zsh
export GAZEBO_MODEL_PATH=$(rospack find suturo_manipulation_gazebo)/models
roslaunch suturo_manipulation_gazebo gazebo_pr2_test_simulation_with_3_obj.launch
source ~/.zshrc

