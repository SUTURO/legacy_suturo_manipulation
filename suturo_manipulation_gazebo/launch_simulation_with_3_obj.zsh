#!/bin/zsh
GROOVY_PATH=$(rospack find suturo_manipulation_gazebo)/../../../devel
source $(cd $GROOVY_PATH && pwd)/setup.bash
export GAZEBO_MODEL_PATH=$(rospack find suturo_manipulation_gazebo)/models
roslaunch suturo_manipulation_gazebo gazebo_pr2_test_simulation_with_3_obj.launch
source ~/.zshrc

