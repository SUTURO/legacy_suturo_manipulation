#include "suturo_manipulation_collision_checker.h"
#include "boost/thread.hpp"

void Collision_Checker::l_gripperCallback(const pr2_msgs::PressureState& msg)
{
  // update values
  for(int i = 0; i<22; i++){
    l_arm_r_finger_[i] = msg.r_finger_tip[i];
    l_arm_l_finger_[i] = msg.l_finger_tip[i];
    updated_ = true;
  }
}

void Collision_Checker::r_gripperCallback(const pr2_msgs::PressureState& msg)
{
  // update values
  for(int i = 0; i<22; i++)
  {
    r_arm_r_finger_[i] = msg.r_finger_tip[i];
    r_arm_l_finger_[i] = msg.l_finger_tip[i];
    updated_ = true;
  }
}

void Collision_Checker::clear()
{
  for(int i = 0; i<22; i++)
  {
    r_arm_r_finger_tara_[i] = r_arm_r_finger_[i];
    r_arm_l_finger_tara_[i] = r_arm_l_finger_[i];
    l_arm_r_finger_tara_[i] = l_arm_r_finger_[i];
    l_arm_l_finger_tara_[i] = l_arm_l_finger_[i];
  }
}

int Collision_Checker::r_collision()
{
  int result = 0;

  if (r_arm_l_finger_[3] - r_arm_l_finger_tara_[3] > 1000) 
    result = result + 1;

  if (r_arm_l_finger_[4] - r_arm_l_finger_tara_[4] > 1000) 
    result = result + 2;

  if (r_arm_r_finger_[3] - r_arm_r_finger_tara_[3] > 1000) 
    result = result + 4;

  if (r_arm_r_finger_[4] - r_arm_r_finger_tara_[4] > 1000) 
    result = result + 8;

  return result;
}

int Collision_Checker::l_collision()
{
  ROS_INFO("l_tara_3 %d", r_arm_l_finger_tara_[3]);
  ROS_INFO("l_tara_4 %d", r_arm_l_finger_tara_[4]);
  ROS_INFO("r_tara_3 %d", r_arm_r_finger_tara_[3]);
  ROS_INFO("r_tara_4 %d", r_arm_r_finger_tara_[4]);

  return r_arm_r_finger_[3];
}

Collision_Checker::Collision_Checker(ros::NodeHandle* nh)
{
  l_sub_ = nh->subscribe("/pressure/l_gripper_motor", 1000, &Collision_Checker::l_gripperCallback, this);

  r_sub_ = nh->subscribe("/pressure/r_gripper_motor", 1000, &Collision_Checker::r_gripperCallback, this);

  updated_ = false;
}

