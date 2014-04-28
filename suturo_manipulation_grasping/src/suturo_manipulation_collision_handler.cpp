#include "suturo_manipulation_collision_handler.h"

Collision_Handler::Collision_Handler(ros::NodeHandle* nh, int maxAttempts, Suturo_Manipulation_Planning_Scene_Interface* pi)
{
  collisionValues_ = new int[maxAttempts];
  pi = pi_;
}

void Collision_Handler::reset()
{
  for(int i = 0; i < maxAttempts_; i++)
    collisionValues_[i] = 0;
  attempt_ = 0;
  tf::StampedTransform transform;
  listener_ = new tf::TransformListener();
  listener_->waitForTransform("/r_gripper_r_finger_tip_link", "/r_gripper_l_finger_tip_link", ros::Time(0), ros::Duration(3.0));
  try{
    listener_->lookupTransform("/r_gripper_r_finger_tip_link", "/r_gripper_l_finger_tip_link",
        ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  ROS_INFO("x: %f, y: %f",transform.getOrigin().x(),  transform.getOrigin().y());
}

void Collision_Handler::handleCollision(int collisionValue, moveit_msgs::CollisionObject& co)
{
    /*
     * co->position->...
     * pi->addObject(co);
     */
 }

bool Collision_Handler::attemptValid()
{
  if(attempt_ < maxAttempts_)
    return true;
  return false;
}
