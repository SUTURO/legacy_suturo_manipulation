#include "suturo_manipulation_collision_handler.h"

Collision_Handler::Collision_Handler(ros::NodeHandle* nh, int maxAttempts, Suturo_Manipulation_Planning_Scene_Interface* pi)
{
  collisionValues_ = new int[maxAttempts];
  pi_ = pi;
  maxAttempts_ = maxAttempts;
  listener_ = new tf::TransformListener();
}

void Collision_Handler::reset(bool rightArm)
{
  for(int i = 0; i < maxAttempts_; i++)
    collisionValues_[i] = 0;
  attempt_ = 0;
  rightArm_ = rightArm;
}

void Collision_Handler::handleCollision(int collisionValue, moveit_msgs::CollisionObject& co)
{
  attempt_++;

  // Add CollisionValue to the list
  int i = 0;
  while(collisionValues_[i] != 0)
    i++;
  collisionValues_[i] = collisionValue;

  ROS_WARN("Calling checkForPreviousCollision");
  // React to collision
  switch ( collisionValue )
  {
    case 1:
      // {
      //   // +y and +z in r_finger_tip_link
      //   checkForPreviousCollision(1, 1, co);
      // break;
      // }
    case 2:
      // {
      //   // +y and -z in r_finger_tip_link
      //   checkForPreviousCollision(1, -1, co);
      // break;
      // }
    case 3:
      {
        // +y in r_finger_tip_link
        checkForPreviousCollision(1, 0, co);
        break;
      }
      // case 4:
      //   {
      //     // -y and +z in r_finger_tip_link
      //     checkForPreviousCollision(-1, 1, co);
      // break;
      //   }
    case 5:
      {
        // +z in r_finger_tip_link
        checkForPreviousCollision(0, 1, co);
        break;
      }
      // case 8:
      //   {
      //     // -y and -z in r_finger_tip_link
      //     checkForPreviousCollision(-1, -1, co);
      // break;
      //   }
    case 10:
      {
        // -z in r_finger_tip_link
        checkForPreviousCollision(0, -1, co);
        break;
      }
    case 4:
    case 8:
    case 12:
      {
        // -y in r_finger_tip_link
        checkForPreviousCollision(-1, 0, co);
        break;
      }
    case 15:
      {
        // object too big oder collision with table
      }
  }
}

bool Collision_Handler::attemptValid()
{
  if(attempt_ < maxAttempts_)
    return true;
  return false;
}

void Collision_Handler::checkForPreviousCollision(int yValue, int zValue, moveit_msgs::CollisionObject& co)
{
  ROS_WARN("Calculating new Position for CollisionObject");
  // check for previous collisions and calculate diff
  double yDiff = 0;
  double zDiff = 0;
  if(yValue != 0)
  {
    for(int i = 0; i < maxAttempts_; i++)
    {
      if(collisionValues_[i] == 0)
      {
        break;
      }
      else if (collisionValues_[i] == 1 
          || collisionValues_[i] == 2
          || collisionValues_[i] == 3)
      {
        yDiff = 0.05 * (1.0/(i+1));
      }
      else if (collisionValues_[i] == 4
          || collisionValues_[i] == 8
          || collisionValues_[i] == 12)
      {
        yDiff = -0.05 * (1.0/(i+1));
      }
    }
  }
  if(zValue != 0)
  {
    for(int i = 0; i < maxAttempts_; i++)
    {
      if(collisionValues_[i] == 0)
      {
        break;
      }
      else if (collisionValues_[i] == 1 
          || collisionValues_[i] == 4
          || collisionValues_[i] == 5)
      {
        zDiff = 0.05 * (1.0/(i+1));
      }
      else if (collisionValues_[i] == 2
          || collisionValues_[i] == 8
          || collisionValues_[i] == 10)
      {
        zDiff = -0.05 * (1.0/(i+1));
      }
    }
  }

  ROS_WARN("Finished Calculating, start to transform with tf");

  // create new poseStamped with data from CollisionObject
  geometry_msgs::PoseStamped pose;
  pose.header = co.header;

  // check if position is in primitive_pose or mesh_pose
  if(co.primitive_poses.size() == 1)
  {
    pose.pose.position = co.primitive_poses[0].position;
    pose.pose.orientation = co.primitive_poses[0].orientation;
    try{
      if(rightArm_)
      {
        listener_->waitForTransform("/r_gripper_r_finger_tip_link", pose.header.frame_id, ros::Time(0), ros::Duration(3));
        listener_->transformPose("/r_gripper_r_finger_tip_link", ros::Time(0), pose, co.header.frame_id, pose);
      }
      else
      {
        listener_->waitForTransform("/l_gripper_r_finger_tip_link", pose.header.frame_id, ros::Time(0), ros::Duration(3));
        listener_->transformPose("/l_gripper_r_finger_tip_link", ros::Time(0), pose, co.header.frame_id, pose);
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    // set new frame in header
    co.header.frame_id = pose.header.frame_id;
    co.primitive_poses[0] = pose.pose;
    // apply corrections
    co.primitive_poses[0].position.y += yDiff;
    co.primitive_poses[0].position.z += zDiff;
  }
  // position in mesh_pose
  else
  {
    pose.pose.position = co.mesh_poses[0].position;
    pose.pose.orientation = co.mesh_poses[0].orientation;
    try{
      if(rightArm_)
      {
        listener_->waitForTransform("/r_gripper_r_finger_tip_link", pose.header.frame_id, ros::Time(0), ros::Duration(3));
        listener_->transformPose("/r_gripper_r_finger_tip_link", ros::Time(0), pose, co.header.frame_id, pose);
      }
      else
      {
        listener_->waitForTransform("/l_gripper_r_finger_tip_link", pose.header.frame_id, ros::Time(0), ros::Duration(3));
        listener_->transformPose("/l_gripper_r_finger_tip_link", ros::Time(0), pose, co.header.frame_id, pose);
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    // set new frame in header
    co.header.frame_id = pose.header.frame_id;
    co.mesh_poses[0] = pose.pose;
    // apply corrections
    co.mesh_poses[0].position.y += yDiff;
    co.mesh_poses[0].position.z += zDiff;
  }

  ROS_WARN("Finished Transforming, going to publish CollisionObject");

  // publish moved collisionObject
  pi_->addObject(co);
}
