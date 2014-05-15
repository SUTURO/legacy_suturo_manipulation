#include "suturo_manipulation_collision_handler.h"
#include <visualization_msgs/Marker.h>

Collision_Handler::Collision_Handler(ros::NodeHandle* nh, int maxAttempts, Suturo_Manipulation_Planning_Scene_Interface* pi)
{
  collisionValues_ = new int[maxAttempts];
  pi_ = pi;
  maxAttempts_ = maxAttempts;
  listener_ = new tf::TransformListener();
  vis_pub_ = nh->advertise<visualization_msgs::Marker>( "/suturo/visualization_marker", 0 );
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
  ROS_WARN_STREAM("collisionValues " << collisionValues_[0] << ", "  
      << collisionValues_[1] << ", " << collisionValues_[2] << endl);

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
      ROS_WARN_STREAM("yDiff: " << yDiff << endl);
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

  // create new poseStamped with data from CollisionObject
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = co.id;
  pose.header.stamp = co.header.stamp;

  // check if position is in primitive_pose or mesh_pose
  if(co.primitive_poses.size() == 1)
  {
    pose.pose.orientation.w = 1;
    // ROS_WARN("Going to publish the marker BEFORE transformation, press any key and enter to continue");
    // std::string input;
    // std::cin >> input;
    publishMarker(pose);
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
    co.header.stamp = ros::Time(0);

    // pose for marker
    pose.pose.position.y += yDiff;
    pose.pose.position.z += zDiff;
    // ROS_WARN("Going to publish the marker AFTER transformation, press any key and enter to continue");
    // std::cin >> input;
    // publishMarker(pose);
  }
  // position in mesh_pose
  else
  {
    pose.pose.orientation.w = 1;
  pose.header.frame_id = co.id;
  pose.header.stamp = co.header.stamp;
    // ROS_WARN("Going to publish the marker BEFORE transformation, press any key and enter to continue");
    // std::string input;
    // std::cin >> input;
    publishMarker(pose);
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
    co.header.stamp = ros::Time(0);

    // pose for marker
    pose.pose.position.y += yDiff;
    pose.pose.position.z += zDiff;
    // ROS_WARN("Going to publish the marker AFTER transformation, press any key and enter to continue");
    // std::cin >> input;
    // publishMarker(pose);
  }

  // publish moved collisionObject
  pi_->addObject(co);
}

/**
 * This method publishes the goal as a marker for rviz
 */
 void Collision_Handler::publishMarker(geometry_msgs::PoseStamped pose)
 {
// Publish the Goalmarker
  visualization_msgs::Marker goal_marker;
  goal_marker.header = pose.header;
  goal_marker.ns = "suturo_manipulation";
  goal_marker.id = 0;
  goal_marker.type = visualization_msgs::Marker::SPHERE;
  goal_marker.action = visualization_msgs::Marker::ADD;
  goal_marker.pose = pose.pose;
  goal_marker.scale.x = 0.1;
  goal_marker.scale.y = 0.1;
  goal_marker.scale.z = 0.1;
  goal_marker.color.a = 1.0;
  goal_marker.color.r = 0.0;
  goal_marker.color.g = 1.0;
  goal_marker.color.b = 0.0;
  vis_pub_.publish( goal_marker );   
}
