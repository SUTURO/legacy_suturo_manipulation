#include "suturo_manipulation_grasping_reactive.h"

using namespace std;

Grasping_reactive::Grasping_reactive(ros::NodeHandle* nh, 
    Suturo_Manipulation_Planning_Scene_Interface* pi, 
    ros::Publisher* head_publisher) : Grasping(nh, pi, head_publisher) 
{
  cc_ = new Collision_Checker(nh);
  // Collision_Handler for three tries
  ch_ = new Collision_Handler(nh, 3, pi);
}

int Grasping_reactive::move(move_group_interface::MoveGroup *move_group, 
        geometry_msgs::PoseStamped desired_pose,
        moveit_msgs::CollisionObject& co,
        geometry_msgs::PoseStamped preGraspPose)

{
  ROS_WARN("Grasping_reactive::move called");
  // ROS_INFO("Arm: %s", move_group->getName().c_str());
  //look
  lookAt(desired_pose);
  //set marker
  pi_->publishMarker(desired_pose);
  //set goal
  move_group->setPoseTarget(desired_pose);

  bool rightArm;

  //get arm to move
  if(move_group->getName().compare("right_arm") == 0)
    rightArm = true;
  else
    rightArm = false;

  while(!cc_->updated_) 
  {
    ROS_WARN("Waiting for fingertippressuredata");
    ros::Duration(0.1).sleep();
  }
  // set tara for pressurevalues
  cc_->clear();
  ch_->reset(rightArm);
  moveSuccess_ = false;
  collisionDetected_ = false;

  // check that previous grasp failed 
  // and the max amount of attempts is not reached
  while(!moveSuccess_ && ch_->attemptValid())
  {
    ROS_WARN("Start Grasping Attempt");
    // start threaded moving of the arm
    boost::thread* t = new boost::thread(boost::bind(&Grasping_reactive::threaded_move, this, move_group));
    int collisionValue;
    collisionDetected_ = false;
    // check if grasp succeeded or gripper collided
    while(!moveSuccess_ && !collisionDetected_)
    {
      // get collision-status
      if (rightArm)
        collisionValue = cc_->r_collision();
      else 
        collisionValue = cc_->l_collision();
      if(collisionValue > 0) 
      {
        collisionDetected_ = true;
        ROS_WARN("Fingertipcollision detected: %d", collisionValue);
        move_group->stop();
        ros::Duration(0.1).sleep();
      }
    }
    // wait for threaded move_group to be finished
    ROS_WARN("wait for threaded move_group to be finished");
    t->join();
    if(collisionDetected_)
    {
      // Create new Pose to back up from collision
      geometry_msgs::PoseStamped newPreGraspPose;
      if(rightArm)
        newPreGraspPose.header.frame_id = "/r_wrist_roll_link";
      else 
        newPreGraspPose.header.frame_id = "/l_wrist_roll_link";
      newPreGraspPose.header.stamp = ros::Time::now();
      newPreGraspPose.pose.position.x -= Gripper::GRIPPER_DEPTH;
      newPreGraspPose.pose.orientation.w = 1;
      move_group->setPoseTarget(newPreGraspPose);
      move_group->move();

      // move collisionObject in PlanningScene
      ch_->handleCollision(collisionValue, co);
      // preGraspPose.pose = co.pose;
      // preGraspPose.header = co.header;
      // preGraspPose.pose.position.x -= Gripper::GRIPPER_DEPTH - 0.05;
      ros::Duration(2).sleep();
      // move to new preGraspPose
      move_group->setPoseTarget(preGraspPose);
      move_group->move();
      // Set new CollisionObject as target
      move_group->setPoseTarget(desired_pose);
    } 
  }

  ROS_WARN("Grasping_reactive::move finished");
  if(moveSuccess_)
    return 1;
  return 0;
}

void Grasping_reactive::threaded_move(move_group_interface::MoveGroup* move_group){
  moveSuccess_ = move_group->move() ? 1 : 0;
}

