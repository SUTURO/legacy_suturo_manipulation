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
        moveit_msgs::CollisionObject co,
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
  
  while(!cc_->updated_) 
  {
    ROS_WARN("Waiting for fingertippressuredata");
    ros::Duration(0.1).sleep();
  }
  // set tara for pressurevalues
  cc_->clear();
  ch_->reset();
  moveSucces_ = false;
  collisionDetected_ = false;

  // check that previous grasp failed 
  // and the max amount of attempts is not reached
  while(!moveSucces_ && ch_->attemptValid())
  {
    ROS_WARN("Start Grasping Attempt");
    // start threaded moving of the arm
    boost::thread* t = new boost::thread(boost::bind(&Grasping_reactive::threaded_move, this, move_group));
    int collisionValue;
    // check if grasp succeeded or gripper collided
    while(!moveSucces_ && !collisionDetected_)
    {
      // get collision-status
      collisionValue = cc_->r_collision();
      // collisionValue = 0; // TESTINGSTUB
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
      ROS_WARN("Calling collision handling");
      ch_->handleCollision(collisionValue, co);
      ROS_WARN("CollisionObject moved, returning to preGraspPosition");
      move_group->setPoseTarget(preGraspPose);
      move_group->move();
      move_group->setPoseTarget(desired_pose);
    } 
  }

  ROS_WARN("Grasping_reactive::move finished");
  if(moveSucces_)
    return 1;
  return 0;
}

void Grasping_reactive::threaded_move(move_group_interface::MoveGroup* move_group){
  moveSucces_ = move_group->move() ? 1 : 0;
}

