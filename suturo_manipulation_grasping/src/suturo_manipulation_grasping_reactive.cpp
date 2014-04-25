#include "suturo_manipulation_grasping_reactive.h"
#include <suturo_manipulation_collision_checker.h>

using namespace std;

Grasping_reactive::Grasping_reactive(ros::NodeHandle* nh, Suturo_Manipulation_Planning_Scene_Interface* pi, ros::Publisher* head_publisher) : Grasping(nh, pi, head_publisher) {
  cc_ = new Collision_Checker(nh);
  moveSucces_ = false;
}

int Grasping_reactive::move(move_group_interface::MoveGroup* move_group, geometry_msgs::PoseStamped desired_pose)
{
  ROS_INFO("Grasping_reactive::move called");
  // this->Grasping::move(move_group, desired_pose);
  //look
  lookAt(desired_pose);
  //set marker
  pi_->publishMarker(desired_pose);
  //set goal
  move_group->setPoseTarget(desired_pose);
  //move
  // return move_group->move() ? 1 : 0;

  // boost::thread* thr = new boost::thread(boost::bind(&move_group_interface::MoveGroup::move, move_group));

  boost::thread* t = new boost::thread(boost::bind(&Grasping_reactive::threaded_move, this, move_group));

  int cnt=0;
  while(!moveSucces_ && cnt < 30){
    cout << "checking collision" << endl;
    ros::Duration(0.1).sleep();
    cnt++;
  }

  move_group->stop();

  t->join();
  ROS_INFO("Grasping_reactive::move finished");
  return 0;
}

void Grasping_reactive::threaded_move(move_group_interface::MoveGroup* move_group){
  moveSucces_ = move_group->move() ? 1 : 0;
}

