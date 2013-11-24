#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_moveAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_moveAction> Server;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "suturo_manipulation_move_head");
  ros::NodeHandle n;
  
  if (argc != 3)
  {
    ROS_INFO("usage: tilt pitch");
    return 1;
  }
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  move_group_interface::MoveGroup group("head");
  vector<double> p;
  p.push_back(atof(argv[1]));
  p.push_back(atof(argv[2]));
  group.setJointValueTarget(p);
  group.move();

  ros::waitForShutdown();
  return 0;
}
