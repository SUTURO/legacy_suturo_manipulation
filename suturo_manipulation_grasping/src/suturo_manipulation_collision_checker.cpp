#include "suturo_manipulation_collision_checker.h"

void gripperCallback(const pr2_msgs::PressureState& msg)
{
  ROS_INFO("Pressuremessage arrived");
}

Collision_Checker::Collision_Checker(ros::NodeHandle nh)
{
  /*
  ros::init(argc, argv, "suturo_manipulation_reactive_grasping");

  ros::Subscriber sub = nh.subscribe("/pressure/r_gripper_motor", 1000, gripperCallback);

  ros::spin();
  */
}

