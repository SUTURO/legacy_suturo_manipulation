#include "pr2_msgs/PressureState.h";
#include "ros/ros.h";

void gripperCallback(const pr2_msgs::PressureState& msg)
{
  ROS_INFO("Pressuremessage arrived");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "suturo_manipulation_reactive_grasping");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/pressure/r_gripper_motor", 1000, gripperCallback);

  ros::spin();
}

