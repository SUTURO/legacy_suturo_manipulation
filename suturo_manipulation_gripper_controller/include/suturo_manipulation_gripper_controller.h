#ifndef SUTURO_MANIPULATION_GRIPPER_CONTROLLER
#define SUTURO_MANIPULATION_GRIPPER_CONTROLLER

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;


class Gripper{
private:
  GripperClient* r_gripper_client_;  
  GripperClient* l_gripper_client_;  

public:
  static const double GRIPPER_MAX_POSITION = 0.09;
  static const double GRIPPER_MIN_POSITION = 0.0;
  static const double GRIPPER_DEPTH = 0.1;
  
  //Action client initialization
  Gripper();

  ~Gripper();

  //Open the gripper
  actionlib::SimpleClientGoalState open_r_gripper();
  
  actionlib::SimpleClientGoalState open_l_gripper();

  //Close the gripper
  actionlib::SimpleClientGoalState close_r_gripper();
  
  actionlib::SimpleClientGoalState close_l_gripper();
};
     
#endif
