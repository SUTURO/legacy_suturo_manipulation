
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
     
