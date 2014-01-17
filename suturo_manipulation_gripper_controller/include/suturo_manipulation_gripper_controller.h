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
  double n;

public:
  static const double GRIPPER_MAX_POSITION = 0.09;
  static const double GRIPPER_MIN_POSITION = 0.0;
  static const double GRIPPER_DEPTH = 0.14;
  //Action client initialization
  Gripper();

  ~Gripper();
 
	actionlib::SimpleClientGoalState close_gripper(GripperClient* gripper_client_);

	actionlib::SimpleClientGoalState open_gripper(GripperClient* gripper_client_);

  //Open the gripper
  actionlib::SimpleClientGoalState open_r_gripper(double force=-1);
  
  actionlib::SimpleClientGoalState open_l_gripper(double force=-1);

  //Close the gripper
  actionlib::SimpleClientGoalState close_r_gripper(double force=50);
  
  actionlib::SimpleClientGoalState close_l_gripper(double force=50);
  
  static const std::vector<std::string> get_r_gripper_links() {
    static std::vector<std::string> r2;
    r2.push_back("r_gripper_l_finger_link");
		r2.push_back("r_gripper_l_finger_tip_link");
		r2.push_back("r_gripper_motor_accelerometer_link");
		r2.push_back("r_gripper_palm_link");
		r2.push_back("r_gripper_r_finger_link");
		r2.push_back("r_gripper_r_finger_tip_link");
		static const std::vector<std::string> r = r2;
    return r;
  } 

  static const std::vector<std::string> get_l_gripper_links() {
    static std::vector<std::string> r2;
    r2.push_back("l_gripper_l_finger_link");
		r2.push_back("l_gripper_l_finger_tip_link");
		r2.push_back("l_gripper_motor_accelerometer_link");
		r2.push_back("l_gripper_palm_link");
		r2.push_back("l_gripper_r_finger_link");
		r2.push_back("l_gripper_r_finger_tip_link");
		static const std::vector<std::string> r = r2;
    return r;
  } 
  
};
     
#endif
