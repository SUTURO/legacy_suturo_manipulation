/**
* This class implements the action server to grasp/drop a object.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_graspingAction.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <suturo_manipulation_grasping.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>


using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_graspingAction> Server;

/**
* This method grasps or drops a object!
*/
void grop(const suturo_manipulation_msgs::suturo_manipulation_graspingGoalConstPtr& graspGoal, ros::NodeHandle* nh, Server* server_grasp)
{	
	suturo_manipulation_msgs::suturo_manipulation_graspingResult r;	
	
	r.succ.header.stamp = ros::Time();
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;

	// set Planning Interface and Grasper
	ROS_INFO("Create Planning Scene Interface...");
	Suturo_Manipulation_Planning_Scene_Interface pi(nh);
	ROS_INFO("Done. Create Grasper...")	;
	Grasping grasper(&pi);
	ROS_INFO("Done.");

  // if(graspGoal->goal.bodypart.bodyPart=="left_arm"){
  //   ROS_INFO("blabla");
  //   picking_arm=suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM;
  // }

	// Set arm to pick and object name
	string picking_arm = graspGoal->goal.bodypart.bodyPart;

  ROS_INFO("picking_arm: %s", picking_arm.c_str());
  ROS_INFO("goal.bodypart.bodyPart: %s", graspGoal->goal.bodypart.bodyPart.c_str());
  ROS_INFO("LEFT_ARM; %s", suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM.c_str());
  ROS_INFO("RIGHT_ARM; %s", suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM.c_str());

  if (picking_arm == suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM ){  
    picking_arm = "left_arm";
  } else if (picking_arm == suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM) {
    picking_arm = "right_arm";
  } else {
    ROS_INFO("Unknown arm! Please use suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM or suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM as names!");
    r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
    server_grasp->setAborted(r); 
    return; 
  }

	ROS_INFO("Arm to pick: %s", picking_arm.c_str());
	string obj_name = graspGoal->goal.objectName;
	ROS_INFO("ObjectName to pick: %s", obj_name.c_str());

  double newton = graspGoal->goal.newton;
  ROS_INFO("Newton: %f", newton);

	// Check if we should grasp or drop...
	if(graspGoal->goal.grasp){
		ROS_INFO("Begin to pick object...");
		// Grasp the object
		if (grasper.pick(obj_name, picking_arm, newton))
		{
			ROS_INFO("Object picked...");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
	    server_grasp->setSucceeded(r);
      return;
		} else {
			ROS_INFO("Picking failed!");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
  		server_grasp->setAborted(r);
      return;
		}
	} else {
		ROS_INFO("Begin to drop object...");
		// Drop the object
		if (grasper.drop(obj_name))
		{
			ROS_INFO("Object droped...");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
	    server_grasp->setSucceeded(r);
      return;
		} else {
			ROS_INFO("Droping failed!");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
  		server_grasp->setAborted(r);
      return;
		}
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_grasp_server");
	ros::NodeHandle nh;
	
	// Publishing the planning scene
	ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	
	Server server_grasp(nh, "suturo_man_grasping_server", boost::bind(&grop, _1, &nh, &server_grasp), false);
	server_grasp.start();
	

	ROS_INFO("Ready to grasp!!!");

	ros::spin();
	return 0;
}
