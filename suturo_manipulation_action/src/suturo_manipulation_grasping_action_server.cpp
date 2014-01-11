#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_graspingAction.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_grasping.h>
#include <suturo_manipulation_planning_scene_interface.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_graspingAction> Server_grasp;

/**
* This method grasps or drops a object!
*/
void grop(const suturo_manipulation_msgs::suturo_manipulation_graspingGoalConstPtr& graspGoal, ros::NodeHandle* nh, Server_grasp* as)
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

	// Set arm to pick and object name
	string picking_arm = graspGoal->goal.arm;
	ROS_INFO("Arm to pick: %s", picking_arm.c_str());
	string obj_name = graspGoal->goal.objectName;
	ROS_INFO("ObjectName to pick: %s", obj_name.c_str());

	// Check if we should grasp or drop...
	if(graspGoal->goal.grasp){
		ROS_INFO("Begin to pick object...");
		// Grasp the object
		if (grasper.pick(obj_name, picking_arm))
		{
			ROS_INFO("Object picked...");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
	    as->setSucceeded(r);
		} else {
			ROS_INFO("Picking failed!");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		as->setAborted(r);
		}
	} else {
		ROS_INFO("Begin to drop object...");
		// Drop the object
		if (grasper.drop(obj_name))
		{
			ROS_INFO("Object droped...");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
	    as->setSucceeded(r);
		} else {
			ROS_INFO("Droping failed!");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		as->setAborted(r);
		}
	}


}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_grasp_server");
	ros::NodeHandle nh;
	
	Server_grasp server(nh, "suturo_man_grasping_server", boost::bind(&grop, _1, &nh, &server), false);
	server.start();
	
	ROS_INFO("Ready to move the head!");
	ros::spin();
	return 0;
}
