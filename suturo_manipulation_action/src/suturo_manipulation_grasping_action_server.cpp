#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_graspingAction.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_grasping.h>
#include <suturo_manipulation_planning_scene_interface.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_graspingAction> Server_grasp;

void grasp(const suturo_manipulation_msgs::suturo_manipulation_graspingGoalConstPtr& goal, ros::NodeHandle* nh, Server_grasp* as)
{	
	suturo_manipulation_msgs::suturo_manipulation_graspingResult r;	
	
	r.succ.header.stamp = ros::Time();
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;
	
	//0 für fail

	// set Planning Interface and Grasper
	// Suturo_Manipulation_Planning_Scene_Interface pi(nh);
	// Grasping grasper(&pi);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_grasp_server");
	ros::NodeHandle nh;
	
	Server_grasp server(nh, "suturo_man_grasping_server", boost::bind(&grasp, _1, &nh, &server), false);
	server.start();
	
	ROS_INFO("Ready to move the head!");
	ros::spin();
	return 0;
}
