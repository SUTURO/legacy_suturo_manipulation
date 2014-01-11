#include <ros/ros.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/suturo_manipulation_graspingAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<suturo_manipulation_msgs::suturo_manipulation_graspingAction> Client;

using namespace std;


int main(int argc, char** argv)
{
	
	if (argc != 3)
	{
		ROS_INFO("arguments: box1/box2/beer1 right_arm/left_arm");
		return 1;
	}
	
	// Old Stuff!!!
	//~ string frame = argv[5];
	//~ if (frame == "home"){
		//~ ros::init(argc, argv, "test_action_client");
		//~ Client_home client("home_action_server", true); 
		//~ client.waitForServer();
		//~ 
		//~ suturo_manipulation_msgs::suturo_manipulation_homeGoal goal;
		//~ //goal.p.frame_id = frame;
		//~ 
		//~ goal.arm = argv[1];
		//~ client.sendGoal(goal);
		//~ 
		//~ 
		//~ //2x da es sonst nicht geht...
		//~ client.waitForResult(ros::Duration(20.0));
		//~ client.waitForResult(ros::Duration(20.0));
		//~ suturo_manipulation_msgs::suturo_manipulation_homeResultConstPtr r = client.getResult();
		//~ 
		//~ if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			//~ if (suturo_manipulation_msgs::ActionAnswer::SUCCESS == r->succ.type){
				//~ ROS_INFO("moved!!!!!!! result: %i", r->succ.type);
			//~ }else{
				//~ ROS_INFO("not moved!!!!!!! result: %i", r->succ.type);
			//~ }
			//~ 
		//~ }
		//~ printf("Current State: %s\n", client.getState().toString().c_str());
	//~ } else {
	//~ 
		//~ suturo_manipulation_msgs::suturo_manipulation_moveGoal goal;
		//~ goal.p.frame_id = frame;
		
		ros::init(argc, argv, "test_action_pick_client");
		Client client("suturo_man_grasping_server", true); // true -> don't need ros::spin()
		client.waitForServer();
		ROS_INFO("Connected to server, ready to pick");
		
		suturo_manipulation_msgs::suturo_manipulation_graspingGoal goal;
		goal.goal.header.seq = 0;
		goal.goal.header.stamp = ros::Time();
		goal.goal.header.frame_id = "/base_footprint";
		goal.goal.objectName = argv[1];
		goal.goal.grasp = 1;
		goal.goal.arm = argv[2];
		client.sendGoal(goal);
		client.waitForResult(ros::Duration(20.0));
		
		
		
		
		// Old Stuff!!!
		//~ goal.arm = argv[1];
		//~ goal.p.c_centroid.x = atof(argv[2]);
		//~ goal.p.c_centroid.y = atof(argv[3]);
		//~ goal.p.c_centroid.z = atof(argv[4]);
		//~ client.sendGoal(goal);
		//~ //2x da es sonst nicht geht...
		//~ client.waitForResult(ros::Duration(20.0));
		//~ client.waitForResult(ros::Duration(20.0));
		//~ suturo_manipulation_msgs::suturo_manipulation_moveResultConstPtr r = client.getResult();
		
		suturo_manipulation_msgs::suturo_manipulation_graspingResultConstPtr r = client.getResult();
		
		if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			if (suturo_manipulation_msgs::ActionAnswer::SUCCESS == r->succ.type){
				ROS_INFO("moved!!!!!!! result: %i", r->succ.type);
			}else{
				ROS_INFO("not moved!!!!!!! result: %i", r->succ.type);
			}
			
		}
		printf("Current State: %s\n", client.getState().toString().c_str());
	//~ }
	return 0;
}
