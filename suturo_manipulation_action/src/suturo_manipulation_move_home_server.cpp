/**
* This class implements the action server to move a
* part of the robot (left_arm, right_arm, head)
* to a definied home position.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_homeAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_homeAction> Server;

/**
* This method moves the given bodypart to the homeposition.
*/
void moveHome(const suturo_manipulation_msgs::suturo_manipulation_homeGoalConstPtr& goal, ros::Publisher* publisher, Server* server_home)
{	
	ROS_INFO("callback moveHome() begins...");

	// create a move result message
	suturo_manipulation_msgs::suturo_manipulation_homeResult r;	
	// Set part which should be go home
	string body_part = goal->bodypart;
	// Set header
	r.succ.header.stamp = ros::Time();
	// Set Answer fot planning to undefined
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;
	
	if(body_part=="head"){
		// Create headHome object
		geometry_msgs::PoseStamped headHome;

		// Set home Coordinates, time and frame
		headHome.pose.position.x = 1;
		headHome.pose.position.y = 0;
		headHome.pose.position.z = 0;
		headHome.header.stamp = ros::Time::now();
		headHome.header.frame_id = "/torso_lift_link";

        // Publish goal on topic /suturo/head_controller
        if( !publisher ) {
           ROS_WARN("Publisher invalid!");
           server_home->setAborted(r);
        } else {
            publisher->publish(headHome);
            ROS_INFO("Home Goal published!");
            server_home->setSucceeded(r);
        }
	} else {
		// set group to move
		move_group_interface::MoveGroup group(body_part);
		
		// set group name to go home
		group.setNamedTarget(body_part+"_home");
		
		//move bodypart
		if (group.move()){
		    r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
		    server_home->setSucceeded(r);
		} else {
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
			server_home->setAborted(r);
		}
	}	
	ROS_INFO("moved: %i", r.succ.type);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_move_home_server");
	ros::NodeHandle n;

	// Publish a topic for the head controller
    ros::Publisher head_publisher = n.advertise<geometry_msgs::PoseStamped>("/suturo/head_controller_goal_point", 1000);

	// create the action server
	Server server_home(n, "suturo_man_move_home_server", boost::bind(&moveHome, _1, &head_publisher, &server_home), false);
	// start the server
	server_home.start();
	
	ROS_INFO("Ready to go home!");
	ros::spin();
	return 0;
}
