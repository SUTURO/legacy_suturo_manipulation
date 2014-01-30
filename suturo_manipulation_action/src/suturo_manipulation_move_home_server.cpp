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
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <tf/transform_listener.h>
#include <suturo_manipulation_msgs/torque_values.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_homeAction> Server;

// TF Listener...
tf::TransformListener* listener = NULL;

double yaw_error, pitch_error;

// Transform the incoming frame to /torso_lift_link for the head move controller
int transform(geometry_msgs::PoseStamped &goalPose,
					geometry_msgs::Point goalPoint, const char* s)
{ 
	//save goal position in pose
	goalPose.header.frame_id = s;
	goalPose.pose.position.x = goalPoint.x;
	goalPose.pose.position.y = goalPoint.y;
	goalPose.pose.position.z = goalPoint.z;
	goalPose.pose.orientation.x = 0;
	goalPose.pose.orientation.y = 0;
	goalPose.pose.orientation.z = 0;
	goalPose.pose.orientation.w = 1;
	
	// goal_frame
  const string goal_frame = "/torso_lift_link";

	ROS_INFO("Begin transformation");
  try{
		//transform pose from s to -torso_lift_link and save it in pose again
		listener->transformPose(goal_frame, goalPose, goalPose);
	}catch(...){
		ROS_INFO("ERROR: Transformation failed.");
		return 0;
	}

    return 1;
}

/**
* This method moves the given bodypart to the homeposition.
*/
void moveHome(const suturo_manipulation_msgs::suturo_manipulation_homeGoalConstPtr& goal, 
			ros::Publisher* publisher, Server* server_home)
{	
	ROS_INFO("callback moveHome() begins...");

	// create a move result message
	suturo_manipulation_msgs::suturo_manipulation_homeResult r;	
	// Set part which should be go home
	string body_part = goal->bodypart.bodyPart;
	ROS_INFO("Bodypart: %s", body_part.c_str());
	// Set header
	r.succ.header.stamp = ros::Time();
	// Set Answer for planning to undefined
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;
	
	if(body_part==suturo_manipulation_msgs::RobotBodyPart::HEAD){
		// bodypart = head
		ROS_INFO("Move head home!");
		// Create headHome object
		geometry_msgs::PoseStamped headHome;

		// Set home Coordinates, time and frame
		headHome.pose.position.x = 1;
		headHome.pose.position.y = 0;
		headHome.pose.position.z = 0;
		headHome.header.stamp = ros::Time::now();
		headHome.header.frame_id = "/base_link";

		geometry_msgs::PoseStamped transformedPose;

		//transform pose
		if (!transform(transformedPose, headHome.pose.position, headHome.header.frame_id.c_str())){
			// If tranfsormation fails, update the answer for planning to "FAIL" and set the server aborted
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
			server_home->setAborted(r);
		}

        // Publish goal on topic /suturo/head_controller_goal_point
        if( !publisher ) {
        	ROS_INFO("Publisher invalid!\n");
        	r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
          	server_home->setAborted(r);
        } else {
            publisher->publish(transformedPose);
            ROS_INFO("Home Goal published!\n");

            int counter = 0;
            int duration = 0;
			
			// Checking of the goal is reached
			while (duration < 3 && counter < 15){
				if (yaw_error < 5 && pitch_error < 1){
					duration++;
					ROS_INFO_STREAM("Duration Home Server: " <<  duration);
				} else {
					duration = 0;
					ROS_INFO("Over Treshold home Server");
				}
				ros::Duration(1).sleep();
				counter++;
			}
			if(counter >= 15){
				r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
				server_home->setAborted(r);
			} else {
				r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
				server_home->setSucceeded(r);			
			}
        }
	} else if ((body_part==suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM)){
		// bodypart = left_arm
		ROS_INFO("Move left_arm home!");
		// set group to move
		move_group_interface::MoveGroup group(body_part);
		
		// set group name to go home
		group.setNamedTarget(body_part+"_home");
		
		//move bodypart
		if (group.move()){
			ROS_INFO("Moved home!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
			server_home->setSucceeded(r);
		} else {
			ROS_INFO("Moving home failed!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
			server_home->setAborted(r);
		}
	} else if ((body_part==suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM)){
		// bodypart = right_arm
		ROS_INFO("Move right_arm home!");
		// set group to move
		move_group_interface::MoveGroup group(body_part);
		
		// set group name to go home
		group.setNamedTarget(body_part+"_home");
		
		//move bodypart
		if (group.move()){
			ROS_INFO("Moved home!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
			server_home->setSucceeded(r);
		} else {
			ROS_INFO("Moving home failed!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
			server_home->setAborted(r);
		}
	} else if ((body_part==suturo_manipulation_msgs::RobotBodyPart::BOTH_ARMS)){
		// bodypart = both_arms
		ROS_INFO("Move both arms home!");
		// set group to move
		move_group_interface::MoveGroup group(body_part);
		
		// set group name to go home
		group.setNamedTarget(body_part+"_home");
		
		//move bodypart
		if (group.move()){
			ROS_INFO("Moved home!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
			server_home->setSucceeded(r);
		} else {
			ROS_INFO("Moving home failed!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
			server_home->setAborted(r);
		}
	} else {
		ROS_INFO("Unknown bodypart!\n");
		r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		server_home->setAborted(r);	
	}
}

/**
* This callback saves the current controller_errors in class variables
*/
void subCallback(const suturo_manipulation_msgs::torque_valuesConstPtr& msg){
	yaw_error = msg->yaw_error;
	pitch_error = msg->pitch_error;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_move_home_server");
	ros::NodeHandle n;
	listener = new (tf::TransformListener);

	// Publish a topic for the head controller
    ros::Publisher head_publisher = n.advertise<geometry_msgs::PoseStamped>("/suturo/head_controller_goal_point", 1000);

    // Subscribe to the controller_errors
	ros::Subscriber sub = n.subscribe("/suturo/head_controller/torque_values", 1000, subCallback);

	// create the action server
	Server server_home(n, "suturo_man_move_home_server", boost::bind(&moveHome, _1, &head_publisher, &server_home), false);
	// start the server
	server_home.start();
	
	ROS_INFO("Ready to go home!");
	ros::spin();
	return 0;
}
