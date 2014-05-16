/**
* This class implements the action server for moving the head.
* If the action server is called, the server publishes 
* the goal to the topic /suturo/head_controller_goal_point .
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_headAction.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <tf/transform_listener.h>
#include <control_msgs/PointHeadActionGoal.h>
#include <pr2_controllers_msgs/PointHeadActionResult.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_headAction> Server;

// TF Listener...
tf::TransformListener* listener = NULL;

control_msgs::PointHeadActionGoal goal_msg;

bool moved = 0;

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

template <class T>
/**
* This method formats a ros time to a string.
* Thanks to https://code.ros.org/trac/ros/ticket/2030
*/
std::string time_to_str(T ros_t)
{
  char buf[1024]      = "";
  time_t t = ros_t.sec;
  struct tm *tms = localtime(&t);
  strftime(buf, 1024, "%Y-%m-%d-%H-%M-%S", tms);
  return std::string(buf);
}

/**
* This method is a callback method and checks, if the head moved.
*/
void moveHeadResult(pr2_controllers_msgs::PointHeadActionResult msg)
{
  if(goal_msg.goal_id.id == msg.status.goal_id.id){
  	// ROS_INFO("Get result!");
  	moved = 1;
  } else {
  	// ROS_INFO("No result!");
  	moved = 0;
  }
}

/**
* This method publishes a goal to the ros intern head mover
*/
void moveHead(const suturo_manipulation_msgs::suturo_manipulation_headGoalConstPtr& goal, 
			ros::Publisher* publisher, Server* server_head)
{	
	ROS_INFO("callback moveHead() begins...");
	suturo_manipulation_msgs::suturo_manipulation_headResult r;	

	// Set header
	r.succ.header.stamp = ros::Time();
	// Set Answer for planning to undefined
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;

	//transform pose
	// if (!transform(transformedPose, goal->ps.pose.position, goal->ps.header.frame_id.c_str())){
	// 	// If tranfsormation fails, update the answer for planning to "FAIL" and set the server aborted
	// 	r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
	// 	server_head->setAborted(r);
	// }

	// Set home Coordinates, time and frame
	goal_msg.header.seq = 1;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = goal->ps.header.frame_id;
    goal_msg.goal_id.stamp = goal_msg.header.stamp;
    // set unique id with timestamp
    goal_msg.goal_id.id = "goal_"+time_to_str(goal_msg.header.stamp);
    goal_msg.goal.target.header = goal_msg.header;
    goal_msg.goal.target.point.x = goal->ps.pose.position.x;
    goal_msg.goal.target.point.y = goal->ps.pose.position.y;
    goal_msg.goal.target.point.z = goal->ps.pose.position.z;
    goal_msg.goal.pointing_axis.x = 1;
    goal_msg.goal.pointing_axis.y = 0;
    goal_msg.goal.pointing_axis.z = 0;
    goal_msg.goal.pointing_frame = "head_plate_frame";
    goal_msg.goal.min_duration = ros::Duration(1.0);
    goal_msg.goal.max_velocity = 10;

	// Publish goal on topic /suturo/head_controller_goal_point
	if( !publisher ) {
		ROS_WARN("Publisher invalid!\n");
	  	r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
	  	server_head->setAborted(r);
	} else {
		ROS_INFO("Published goal: x: %f, y: %f, z: %f in Frame %s", goal_msg.goal.target.point.x,
		goal_msg.goal.target.point.y, goal_msg.goal.target.point.z, goal_msg.goal.pointing_frame.c_str());	
		publisher->publish(goal_msg);
		ROS_INFO("Goal published!");
		ros::WallDuration(2.0).sleep();
		if(moved == 1){
			ROS_INFO("Head moved!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
			server_head->setSucceeded(r);
		} else {
			ROS_INFO("Head doesn't move!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
	  		server_head->setAborted(r);
		}		
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_head_server");
	ros::NodeHandle n;
	listener = new (tf::TransformListener);
	
	// Subscribe the topic, call moveHeadResult if data is published
	ros::Subscriber sub = n.subscribe("/head_traj_controller/point_head_action/result", 1000, moveHeadResult);

	// Publish a topic for the ros intern head controller
	ros::Publisher head_publisher = n.advertise<control_msgs::PointHeadActionGoal>("/head_traj_controller/point_head_action/goal", 1000);

	// create the action server
	Server server_head(n, "suturo_man_move_head_server", boost::bind(&moveHead, _1, &head_publisher, &server_head), false);
	// start the server
	server_head.start();

	ROS_INFO("Ready to move the head!");
	ros::spin();
	return 0;
}
