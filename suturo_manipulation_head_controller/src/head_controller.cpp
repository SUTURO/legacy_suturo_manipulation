#include "suturo_manipulation_head_controller/head_controller.h"
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"
#include <suturo_manipulation_msgs/suturo_manipulation_headAction.h>
#include <tf/transform_listener.h>
#include <ros/callback_queue.h>

using namespace suturo;

/*
 * This method is called when a new goal is set.
 * It updates the goal variables and sets the
 * updated varaible to true.
 */
void Head_Controller::setGoalCB(geometry_msgs::PoseStamped msg)
{
	ROS_INFO("I heard: x: %f, y: %f, z: %f in Frame %s", msg.pose.position.x,
        msg.pose.position.y, msg.pose.position.z, msg.header.frame_id.c_str());
    originPoint_.header.frame_id = msg.header.frame_id;
    originPoint_.header.seq = msg.header.seq;
    originPoint_.header.stamp = msg.header.stamp;
    originPoint_.point.x = msg.pose.position.x;
    originPoint_.point.y = msg.pose.position.y;
    originPoint_.point.z = msg.pose.position.z;
    
    updated = true;
}

/// Controller initialization in non-realtime
bool Head_Controller::init(pr2_mechanism_model::RobotState *robot,
        ros::NodeHandle &n)
{
	updated = false;
	
    // Construct a chain from the root to the tip and prepare the kinematics.
    // Note the joints must be calibrated.
    if (!chain_.init(robot, "torso_lift_link", "head_plate_frame"))
    {
        ROS_ERROR("MyCartController could not use the chain from torso_lift_link to head_plate_frame");
        return false;
    }

    // Construct the kdl solvers in non-realtime.
    chain_.toKDL(kdl_chain_);

    // Resize (pre-allocate) the variables in non-realtime.
    tau_.resize(kdl_chain_.getNrOfJoints());

    // Subscribe to the goalpublisher and get ready to publish markers
    sub_ = n.subscribe("/suturo/head_controller_goal_point", 1, &Head_Controller::setGoalCB, this);
    ROS_INFO("Subscribed!");
    vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    
    // Prepare the tf-chain
    listener.waitForTransform("torso_lift_link", "head_plate_frame", ros::Time(0), ros::Duration(1.0));
    
    return true;
}

/// Controller startup in realtime 
void Head_Controller::starting()
{

}

/// Controller update loop in realtime
void Head_Controller::update()
{
	if (updated){

        // transform the Point into the targetframe
        listener.transformPoint("/head_plate_frame", ros::Time(0), originPoint_, "/torso_lift_link", goalPoint_);
		// ROS_INFO("originPoint: %f, %f, %f", originPoint_.point.x, originPoint_.point.y, originPoint_.point.z);
		// ROS_INFO("goalPoint: %f, %f, %f", goalPoint_.point.x, goalPoint_.point.y, goalPoint_.point.z);

		// Publish the Goalmarker
		visualization_msgs::Marker goal_marker;
		goal_marker.header.frame_id = "head_plate_frame";
		goal_marker.header.stamp = ros::Time();
		goal_marker.ns = "suturo_manipulation";
		goal_marker.id = 0;
		goal_marker.type = visualization_msgs::Marker::SPHERE;
		goal_marker.action = visualization_msgs::Marker::ADD;
		goal_marker.pose.position.x = goalPoint_.point.x;
		goal_marker.pose.position.y = goalPoint_.point.y;
		goal_marker.pose.position.z = goalPoint_.point.z;
		goal_marker.pose.orientation.x = 0.0;
		goal_marker.pose.orientation.y = 0.0;
		goal_marker.pose.orientation.z = 0.0;
		goal_marker.pose.orientation.w = 1.0;
		goal_marker.scale.x = 0.1;
		goal_marker.scale.y = 0.1;
		goal_marker.scale.z = 0.1;
		goal_marker.color.a = 1.0;
		goal_marker.color.r = 0.0;
		goal_marker.color.g = 1.0;
		goal_marker.color.b = 0.0;
		vis_pub.publish( goal_marker );
        
        // Get the errors
        double z_error = goalPoint_.point.z;
        double y_error = goalPoint_.point.y;
        ROS_INFO("z_error: %f, y_error: %f",z_error,y_error);

        // tau_(0) > 0 => kopf dreht nach links
		tau_(0) = y_error;
		// tau_(1) > 0 => kopf kippt nach unten
		tau_(1) = -z_error * 5;

		// And finally send these torques out.
		chain_.setEfforts(tau_);
	}
}

/// Controller stopping in realtime
void Head_Controller::stopping()
{}

/// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(suturo::Head_Controller,
        pr2_controller_interface::Controller)

