#include "suturo_manipulation_head_controller/head_controller.h"
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"
#include <suturo_manipulation_msgs/suturo_manipulation_headAction.h>
#include <tf/transform_listener.h>
#include <ros/callback_queue.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/LoadController.h>

using namespace suturo;

/// Controller initialization in non-realtime
bool Head_Controller::init(pr2_mechanism_model::RobotState *robot,
        ros::NodeHandle &n)
{
    updated = false;

    //ROS_INFO("begin switching");
    // Load ond switch controllers
    //switchControllers(n);
    
    // Construct a chain from the root to the tip and prepare the kinematics.
    // Note the joints must be calibrated.
    if (!chain_.init(robot, "torso_lift_link", "head_plate_frame"))
    {
        ROS_ERROR("MyCartController could not use the chain from torso_lift_link to head_plate_frame");
        return false;
    }

    // Resize (pre-allocate) the variables in non-realtime.
    tau_.resize(2);

    // Subscribe to the goalpublisher and get ready to publish markers
    sub_ = n.subscribe("/suturo/head_controller_goal_point", 1, &Head_Controller::setGoalCB, this);
    ROS_INFO("Subscribed!");
    vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    
    // Prepare the tf-chain
    listener.waitForTransform("torso_lift_link", "head_plate_frame", ros::Time(0), ros::Duration(1.0));

    if(!pid_controller_.init(ros::NodeHandle(n, "pid_parameters"))){
        ROS_ERROR("suturo_head_controller could not construct PID controller for head_tilt_joint");
    }

    robot_ = robot;
    
    return true;
}

/// Controller startup in realtime 
void Head_Controller::starting()
{
    time_of_last_cycle_ = robot_->getTime();
    pid_controller_.reset();
}

/// Controller update loop in realtime
void Head_Controller::update()
{
    if (!updated){
        // get the current state
        originPoint_.header.frame_id = "head_plate_frame";
        originPoint_.header.seq = 0;
        originPoint_.header.stamp = ros::Time::now();
        originPoint_.point.x = 2;
        originPoint_.point.y = 0;
        originPoint_.point.z = 0;
        listener.transformPoint("/torso_lift_link", ros::Time(0), originPoint_, "/head_plate_frame", originPoint_);
        updated = true;
    } else {
        // transform the Point into the targetframe
        listener.transformPoint("/head_plate_frame", ros::Time(0), originPoint_, "/torso_lift_link", goalPoint_);

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
        // max_torque for head_pan_joint: 2.65
        if (y_error > 2.65){
            y_error = 2.65;
        } else if (y_error < -2.65){
            y_error = -2.65;
        }
        tau_(0) = y_error;
        // tau_(1) > 0 => kopf kippt nach unten
        //tau_(1) = -z_error * 5;

        // PID
        ros::Duration dt = robot_->getTime() - time_of_last_cycle_;
        time_of_last_cycle_ = robot_->getTime();
        pid_error_ = pid_controller_.updatePid(z_error*5, dt);

        // max_torque for head_tilt_joint: 15.00
        if (pid_error_ > 15){
            pid_error_ = 15;
        } else if (pid_error_ < -15){
            pid_error_ = -15;
        }

        tau_(1)=pid_error_;

        // And finally send these torques out.
        chain_.setEfforts(tau_);
        ROS_INFO("pid_error: %f", pid_error_);
    }
}
/// Controller stopping in realtime
void Head_Controller::stopping()
{}

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

/*
 * This method sends a servicecall to stop the
 * head_traj_controller and to start the suturo_head_controller
 */
bool Head_Controller::switchControllers(ros::NodeHandle &n){
    /*
       ros::ServiceClient load_client = n.serviceClient<pr2_mechanism_msgs::LoadController>("/pr2_controller_manager/load_controller");

       pr2_mechanism_msgs::LoadController service_data;
       service_data.request.name = "suturo_head_controller";
       if (load_client.call(service_data)) { 
       ROS_INFO_STREAM("Loaded suturo_head_controller");  
       } else { 
       ROS_INFO_STREAM("Failed to load suturo_head_controller"); 
       return false;
       } 
       */
    ros::ServiceClient switch_client = n.serviceClient<pr2_mechanism_msgs::SwitchController>("/pr2_controller_manager/switch_controller");

    pr2_mechanism_msgs::SwitchController to_imped;
    to_imped.request.start_controllers.push_back("suturo_head_controller");
    to_imped.request.stop_controllers.push_back("head_traj_controller");
    to_imped.request.strictness = pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT;
    ROS_INFO("sending the call from inside of the method");
    if (switch_client.call(to_imped)) { 
        ROS_INFO("Switched from head_traj_controller to suturo_head_controller"); 
        return to_imped.response.ok;
    } else { 
        ROS_INFO("Failed to switch from head_traj_controller to suturo_head_controller"); 
        return false;
    } 

} 
/// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(suturo::Head_Controller,
        pr2_controller_interface::Controller)

