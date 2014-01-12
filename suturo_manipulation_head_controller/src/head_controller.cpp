#include "suturo_manipulation_head_controller/head_controller.h"
#include <pluginlib/class_list_macros.h>
#include <math.h>       /* acos */
#include <visualization_msgs/Marker.h>

#define PI 3.14159265

#include "ros/ros.h"
#include <suturo_manipulation_msgs/suturo_manipulation_headAction.h>
#include <tf/transform_listener.h>

#include <ros/callback_queue.h>

using namespace my_controller_ns;


void MyCartControllerClass::setGoalCB(geometry_msgs::PoseStamped msg)
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
bool MyCartControllerClass::init(pr2_mechanism_model::RobotState *robot,
        ros::NodeHandle &n)
{
	updated = false;
	
    /*
    // Get the root and tip link names from parameter server.
    std::string root_name, tip_name;
    if (!n.getParam("root_name", root_name))
    {
        ROS_ERROR("No root name given in namespace: %s)",
                n.getNamespace().c_str());
        return false;
    }
    if (!n.getParam("tip_name", tip_name))
    {
        ROS_ERROR("No tip name given in namespace: %s)",
                n.getNamespace().c_str());
        return false;
    }
    */

    // Construct a chain from the root to the tip and prepare the kinematics.
    // Note the joints must be calibrated.
    if (!chain_.init(robot, "torso_lift_link", "head_plate_frame"))
    {
      //  ROS_ERROR("MyCartController could not use the chain from '%s' to '%s'",
        //        root_name.c_str(), tip_name.c_str());
        return false;
    }

    // Store the robot handle for later use (to get time).
    robot_state_ = robot;

    // Construct the kdl solvers in non-realtime.
    chain_.toKDL(kdl_chain_);
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

    // Resize (pre-allocate) the variables in non-realtime.
    q_.resize(kdl_chain_.getNrOfJoints());
    q0_.resize(kdl_chain_.getNrOfJoints());
    qdot_.resize(kdl_chain_.getNrOfJoints());
    tau_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());
    ROS_INFO("Number of Joints: %d", kdl_chain_.getNrOfJoints());

    // Pick the gains.
    Kp_.vel(0) = 100.0;  Kd_.vel(0) = 1.0;        // Translation x
    Kp_.vel(1) = 100.0;  Kd_.vel(1) = 1.0;        // Translation y
    Kp_.vel(2) = 100.0;  Kd_.vel(2) = 1.0;        // Translation z
    Kp_.rot(0) = 100.0;  Kd_.rot(0) = 1.0;        // Rotation x
    Kp_.rot(1) = 100.0;  Kd_.rot(1) = 1.0;        // Rotation y
    Kp_.rot(2) = 100.0;  Kd_.rot(2) = 1.0;        // Rotation z


    sub_ = n.subscribe("/suturo/head_controller_goal_point", 1, &MyCartControllerClass::setGoalCB, this);
    ROS_INFO("Subscribed!");

    vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    
    listener.waitForTransform("torso_lift_link", "head_plate_frame", ros::Time(0), ros::Duration(1.0));
    
    return true;
}

/// Controller startup in realtime 
void MyCartControllerClass::starting()
{
    // Get the current joint values to compute the initial tip location.
    chain_.getPositions(q0_);
    jnt_to_pose_solver_->JntToCart(q0_, x0_);

    // Initialize the phase of the circle as zero.
    circle_phase_ = 0.0;

    // Also reset the time-of-last-servo-cycle.
    last_time_ = robot_state_->getTime();
}


/// Controller update loop in realtime
void MyCartControllerClass::update()
{
	if (updated){
		
		/*
		double dt;                    // Servo loop time step

		// Calculate the dt between servo cycles.
		dt = (robot_state_->getTime() - last_time_).toSec();
		last_time_ = robot_state_->getTime();
		*/

		// Get the current joint positions and velocities.
		chain_.getPositions(q_);
		chain_.getVelocities(qdot_);

		// Compute the forward kinematics and Jacobian (at this location).
		jnt_to_pose_solver_->JntToCart(q_, x_);
		jnt_to_jac_solver_->JntToJac(q_, J_);

		for (unsigned int i = 0 ; i < 6 ; i++)
		{
			xdot_(i) = 0;
			for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; j++)
				xdot_(i) += J_(i,j) * qdot_.qdot(j);
		}

		/*
		// Follow a circle of 10cm at 3 rad/sec.
		circle_phase_ += 3.0 * dt;
		KDL::Vector  circle(0,0,0);
		circle(2) = 0.1 * sin(circle_phase_);
		circle(1) = 0.1 * (cos(circle_phase_) - 1);

		xd_ = x0_;
		xd_.p += circle;

		// Calculate a Cartesian restoring force.
		xerr_.vel = x_.p - xd_.p;
		xerr_.rot = 0.5 * (xd_.M.UnitX() * x_.M.UnitX() +
				xd_.M.UnitY() * x_.M.UnitY() +
				xd_.M.UnitZ() * x_.M.UnitZ());

		for (unsigned int i = 0 ; i < 6 ; i++)
			F_(i) = - Kp_(i) * xerr_(i) - Kd_(i) * xdot_(i);

		// Convert the force into a set of joint torques.
		for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++)
		{
			tau_(i) = 0;
			for (unsigned int j = 0 ; j < 6 ; j++)
				tau_(i) += J_(j,i) * F_(j);
		}

		// And finally send these torques out.
		chain_.setEfforts(tau_);
		*/
		
			
		listener.transformPoint("/head_plate_frame", ros::Time(0), originPoint_, "/torso_lift_link", goalPoint_);
		//~ ROS_INFO("originPoint: %f, %f, %f", originPoint_.point.x, originPoint_.point.y, originPoint_.point.z);
		//~ ROS_INFO("goalPoint: %f, %f, %f", goalPoint_.point.x, goalPoint_.point.y, goalPoint_.point.z);

		// Publish the Marker
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

		//~ // Calculating the angle on the x-y-plane
		//~ // Setting the z-value to zero
		//~ KDL::Vector goal_xy(goalPoint.x,goalPoint.y,0);
		//~ KDL::Vector curr_xy(x_.p.x(),x_.p.y(),0);
//~ 
		//~ // angular error = arccos((goal_xy.curr_xy)/(|goal_xy|*|curr_xy|))*(180/PI)
		//~ double x_error = acos(dot(goal_xy, curr_xy)/(goal_xy.Norm()*curr_xy.Norm())) * (180.0/PI);
		double z_error = goalPoint_.point.z;
		

		//~ // Calculating the angle on the x-z-plane
		//~ // Setting the y-value to zero
		//~ KDL::Vector goal_xz(goal_pose_.x(),0,goal_pose_.z());
		//~ KDL::Vector curr_xz(x_.p.x(),0,x_.p.z());
//~ 
		//~ // angular error = arccos((goal_xz.curr_xz)/(|goal_xz|*|curr_xz|))*(180/PI)
		//~ double y_error = acos(dot(goal_xz, curr_xz)/(goal_xz.Norm()*curr_xz.Norm())) * (180.0/PI);
		//~ if((x_.p.z() * goal_pose_.x() - x_.p.x() * goal_pose_.z()) > 0)
		//~ {
			//~ y_error = - y_error;
		//~ }
		double y_error = goalPoint_.point.y;
		ROS_INFO("z_error: %f, y_error: %f",z_error,y_error);

		// Setting the Force
		F_(0) = 0;
		F_(1) = 0;
		F_(2) = 0;
		F_(3) = 0;
		F_(4) = 50 * -z_error;
		F_(5) = 50 * -y_error;

		//~ // Convert the force into a set of joint torques.
		//~ for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++)
		//~ {
			//~ tau_(i) = 0;
			//~ for (unsigned int j = 0 ; j < 6 ; j++)
				//~ tau_(i) += J_(j,i) * F_(j);
			//~ ROS_INFO("Effort: %f " , tau_(i));
		//~ 
		// tau_(0) > 0 => kopf dreht nach links
		tau_(0) = y_error;
		// tau_(1) > 0 => kopf kippt nach unten
		tau_(1) = -z_error * 5;

		// And finally send these torques out.
		chain_.setEfforts(tau_);
	}
}



/// Controller stopping in realtime
void MyCartControllerClass::stopping()
{}



/// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(my_controller_ns::MyCartControllerClass,
        pr2_controller_interface::Controller)

