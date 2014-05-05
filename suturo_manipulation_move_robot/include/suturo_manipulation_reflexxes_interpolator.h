#ifndef SUTURO_MANIPULATION_REFLEXXES_INTERPOLATOR
#define SUTURO_MANIPULATION_REFLEXXES_INTERPOLATOR

#include <ros/ros.h>

#include <iostream>
#include <fstream>

#include <kdl/jntarray.hpp>

#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>


#include <geometry_msgs/PoseStamped.h>


class Suturo_Manipulation_Reflexxes_Interpolator
{
protected:
    //! The node handle we'll be using
    ros::NodeHandle *nh_;
    const static int dof_ = 2;
    double cycle_time_;
    double time_;
    KDL::JntArray q_target_;
    KDL::JntArray qdot_target_;
    KDL::JntArray q_;
    KDL::JntArray qdot_;
    KDL::JntArray qdot_export_;
    KDL::JntArray qdot_max_;
    KDL::JntArray qddot_max_;
    KDL::JntArray jerk_max_;
    int ResultValue_;
    // set up relfexxes-trajectory structures
    ReflexxesAPI *trajectory_generator;
    
    RMLPositionInputParameters *trajectory_input;
    RMLPositionOutputParameters *trajectory_output;
    RMLPositionFlags trajectory_generator_flags;

    RMLVelocityInputParameters *vi;
    RMLVelocityOutputParameters *vo;
    RMLVelocityFlags vf;

    bool selection_vector_[dof_];

    bool initDone;

    std::vector<double> twist_;

    bool arriveX(double targetPose, double robotPose);

    bool arriveY(double targetPose, double robotPose);

    bool init();

    bool setTrajectoryInput();

    bool calculate();

    bool initPositions(geometry_msgs::PoseStamped robotPose, geometry_msgs::PoseStamped targetPose);

    bool updatePositions(geometry_msgs::PoseStamped robotPose, geometry_msgs::PoseStamped targetPose);

    bool targetInRange(geometry_msgs::PoseStamped robotPose, geometry_msgs::PoseStamped targetPose);

public:
    Suturo_Manipulation_Reflexxes_Interpolator();

    Suturo_Manipulation_Reflexxes_Interpolator(ros::NodeHandle *n);

    ~Suturo_Manipulation_Reflexxes_Interpolator();

    std::vector<double> interpolate(geometry_msgs::PoseStamped robotPose, geometry_msgs::PoseStamped targetPose);

    int getResultValue();
};

#endif