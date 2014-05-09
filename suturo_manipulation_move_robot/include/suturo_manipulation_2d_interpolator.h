#ifndef SUTURO_MANIPULATION_2D_INTERPOLATOR
#define SUTURO_MANIPULATION_2D_INTERPOLATOR

#include <ros/ros.h>

#include <iostream>
#include <fstream>

#include <kdl/jntarray.hpp>

#include <ReflexxesAPI.h>

using namespace std;

/**
* Pose with x and y coord and with a reference frame.
*/
struct pose_2d
{
    double x_, y_;
    string reference_;
};

/**
* Twist with x and y velocity and a reference frame.
*/
struct twist_2d
{
    double xdot_, ydot_;
    string reference_;
};

/**
* Initialize parameters for the interpolator.
*/
struct interpolator_2d_init_params
{
    double cycle_time_;
    double vel_limit_;
    double acc_limit_;
    double jerk_limit_;
    double range_;
};

/**
* Input paramaters for the interpolator.
*/
struct interpolator_2d_params
{
    pose_2d robot_pose_, target_pose_;
    twist_2d twist_;
};

/**
* The result which the method interpolate returns.
*/
struct interpolator_2d_result
{
    pose_2d int_pose_;
    twist_2d twist_;
    int result_value_;
};


class Suturo_Manipulation_2d_Interpolator
{
public:
    Suturo_Manipulation_2d_Interpolator() {}
    ~Suturo_Manipulation_2d_Interpolator() {}

    /**
    * Initialize the interpolator with given params.
    */
    void init(const interpolator_2d_init_params &params); 

    /**
    * Interpolates a given pose_2d to a given target (pose_2d).
    * The method calculates a new position and twists to moving
    * the robot.
    *
    * @return The interpolatet result with position, twist and result value.
    *           Returns if the target is arrive
    */
    const interpolator_2d_result &interpolate(const interpolator_2d_params &params);   

private:
    // degrees of freedom
    const static int dof_ = 2;
    // generator to interpolate
    ReflexxesAPI *trajectory_generator_;
    // input for the interpoaltor
    RMLPositionInputParameters *trajectory_input_;
    // output from the interpolator
    RMLPositionOutputParameters *trajectory_output_;
    // flags for the interpolator
    RMLPositionFlags trajectory_generator_flags_;
    // result value from the interpolator
    int rv_;
    // range where the target count as arrived (radius!)
    double radius_range_;
};

#endif