#ifndef SUTURO_MANIPULATION_2D_INTERPOLATOR
#define SUTURO_MANIPULATION_2D_INTERPOLATOR

#include <ros/ros.h>

#include <iostream>
#include <fstream>

#include <kdl/jntarray.hpp>

#include <ReflexxesAPI.h>

using namespace std;

struct pose_2d
{
    double x_, y_;
    string reference_;
};

struct twist_2d
{
    double xdot_, ydot_;
    string reference_;
};

struct interpolator_2d_init_params
{
    double cycle_time_;
    double vel_limit_;
    double acc_limit_;
    double jerk_limit_;
    double range_;
};

struct interpolator_2d_params
{
    pose_2d robot_pose_, target_pose_;
    twist_2d twist_;
};

struct interpolator_2d_result
{
    pose_2d int_pose_;
    twist_2d twist_;
    int result_value_;
};

// class 2d_interpolator_exception: public exception
// {
//   virtual const char* what() const throw()
//   {
//     return "2d interpolator died.";
//   }
// };

class Suturo_Manipulation_2d_Interpolator
{
public:
    Suturo_Manipulation_2d_Interpolator() {}
    ~Suturo_Manipulation_2d_Interpolator() {}

    void init(const interpolator_2d_init_params &params); // throws {}; //throws {2d_interpolator_exception};

    const interpolator_2d_result& interpolate(const interpolator_2d_params &params);   //throws {};

private:
    interpolator_2d_result result_;

    const static int dof_ = 2;

    ReflexxesAPI *trajectory_generator_;

    RMLPositionInputParameters *trajectory_input_;
    RMLPositionOutputParameters *trajectory_output_;
    RMLPositionFlags trajectory_generator_flags_;

    int rv_;

    double radius_range_;

    // bool targetInRange(pose_2d robot_pose_, pose_2d target_pose_);
};

#endif