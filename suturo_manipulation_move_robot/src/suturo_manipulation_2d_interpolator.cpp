#include "suturo_manipulation_2d_interpolator.h"

bool targetInRange(const pose_2d &robot_pose, const pose_2d &target_pose, double range)
{
    double x_diff = robot_pose.x_ - target_pose.x_;
    double y_diff = robot_pose.y_ - target_pose.y_;

    return (abs(x_diff) < range) && (abs(y_diff) < range);
}

void Suturo_Manipulation_2d_Interpolator::init(const interpolator_2d_init_params &params)
{
    rv_ = 0;

    radius_range_ = params.range_;

    trajectory_generator_ = new ReflexxesAPI(dof_, params.cycle_time_);
    trajectory_input_ = new RMLPositionInputParameters(dof_);
    trajectory_output_ = new RMLPositionOutputParameters(dof_);

    for (int i = 0; i < dof_; ++i)
    {
        trajectory_input_->SetMaxVelocityVectorElement(params.vel_limit_, i);
        trajectory_input_->SetMaxAccelerationVectorElement(params.acc_limit_, i);
        trajectory_input_->SetMaxJerkVectorElement(params.jerk_limit_, i);

        trajectory_input_->SetTargetVelocityVectorElement(0.0, i);

        trajectory_input_->SetSelectionVectorElement(true, i);
    }

}

const interpolator_2d_result &Suturo_Manipulation_2d_Interpolator::interpolate(const interpolator_2d_params &params)
{
    struct twist_2d ip_twist;
    struct pose_2d int_pose;
    struct interpolator_2d_result *ir = new interpolator_2d_result();

    if (params.robot_pose_.reference_ != params.target_pose_.reference_)
    {
        ROS_ERROR("robot_pose_.reference_ != target_pose_.reference_!! Frame must be equal!!");
    }

    if (targetInRange(params.robot_pose_, params.target_pose_, radius_range_))
    {
        trajectory_input_->SetCurrentPositionVectorElement(params.robot_pose_.x_, 0);
        trajectory_input_->SetCurrentPositionVectorElement(params.robot_pose_.y_, 1);

        trajectory_input_->SetTargetPositionVectorElement(params.robot_pose_.x_, 0);
        trajectory_input_->SetTargetPositionVectorElement(params.robot_pose_.y_, 1);

        trajectory_input_->SetCurrentVelocityVectorElement(0, 0);
        trajectory_input_->SetCurrentVelocityVectorElement(0, 1);
    }
    else
    {
        trajectory_input_->SetCurrentPositionVectorElement(params.robot_pose_.x_, 0);
        trajectory_input_->SetCurrentPositionVectorElement(params.robot_pose_.y_, 1);

        trajectory_input_->SetTargetPositionVectorElement(params.target_pose_.x_, 0);
        trajectory_input_->SetTargetPositionVectorElement(params.target_pose_.y_, 1);

        trajectory_input_->SetCurrentVelocityVectorElement(params.twist_.xdot_, 0);
        trajectory_input_->SetCurrentVelocityVectorElement(params.twist_.ydot_, 1);
    }

    // check the input
    if (!trajectory_input_->CheckForValidity())
    {
        ROS_WARN("Trajectory input was not valid.");
    }

    rv_ = trajectory_generator_->RMLPosition(*trajectory_input_, trajectory_output_, trajectory_generator_flags_);


    if (rv_ < 0)
    {
        ROS_ERROR("Trajectory generator returned with an error.");
    }

    trajectory_output_->GetNewPositionVectorElement(&int_pose.x_, 0);
    trajectory_output_->GetNewPositionVectorElement(&int_pose.y_, 1);

    trajectory_output_->GetNewVelocityVectorElement(&ip_twist.xdot_, 0);
    trajectory_output_->GetNewVelocityVectorElement(&ip_twist.ydot_, 1);

    ip_twist.reference_ = params.robot_pose_.reference_;

    int_pose.reference_ = params.robot_pose_.reference_;

    ir->result_value_ = rv_;
    ir->int_pose_ = int_pose;
    ir->twist_ = ip_twist;

    return *ir;
}