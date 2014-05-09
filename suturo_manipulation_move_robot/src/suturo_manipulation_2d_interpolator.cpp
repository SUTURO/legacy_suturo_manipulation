#include "suturo_manipulation_2d_interpolator.h"

bool targetInRange(pose_2d robot_pose_, pose_2d target_pose_)
{
    double x_diff = robot_pose_.x_ - target_pose_.x_;
    double y_diff = robot_pose_.y_ - target_pose_.y_;

    return (abs(x_diff) < 0.025) && (abs(y_diff) < 0.025);
}

void Suturo_Manipulation_2d_Interpolator::init(const interpolator_2d_init_params &params)
{
    rv_ = 0;

    trajectory_generator = new ReflexxesAPI(dof_, params.cycle_time_);
    trajectory_input = new RMLPositionInputParameters(dof_);
    trajectory_output = new RMLPositionOutputParameters(dof_);

    for (int i = 0; i < dof_; ++i)
    {
        trajectory_input->SetMaxVelocityVectorElement(params.vel_limit_, i);
        trajectory_input->SetMaxAccelerationVectorElement(params.acc_limit_, i);
        trajectory_input->SetMaxJerkVectorElement(params.jerk_limit_, i);

        trajectory_input->SetTargetVelocityVectorElement(0.0, i);

        trajectory_input->SetSelectionVectorElement(true, i);
    }

}

const interpolator_2d_result &Suturo_Manipulation_2d_Interpolator::interpolate(const interpolator_2d_params &params)
{
    struct twist_2d ip_twist;
    struct pose_2d int_pose_;
    struct interpolator_2d_result *ir = new interpolator_2d_result();

    if (params.robot_pose_.reference_ != params.target_pose_.reference_)
    {
        ROS_ERROR("robot_pose_.reference_ != target_pose_.reference_!! Frame must be equal!!");
    }

    trajectory_input->SetCurrentPositionVectorElement(params.robot_pose_.x_, 0);
    trajectory_input->SetCurrentPositionVectorElement(params.robot_pose_.y_, 1);

    trajectory_input->SetTargetPositionVectorElement(params.target_pose_.x_, 0);
    trajectory_input->SetTargetPositionVectorElement(params.target_pose_.y_, 1);

    trajectory_input->SetCurrentVelocityVectorElement(params.twist_.xdot_, 0);
    trajectory_input->SetCurrentVelocityVectorElement(params.twist_.ydot_, 1);


    // check the input
    if (!trajectory_input->CheckForValidity())
    {
        ROS_WARN("Trajectory input was not valid.");
    }

    // ROS_INFO_STREAM("rp x: " << params.robot_pose_.x_ << " y: " << params.robot_pose_.y_);
    // ROS_INFO_STREAM("x: " << q_target_(0) << " and y: " << q_target_(1));

    rv_ = trajectory_generator->RMLPosition(*trajectory_input, trajectory_output, trajectory_generator_flags);


    if (rv_ < 0)
    {
        ROS_ERROR("Trajectory generator returned with an error.");
    }

    trajectory_output->GetNewPositionVectorElement(&int_pose_.x_, 0);
    trajectory_output->GetNewPositionVectorElement(&int_pose_.y_, 1);

    trajectory_output->GetNewVelocityVectorElement(&ip_twist.xdot_, 0);
    trajectory_output->GetNewVelocityVectorElement(&ip_twist.ydot_, 1);

    ip_twist.reference_ = params.robot_pose_.reference_;

    // ROS_INFO_STREAM("qdot x: " << qdot_.data(0) << " y: " << qdot_.data(1));

    int_pose_.reference_ = params.robot_pose_.reference_;

    ir->result_value_ = rv_;
    ir->int_pose_ = int_pose_;
    ir->twist_ = ip_twist;

    return *ir;
}