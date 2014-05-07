#include "suturo_manipulation_2d_interpolator.h"

void Suturo_Manipulation_2d_Interpolator::init(const interpolator_2d_init_params &params)
{
    q_.resize(dof_);
    qdot_.resize(dof_);
    qdot_max_.resize(dof_);
    qddot_max_.resize(dof_);
    jerk_max_.resize(dof_);
    q_target_.resize(dof_);
    qdot_target_.resize(dof_);
    rv_ = 0;

    trajectory_generator = new ReflexxesAPI(dof_, params.cycle_time_);
    trajectory_input = new RMLPositionInputParameters(dof_);
    trajectory_output = new RMLPositionOutputParameters(dof_);

    for (unsigned int i = 0; i < q_.rows(); i++)
    {

        double j = i;

        qdot_max_(i) = params.vel_limit_;
        qdot_(i) = 0.0;
        qddot_max_(i) = params.acc_limit_;
        // qddot_max_(i) = qdot_max_(i) / 2.0;
        jerk_max_(i) = params.jerk_limit_;

        // qdot_target_(i) = j * qdot_max_(i) * 0.5;
        qdot_target_(i) = 0.0;

        selection_vector_[i] = true;
    }

    // SetMaxVelocityElement nutzen
    trajectory_input->SetMaxVelocityVector(qdot_max_.data.data());
    trajectory_input->SetMaxAccelerationVector(qddot_max_.data.data());
    trajectory_input->SetMaxJerkVector(jerk_max_.data.data());

    trajectory_input->SetTargetVelocityVector(qdot_target_.data.data());

    trajectory_input->SetSelectionVector(selection_vector_);

}

const interpolator_2d_result &Suturo_Manipulation_2d_Interpolator::interpolate(const interpolator_2d_params &params)
{
    struct twist_2d twist;
    struct pose_2d int_pose_;
    struct interpolator_2d_result *ir = new interpolator_2d_result();

    q_(0) = params.robot_pose_.x_;
    q_(1) = params.robot_pose_.y_;

    q_target_(0) = params.target_pose_.x_;
    q_target_(1) = params.target_pose_.y_;

    if (params.robot_pose_.reference_ != params.target_pose_.reference_)
    {
        ROS_ERROR("robot_pose_.reference_ != target_pose_.reference_!! Frame must be equal!!");
    }

    trajectory_input->SetTargetPositionVector(q_target_.data.data());
    trajectory_input->SetCurrentPositionVector(q_.data.data());
    trajectory_input->SetCurrentVelocityVector(qdot_.data.data());


    // check the input
    if (!trajectory_input->CheckForValidity())
    {
        ROS_WARN("Trajectory input was not valid.");
    }

    rv_ = trajectory_generator->RMLPosition(*trajectory_input, trajectory_output, trajectory_generator_flags);


    if (rv_ < 0)
    {
        ROS_ERROR("Trajectory generator returned with an error.");
    }

    trajectory_output->GetNewPositionVector(q_.data.data(), dof_ * sizeof(double));
    trajectory_output->GetNewVelocityVector(qdot_.data.data(), dof_ * sizeof(double));

    twist.xdot_ = qdot_.data(0);
    twist.ydot_ = qdot_.data(1);
    twist.reference_ = params.robot_pose_.reference_;

    // ROS_INFO_STREAM("qdot x: " << qdot_.data(0) << " y: " << qdot_.data(1));

    int_pose_.x_ = q_.data(0);
    int_pose_.y_ = q_.data(1);
    int_pose_.reference_ = params.robot_pose_.reference_;

    ir->result_value_ = rv_;
    ir->int_pose_ = int_pose_;
    ir->twist_ = twist;

    return *ir;
}


bool Suturo_Manipulation_2d_Interpolator::targetInRange(pose_2d robot_pose_, pose_2d target_pose_)
{
    double x_diff = robot_pose_.x_ - target_pose_.x_;
    double y_diff = robot_pose_.y_ - target_pose_.y_;

    return (abs(x_diff) < 0.05) && (abs(y_diff) < 0.05);
}