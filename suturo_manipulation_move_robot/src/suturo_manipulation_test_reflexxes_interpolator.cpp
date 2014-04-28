#include "suturo_manipulation_test_reflexxes_interpolator.h"

Suturo_Manipulation_Test_Reflexxes_Interpolator::Suturo_Manipulation_Test_Reflexxes_Interpolator() : Suturo_Manipulation_Reflexxes_Interpolator()
{

}

bool Suturo_Manipulation_Test_Reflexxes_Interpolator::setInputFromOutput()
{
    trajectory_output->GetNewPositionVector(q_.data.data(), dof_ * sizeof(double));
    trajectory_output->GetNewVelocityVector(qdot_.data.data(), dof_ * sizeof(double));

    return true;
}


bool Suturo_Manipulation_Test_Reflexxes_Interpolator::interpolate(geometry_msgs::PoseStamped robotPose, geometry_msgs::PoseStamped targetPose)
{

    ROS_INFO_STREAM("interpolate");

    init();
    initPositions(robotPose, targetPose);
    setTrajectoryInput();
    // // set up relfexxes-trajectory structures
    // ReflexxesAPI trajectory_generator(dof_, cycle_time_);
    // RMLPositionInputParameters trajectory_input(dof_);
    // RMLPositionOutputParameters trajectory_output(dof_);
    // RMLPositionFlags trajectory_generator_flags;

    // // perform experimental interpolation
    // // set up input of trajectory generator
    // trajectory_input.SetMaxVelocityVector(qdot_max_.data.data());
    // trajectory_input.SetMaxAccelerationVector(qddot_max_.data.data());
    // trajectory_input.SetMaxJerkVector(jerk_max_.data.data());

    // trajectory_input.SetTargetPositionVector(q_target_.data.data());
    // trajectory_input.SetTargetVelocityVector(qdot_target_.data.data());

    // trajectory_input.SetSelectionVector(selection_vector_);

    ResultValue_ = 0;
    while (ResultValue_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arriveX(targetPose.pose.position.x, q_.data(0)) && !arriveY(targetPose.pose.position.y, q_.data(1)))
    // while (ResultValue_ != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        // ROS_INFO_STREAM("begin while: " << ResultValue_);
        // ROS_INFO_STREAM(trajectory_input->GetMaxVelocityVector());
        // // set the current state of the robot as input
        // trajectory_input.SetCurrentPositionVector(q_.data.data());
        // trajectory_input.SetCurrentVelocityVector(qdot_.data.data());

        // // check the input
        // if (!trajectory_input.CheckForValidity())
        // {
        //     ROS_WARN("Trajectory input was not valid.");
        // }

        // // query the trajectory generator
        // ResultValue_ = trajectory_generator.RMLPosition(trajectory_input, &trajectory_output, trajectory_generator_flags);

        // // check the output of the trajectory generator
        // if (ResultValue_ < 0)
        // {
        //     ROS_ERROR("Trajectory generator returned with an error.");
        //     break;
        // }

        // // perform 'control of the robot'
        // time_ += cycle_time_;

        // trajectory_output.GetNewPositionVector(q_.data.data(), dof_ * sizeof(double));
        // trajectory_output.GetNewVelocityVector(qdot_.data.data(), dof_ * sizeof(double));
        calculate();
        setInputFromOutput();

        // ROS_INFO_STREAM("trajectory_output_qdot x: " << qdot_.data(0) << " trajectory_output_qdot y: " << qdot_.data(1));
    }

    ROS_INFO_STREAM("nach while: " << ResultValue_);

    // ROS_INFO_STREAM("trajectory_output_qdot: " << qdot.data);
    ROS_INFO_STREAM("x: " << q_.data(0) << " x_target: " << targetPose.pose.position.x << " " << arriveX(targetPose.pose.position.x, q_.data(0)));
    ROS_INFO_STREAM("y: " << q_.data(1) << " y_target: " << targetPose.pose.position.y << " " << arriveY(targetPose.pose.position.y, q_.data(1)));

    return (arriveX(targetPose.pose.position.x, q_.data(0)) && arriveY(targetPose.pose.position.y, q_.data(1)));
    // return true;

}