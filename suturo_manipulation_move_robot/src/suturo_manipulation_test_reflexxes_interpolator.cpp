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

    initPositions(robotPose, targetPose);
    setTrajectoryInput();

    ResultValue_ = 0;
    while (ResultValue_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arriveX(targetPose.pose.position.x, q_.data(0)) && !arriveY(targetPose.pose.position.y, q_.data(1)))
    // while (ResultValue_ != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {   
        calculate();
        // setTrajectoryInput(false);
        // setInputFromOutput();
    }

    ROS_INFO_STREAM("nach while: " << ResultValue_);

    ROS_INFO_STREAM("x: " << q_.data(0) << " x_target: " << targetPose.pose.position.x << " = " << arriveX(targetPose.pose.position.x, q_.data(0)));
    ROS_INFO_STREAM("y: " << q_.data(1) << " y_target: " << targetPose.pose.position.y << " = " << arriveY(targetPose.pose.position.y, q_.data(1)));
    ROS_INFO_STREAM("q_data.data(): " << q_.data.data());

    return (arriveX(targetPose.pose.position.x, q_.data(0)) && arriveY(targetPose.pose.position.y, q_.data(1)));

}