#include "suturo_manipulation_reflexxes_interpolator.h"

Suturo_Manipulation_Reflexxes_Interpolator::Suturo_Manipulation_Reflexxes_Interpolator()
{
    ros::NodeHandle *n;
    nh_ = n;
    init();
}

Suturo_Manipulation_Reflexxes_Interpolator::Suturo_Manipulation_Reflexxes_Interpolator(ros::NodeHandle *n)
{
    nh_ = n;
    init();
}

Suturo_Manipulation_Reflexxes_Interpolator::~Suturo_Manipulation_Reflexxes_Interpolator()
{

}

bool Suturo_Manipulation_Reflexxes_Interpolator::initPositions(geometry_msgs::PoseStamped robotPose, geometry_msgs::PoseStamped targetPose)
{

    q_(0) = robotPose.pose.position.x;
    q_(1) = robotPose.pose.position.y;

    q_target_(0) = targetPose.pose.position.x;
    q_target_(1) = targetPose.pose.position.y;


    for (unsigned int i = 0; i < q_.rows(); i++)
    {

        double j = i;

        qdot_max_(i) = 0.2;
        qdot_(i) = 0.0;
        qddot_max_(i) = qdot_max_(i) / 2.0;
        jerk_max_(i) = 0.2;

        qdot_target_(i) = j * qdot_max_(i) * 0.5;

        selection_vector_[i] = true;
    }

    ROS_INFO_STREAM("initPositions done");

    return true;
}

bool Suturo_Manipulation_Reflexxes_Interpolator::updateRobotPositions(geometry_msgs::PoseStamped robotPose)
{
    q_(0) = robotPose.pose.position.x;
    q_(1) = robotPose.pose.position.y;
    return true;
}


int Suturo_Manipulation_Reflexxes_Interpolator::getResultValue()
{
    return ResultValue_;
}


bool Suturo_Manipulation_Reflexxes_Interpolator::arriveX(double targetPose, double robotPose)
{
    return (robotPose < targetPose + 0.02 && robotPose > targetPose - 0.02);
}


bool Suturo_Manipulation_Reflexxes_Interpolator::arriveY(double robotPose, double targetPose)
{
    return (robotPose < targetPose + 0.02 && robotPose > targetPose - 0.02);;
}

bool Suturo_Manipulation_Reflexxes_Interpolator::init()
{
    // init experimental joint-state representation
    // dof_ = 2;
    cycle_time_ = 0.01;
    time_ = 0.00;

    q_.resize(dof_);
    qdot_.resize(dof_);
    qdot_max_.resize(dof_);
    qddot_max_.resize(dof_);
    jerk_max_.resize(dof_);
    q_target_.resize(dof_);
    qdot_target_.resize(dof_);
    ResultValue_ = 0;

    trajectory_generator = new ReflexxesAPI(dof_, cycle_time_);
    trajectory_input = new RMLPositionInputParameters(dof_);
    trajectory_output = new RMLPositionOutputParameters(dof_);

    return true;
}

bool Suturo_Manipulation_Reflexxes_Interpolator::setTrajectoryInput()
{
    trajectory_input->SetMaxVelocityVector(qdot_max_.data.data());
    trajectory_input->SetMaxAccelerationVector(qddot_max_.data.data());
    trajectory_input->SetMaxJerkVector(jerk_max_.data.data());

    trajectory_input->SetTargetPositionVector(q_target_.data.data());
    trajectory_input->SetTargetVelocityVector(qdot_target_.data.data());

    trajectory_input->SetSelectionVector(selection_vector_);

    return true;
}

bool Suturo_Manipulation_Reflexxes_Interpolator::calculate()
{
    // set the current state of the robot as input
    trajectory_input->SetCurrentPositionVector(q_.data.data());
    trajectory_input->SetCurrentVelocityVector(qdot_.data.data());

    // check the input
    if (!trajectory_input->CheckForValidity())
    {
        ROS_WARN("Trajectory input was not valid.");
    }

    // query the trajectory generator
    ResultValue_ = trajectory_generator->RMLPosition(*trajectory_input, trajectory_output, trajectory_generator_flags);

    // check the output of the trajectory generator
    if (ResultValue_ < 0)
    {
        ROS_ERROR("Trajectory generator returned with an error.");
        return false;
    }

    // perform 'control of the robot'
    time_ += cycle_time_;

    return true;
}


std::vector<double> Suturo_Manipulation_Reflexxes_Interpolator::interpolate(geometry_msgs::PoseStamped robotPose, geometry_msgs::PoseStamped targetPose)
{
    ROS_INFO_STREAM("interpolate");

    if (targetInRange(robotPose, targetPose)){
        targetPose.pose.position.x = robotPose.pose.position.x;
        targetPose.pose.position.y = robotPose.pose.position.y;
    }

    initPositions(robotPose, targetPose);
    setTrajectoryInput();
    calculate();

    std::vector<double> twist;

    twist[0] = qdot_.data[0];
    twist[1] = qdot_.data[1];

    return twist;
}

bool Suturo_Manipulation_Reflexxes_Interpolator::targetInRange(geometry_msgs::PoseStamped robotPose, geometry_msgs::PoseStamped targetPose){

    double x_diff = robotPose.pose.position.x - targetPose.pose.position.x;
    double y_diff = robotPose.pose.position.y - targetPose.pose.position.y;

    return (abs(x_diff) < 0.05) && (abs(y_diff) < 0.05);
}