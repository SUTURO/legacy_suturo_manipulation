#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl/jntarray.hpp>
#include <ReflexxesAPI.h>
#include <suturo_manipulation_test_reflexxes_interpolator.h>
#include <suturo_manipulation_2d_interpolator.h>

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "test_arrive_x_y_at_once");
    // ros::NodeHandle n;
    return RUN_ALL_TESTS();
}

bool arrive(double targetPose, double robotPose)
{
    return (robotPose < targetPose + 0.02 && robotPose > targetPose - 0.02);
}

TEST(suturo_manipulation_move_robot, interpolator_2d_arrive_1_1_at_once)
{

    struct pose_2d rp;
    struct pose_2d tp;

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    tp.x_ = 1;
    tp.y_ = 1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator;

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
    }

    ROS_INFO_STREAM("x: " << inp_params->robot_pose_.x_ << " x_target: " << tp.x_ << " = " << arrive(tp.x_, inp_params->robot_pose_.x_));
    ROS_INFO_STREAM("y: " << inp_params->robot_pose_.y_ << " y_target: " << tp.y_ << " = " << arrive(tp.y_, inp_params->robot_pose_.y_));

    ASSERT_TRUE(arrive(tp.x_, rv.int_pose_.x_) && arrive(tp.y_, rv.int_pose_.y_));

    SUCCEED();
}

TEST(suturo_manipulation_move_robot, interpolator_2d_arrive_0_0_at_once)
{

    struct pose_2d rp;
    struct pose_2d tp;

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    tp.x_ = 0;
    tp.y_ = 0;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator;

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
    }

    ROS_INFO_STREAM("x: " << inp_params->robot_pose_.x_ << " x_target: " << tp.x_ << " = " << arrive(tp.x_, inp_params->robot_pose_.x_));
    ROS_INFO_STREAM("y: " << inp_params->robot_pose_.y_ << " y_target: " << tp.y_ << " = " << arrive(tp.y_, inp_params->robot_pose_.y_));

    ASSERT_TRUE(arrive(tp.x_, rv.int_pose_.x_) && arrive(tp.y_, rv.int_pose_.y_));

    SUCCEED();
}

TEST(suturo_manipulation_move_robot, interpolator_2d_arrive_0_1_at_once)
{

    struct pose_2d rp;
    struct pose_2d tp;

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    tp.x_ = 0;
    tp.y_ = 1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator;

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
    }

    ROS_INFO_STREAM("x: " << inp_params->robot_pose_.x_ << " x_target: " << tp.x_ << " = " << arrive(tp.x_, inp_params->robot_pose_.x_));
    ROS_INFO_STREAM("y: " << inp_params->robot_pose_.y_ << " y_target: " << tp.y_ << " = " << arrive(tp.y_, inp_params->robot_pose_.y_));

    ASSERT_FALSE(arrive(tp.x_, rv.int_pose_.x_) && arrive(tp.y_, rv.int_pose_.y_));

    SUCCEED();
}

TEST(suturo_manipulation_move_robot, interpolator_2d_arrive_3_3_at_once)
{

    struct pose_2d rp;
    struct pose_2d tp;

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    tp.x_ = 3;
    tp.y_ = 3;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator;

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
    }

    ROS_INFO_STREAM("x: " << inp_params->robot_pose_.x_ << " x_target: " << tp.x_ << " = " << arrive(tp.x_, inp_params->robot_pose_.x_));
    ROS_INFO_STREAM("y: " << inp_params->robot_pose_.y_ << " y_target: " << tp.y_ << " = " << arrive(tp.y_, inp_params->robot_pose_.y_));

    ASSERT_TRUE(arrive(tp.x_, rv.int_pose_.x_) && arrive(tp.y_, rv.int_pose_.y_));

    SUCCEED();
}


TEST(suturo_manipulation_move_robot, interpolator_2d_arrive_3_1_at_once)
{

    struct pose_2d rp;
    struct pose_2d tp;

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    tp.x_ = 3;
    tp.y_ = 1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator;

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
    }

    ROS_INFO_STREAM("x: " << inp_params->robot_pose_.x_ << " x_target: " << tp.x_ << " = " << arrive(tp.x_, inp_params->robot_pose_.x_));
    ROS_INFO_STREAM("y: " << inp_params->robot_pose_.y_ << " y_target: " << tp.y_ << " = " << arrive(tp.y_, inp_params->robot_pose_.y_));

    ASSERT_TRUE(arrive(tp.x_, rv.int_pose_.x_) && arrive(tp.y_, rv.int_pose_.y_));

    SUCCEED();
}

TEST(suturo_manipulation_move_robot, interpolator_2d_arrive_2_1_at_once)
{

    struct pose_2d rp;
    struct pose_2d tp;

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    tp.x_ = 2;
    tp.y_ = 1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator;

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
    }

    ROS_INFO_STREAM("x: " << inp_params->robot_pose_.x_ << " x_target: " << tp.x_ << " = " << arrive(tp.x_, inp_params->robot_pose_.x_));
    ROS_INFO_STREAM("y: " << inp_params->robot_pose_.y_ << " y_target: " << tp.y_ << " = " << arrive(tp.y_, inp_params->robot_pose_.y_));

    ASSERT_TRUE(arrive(tp.x_, rv.int_pose_.x_) && arrive(tp.y_, rv.int_pose_.y_));

    SUCCEED();
}


TEST(suturo_manipulation_move_robot, interpolator_2d_arrive_3_2_at_once)
{

    struct pose_2d rp;
    struct pose_2d tp;

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    tp.x_ = 3;
    tp.y_ = 2;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator;

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
    }

    ROS_INFO_STREAM("x: " << inp_params->robot_pose_.x_ << " x_target: " << tp.x_ << " = " << arrive(tp.x_, inp_params->robot_pose_.x_));
    ROS_INFO_STREAM("y: " << inp_params->robot_pose_.y_ << " y_target: " << tp.y_ << " = " << arrive(tp.y_, inp_params->robot_pose_.y_));

    ASSERT_TRUE(arrive(tp.x_, rv.int_pose_.x_) && arrive(tp.y_, rv.int_pose_.y_));

    SUCCEED();
}


TEST(suturo_manipulation_move_robot, interpolator_2d_arrive_neg3_2_at_once)
{

    struct pose_2d rp;
    struct pose_2d tp;

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    tp.x_ = -3;
    tp.y_ = 2;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator;

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
    }

    ROS_INFO_STREAM("x: " << inp_params->robot_pose_.x_ << " x_target: " << tp.x_ << " = " << arrive(tp.x_, inp_params->robot_pose_.x_));
    ROS_INFO_STREAM("y: " << inp_params->robot_pose_.y_ << " y_target: " << tp.y_ << " = " << arrive(tp.y_, inp_params->robot_pose_.y_));

    ASSERT_TRUE(arrive(tp.x_, rv.int_pose_.x_) && arrive(tp.y_, rv.int_pose_.y_));

    SUCCEED();
}


TEST(suturo_manipulation_move_robot, interpolator_2d_arrive_1_neg1_at_once)
{

    struct pose_2d rp;
    struct pose_2d tp;

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    tp.x_ = 1;
    tp.y_ = -1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator;

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
    }

    ROS_INFO_STREAM("x: " << inp_params->robot_pose_.x_ << " x_target: " << tp.x_ << " = " << arrive(tp.x_, inp_params->robot_pose_.x_));
    ROS_INFO_STREAM("y: " << inp_params->robot_pose_.y_ << " y_target: " << tp.y_ << " = " << arrive(tp.y_, inp_params->robot_pose_.y_));

    ASSERT_TRUE(arrive(tp.x_, rv.int_pose_.x_) && arrive(tp.y_, rv.int_pose_.y_));

    SUCCEED();
}

TEST(suturo_manipulation_move_robot, interpolator_2d_arrive_1_1)
{

    struct pose_2d rp;
    struct pose_2d tp;

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    tp.x_ = 1;
    tp.y_ = 1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator;

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
    }

    ROS_INFO_STREAM("x: " << inp_params->robot_pose_.x_ << " x_target: " << tp.x_ << " = " << arrive(tp.x_, inp_params->robot_pose_.x_));
    ROS_INFO_STREAM("y: " << inp_params->robot_pose_.y_ << " y_target: " << tp.y_ << " = " << arrive(tp.y_, inp_params->robot_pose_.y_));

    ASSERT_TRUE(rv.result_value_ == ReflexxesAPI::RML_FINAL_STATE_REACHED);

    SUCCEED();
}


TEST(suturo_manipulation_move_robot, interpolator_2d_arrive_3_1)
{

    struct pose_2d rp;
    struct pose_2d tp;

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    tp.x_ = 3;
    tp.y_ = 1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator;

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
    }

    ROS_INFO_STREAM("x: " << inp_params->robot_pose_.x_ << " x_target: " << tp.x_ << " = " << arrive(tp.x_, inp_params->robot_pose_.x_));
    ROS_INFO_STREAM("y: " << inp_params->robot_pose_.y_ << " y_target: " << tp.y_ << " = " << arrive(tp.y_, inp_params->robot_pose_.y_));

    ASSERT_TRUE(rv.result_value_ == ReflexxesAPI::RML_FINAL_STATE_REACHED);

    SUCCEED();
}


TEST(suturo_manipulation_move_robot, interpolator_2d_arrive_1_neg1)
{

    struct pose_2d rp;
    struct pose_2d tp;

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    tp.x_ = 1;
    tp.y_ = -1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator;

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
    }

    ROS_INFO_STREAM("x: " << inp_params->robot_pose_.x_ << " x_target: " << tp.x_ << " = " << arrive(tp.x_, inp_params->robot_pose_.x_));
    ROS_INFO_STREAM("y: " << inp_params->robot_pose_.y_ << " y_target: " << tp.y_ << " = " << arrive(tp.y_, inp_params->robot_pose_.y_));

    ASSERT_TRUE(rv.result_value_ == ReflexxesAPI::RML_FINAL_STATE_REACHED);

    SUCCEED();
}


// TEST(suturo_manipulation_move_robot, arrive_1_1_at_once)
// {

//     // Set start and goal
//     geometry_msgs::PoseStamped robotPose;
//     robotPose.pose.position.x = 0;
//     robotPose.pose.position.y = 0;
//     robotPose.pose.position.z = 0;
//     robotPose.pose.orientation.x = 0;
//     robotPose.pose.orientation.y = 0;
//     robotPose.pose.orientation.z = 0;
//     robotPose.pose.orientation.w = 1;
//     robotPose.header.frame_id = "/base_link";

//     geometry_msgs::PoseStamped targetPose;
//     targetPose.pose.position.x = 1;
//     targetPose.pose.position.y = 1;
//     targetPose.pose.position.z = 0;
//     targetPose.pose.orientation.x = 0;
//     targetPose.pose.orientation.y = 0;
//     targetPose.pose.orientation.z = 0;
//     targetPose.pose.orientation.w = 1;
//     targetPose.header.frame_id = "/base_link";

//     Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

//     ASSERT_TRUE(interpolator->interpolate(robotPose, targetPose));

//     // ASSERT_TRUE(1 == 1);

//     SUCCEED();
// }


// TEST(suturo_manipulation_move_robot, arrive_2_2_at_once)
// {

//     // Set start and goal
//     geometry_msgs::PoseStamped robotPose;
//     robotPose.pose.position.x = 0;
//     robotPose.pose.position.y = 0;
//     robotPose.pose.position.z = 0;
//     robotPose.pose.orientation.x = 0;
//     robotPose.pose.orientation.y = 0;
//     robotPose.pose.orientation.z = 0;
//     robotPose.pose.orientation.w = 1;
//     robotPose.header.frame_id = "/base_link";

//     geometry_msgs::PoseStamped targetPose;
//     targetPose.pose.position.x = 2;
//     targetPose.pose.position.y = 2;
//     targetPose.pose.position.z = 0;
//     targetPose.pose.orientation.x = 0;
//     targetPose.pose.orientation.y = 0;
//     targetPose.pose.orientation.z = 0;
//     targetPose.pose.orientation.w = 1;
//     targetPose.header.frame_id = "/base_link";

//     Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

//     ASSERT_TRUE(interpolator->interpolate(robotPose, targetPose));

//     // ASSERT_TRUE(1 == 1);

//     SUCCEED();
// }


// TEST(suturo_manipulation_move_robot, arrive_1_4_at_once)
// {

//     // Set start and goal
//     geometry_msgs::PoseStamped robotPose;
//     robotPose.pose.position.x = 0;
//     robotPose.pose.position.y = 0;
//     robotPose.pose.position.z = 0;
//     robotPose.pose.orientation.x = 0;
//     robotPose.pose.orientation.y = 0;
//     robotPose.pose.orientation.z = 0;
//     robotPose.pose.orientation.w = 1;
//     robotPose.header.frame_id = "/base_link";

//     geometry_msgs::PoseStamped targetPose;
//     targetPose.pose.position.x = 1;
//     targetPose.pose.position.y = 4;
//     targetPose.pose.position.z = 0;
//     targetPose.pose.orientation.x = 0;
//     targetPose.pose.orientation.y = 0;
//     targetPose.pose.orientation.z = 0;
//     targetPose.pose.orientation.w = 1;
//     targetPose.header.frame_id = "/base_link";

//     Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

//     ASSERT_TRUE(interpolator->interpolate(robotPose, targetPose));

//     // ASSERT_TRUE(1 == 1);

//     SUCCEED();
// }


// TEST(suturo_manipulation_move_robot, arrive_4_1_at_once)
// {

//     // Set start and goal
//     geometry_msgs::PoseStamped robotPose;
//     robotPose.pose.position.x = 0;
//     robotPose.pose.position.y = 0;
//     robotPose.pose.position.z = 0;
//     robotPose.pose.orientation.x = 0;
//     robotPose.pose.orientation.y = 0;
//     robotPose.pose.orientation.z = 0;
//     robotPose.pose.orientation.w = 1;
//     robotPose.header.frame_id = "/base_link";

//     geometry_msgs::PoseStamped targetPose;
//     targetPose.pose.position.x = 4;
//     targetPose.pose.position.y = 1;
//     targetPose.pose.position.z = 0;
//     targetPose.pose.orientation.x = 0;
//     targetPose.pose.orientation.y = 0;
//     targetPose.pose.orientation.z = 0;
//     targetPose.pose.orientation.w = 1;
//     targetPose.header.frame_id = "/base_link";

//     Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

//     ASSERT_TRUE(interpolator->interpolate(robotPose, targetPose));

//     // ASSERT_TRUE(1 == 1);

//     SUCCEED();
// }

// TEST(suturo_manipulation_move_robot, arrive_1_neg1_at_once)
// {

//     // Set start and goal
//     geometry_msgs::PoseStamped robotPose;
//     robotPose.pose.position.x = 0;
//     robotPose.pose.position.y = 0;
//     robotPose.pose.position.z = 0;
//     robotPose.pose.orientation.x = 0;
//     robotPose.pose.orientation.y = 0;
//     robotPose.pose.orientation.z = 0;
//     robotPose.pose.orientation.w = 1;
//     robotPose.header.frame_id = "/base_link";

//     geometry_msgs::PoseStamped targetPose;
//     targetPose.pose.position.x = 1;
//     targetPose.pose.position.y = -1;
//     targetPose.pose.position.z = 0;
//     targetPose.pose.orientation.x = 0;
//     targetPose.pose.orientation.y = 0;
//     targetPose.pose.orientation.z = 0;
//     targetPose.pose.orientation.w = 1;
//     targetPose.header.frame_id = "/base_link";

//     Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

//     ASSERT_TRUE(interpolator->interpolate(robotPose, targetPose));

//     // ASSERT_TRUE(1 == 1);

//     SUCCEED();
// }

// TEST(suturo_manipulation_move_robot, arrive_0_0_at_once)
// {

//     // Set start and goal
//     geometry_msgs::PoseStamped robotPose;
//     robotPose.pose.position.x = 0;
//     robotPose.pose.position.y = 0;
//     robotPose.pose.position.z = 0;
//     robotPose.pose.orientation.x = 0;
//     robotPose.pose.orientation.y = 0;
//     robotPose.pose.orientation.z = 0;
//     robotPose.pose.orientation.w = 1;
//     robotPose.header.frame_id = "/base_link";

//     geometry_msgs::PoseStamped targetPose;
//     targetPose.pose.position.x = 0;
//     targetPose.pose.position.y = 0;
//     targetPose.pose.position.z = 0;
//     targetPose.pose.orientation.x = 0;
//     targetPose.pose.orientation.y = 0;
//     targetPose.pose.orientation.z = 0;
//     targetPose.pose.orientation.w = 1;
//     targetPose.header.frame_id = "/base_link";

//     Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

//     ASSERT_TRUE(interpolator->interpolate(robotPose, targetPose));

//     // ASSERT_TRUE(1 == 1);

//     SUCCEED();
// }

// TEST(suturo_manipulation_move_robot, arrive_0_1_at_once)
// {

//     // Set start and goal
//     geometry_msgs::PoseStamped robotPose;
//     robotPose.pose.position.x = 0;
//     robotPose.pose.position.y = 0;
//     robotPose.pose.position.z = 0;
//     robotPose.pose.orientation.x = 0;
//     robotPose.pose.orientation.y = 0;
//     robotPose.pose.orientation.z = 0;
//     robotPose.pose.orientation.w = 1;
//     robotPose.header.frame_id = "/base_link";

//     geometry_msgs::PoseStamped targetPose;
//     targetPose.pose.position.x = 0;
//     targetPose.pose.position.y = 1;
//     targetPose.pose.position.z = 0;
//     targetPose.pose.orientation.x = 0;
//     targetPose.pose.orientation.y = 0;
//     targetPose.pose.orientation.z = 0;
//     targetPose.pose.orientation.w = 1;
//     targetPose.header.frame_id = "/base_link";

//     Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

//     ASSERT_FALSE(interpolator->interpolate(robotPose, targetPose));

//     // ASSERT_TRUE(1 == 1);

//     SUCCEED();
// }