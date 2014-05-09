#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
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
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";


    tp.x_ = 1;
    tp.y_ = 1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;
    inp_params->twist_.xdot_ = 0.0;
    inp_params->twist_.ydot_ = 0.0;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;
    init_params->range_ = 0.0;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator();

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
        inp_params->twist_ = rv.twist_;
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
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";


    tp.x_ = 0;
    tp.y_ = 0;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;
    inp_params->twist_.xdot_ = 0.0;
    inp_params->twist_.ydot_ = 0.0;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;
    init_params->range_ = 0.0;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator();

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
        inp_params->twist_ = rv.twist_;
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
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";


    tp.x_ = 0;
    tp.y_ = 1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;
    inp_params->twist_.xdot_ = 0.0;
    inp_params->twist_.ydot_ = 0.0;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;
    init_params->range_ = 0.0;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator();

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
        inp_params->twist_ = rv.twist_;
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
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";


    tp.x_ = 3;
    tp.y_ = 3;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;
    inp_params->twist_.xdot_ = 0.0;
    inp_params->twist_.ydot_ = 0.0;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;
    init_params->range_ = 0.0;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator();

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
        inp_params->twist_ = rv.twist_;
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
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";


    tp.x_ = 3;
    tp.y_ = 1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;
    inp_params->twist_.xdot_ = 0.0;
    inp_params->twist_.ydot_ = 0.0;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;
    init_params->range_ = 0.0;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator();

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
        inp_params->twist_ = rv.twist_;
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
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";


    tp.x_ = 2;
    tp.y_ = 1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;
    inp_params->twist_.xdot_ = 0.0;
    inp_params->twist_.ydot_ = 0.0;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;
    init_params->range_ = 0.0;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator();

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
        inp_params->twist_ = rv.twist_;
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
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";


    tp.x_ = 3;
    tp.y_ = 2;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;
    inp_params->twist_.xdot_ = 0.0;
    inp_params->twist_.ydot_ = 0.0;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;
    init_params->range_ = 0.0;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator();

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
        inp_params->twist_ = rv.twist_;
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
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";


    tp.x_ = -3;
    tp.y_ = 2;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;
    inp_params->twist_.xdot_ = 0.0;
    inp_params->twist_.ydot_ = 0.0;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;
    init_params->range_ = 0.0;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator();

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
        inp_params->twist_ = rv.twist_;
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
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";


    tp.x_ = 1;
    tp.y_ = -1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;
    inp_params->twist_.xdot_ = 0.0;
    inp_params->twist_.ydot_ = 0.0;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;
    init_params->range_ = 0.0;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator();

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED && !arrive(tp.x_, rv.int_pose_.x_) && !arrive(tp.y_, rv.int_pose_.y_))
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
        inp_params->twist_ = rv.twist_;
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
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";


    tp.x_ = 1;
    tp.y_ = 1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;
    inp_params->twist_.xdot_ = 0.0;
    inp_params->twist_.ydot_ = 0.0;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;
    init_params->range_ = 0.025;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator();

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
        inp_params->twist_ = rv.twist_;
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
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";


    tp.x_ = 3;
    tp.y_ = 1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;
    inp_params->twist_.xdot_ = 0.0;
    inp_params->twist_.ydot_ = 0.0;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;
    init_params->range_ = 0.025;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator();

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
        inp_params->twist_ = rv.twist_;
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
    rp.x_ = 0;
    rp.y_ = 0;

    rp.reference_ = "/base_link";


    tp.x_ = 1;
    tp.y_ = -1;

    tp.reference_ = "/base_link";

    struct interpolator_2d_params *inp_params = new interpolator_2d_params();
    inp_params->robot_pose_ = rp;
    inp_params->target_pose_ = tp;
    inp_params->twist_.xdot_ = 0.0;
    inp_params->twist_.ydot_ = 0.0;

    struct interpolator_2d_init_params *init_params = new interpolator_2d_init_params();
    init_params->cycle_time_ =  0.1;
    init_params->vel_limit_ =  0.2;
    init_params->acc_limit_ =  0.2;
    init_params->jerk_limit_ =  0.2;
    init_params->range_ = 0.025;

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator();

    struct interpolator_2d_result rv;

    interpolator->init(*init_params);
    while (rv.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        rv = interpolator->interpolate(*inp_params);
        inp_params->robot_pose_ = rv.int_pose_;
        inp_params->twist_ = rv.twist_;
    }

    ROS_INFO_STREAM("x: " << inp_params->robot_pose_.x_ << " x_target: " << tp.x_ << " = " << arrive(tp.x_, inp_params->robot_pose_.x_));
    ROS_INFO_STREAM("y: " << inp_params->robot_pose_.y_ << " y_target: " << tp.y_ << " = " << arrive(tp.y_, inp_params->robot_pose_.y_));

    ASSERT_TRUE(rv.result_value_ == ReflexxesAPI::RML_FINAL_STATE_REACHED);

    SUCCEED();
}