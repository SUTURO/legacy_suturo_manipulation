#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl/jntarray.hpp>
#include <ReflexxesAPI.h>
#include <suturo_manipulation_test_reflexxes_interpolator.h>

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "test_arrive_x_y_at_once");
    // ros::NodeHandle n;
    return RUN_ALL_TESTS();
}

TEST(suturo_manipulation_move_robot, arrive_1_1_at_once)
{

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    robotPose.pose.position.x = 0;
    robotPose.pose.position.y = 0;
    robotPose.pose.position.z = 0;
    robotPose.pose.orientation.x = 0;
    robotPose.pose.orientation.y = 0;
    robotPose.pose.orientation.z = 0;
    robotPose.pose.orientation.w = 1;
    robotPose.header.frame_id = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    targetPose.pose.position.x = 1;
    targetPose.pose.position.y = 1;
    targetPose.pose.position.z = 0;
    targetPose.pose.orientation.x = 0;
    targetPose.pose.orientation.y = 0;
    targetPose.pose.orientation.z = 0;
    targetPose.pose.orientation.w = 1;
    targetPose.header.frame_id = "/base_link";

    Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

    ASSERT_TRUE(interpolator->interpolate(robotPose, targetPose));

    // ASSERT_TRUE(1 == 1);

    SUCCEED();
}


TEST(suturo_manipulation_move_robot, arrive_2_2_at_once)
{

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    robotPose.pose.position.x = 0;
    robotPose.pose.position.y = 0;
    robotPose.pose.position.z = 0;
    robotPose.pose.orientation.x = 0;
    robotPose.pose.orientation.y = 0;
    robotPose.pose.orientation.z = 0;
    robotPose.pose.orientation.w = 1;
    robotPose.header.frame_id = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    targetPose.pose.position.x = 2;
    targetPose.pose.position.y = 2;
    targetPose.pose.position.z = 0;
    targetPose.pose.orientation.x = 0;
    targetPose.pose.orientation.y = 0;
    targetPose.pose.orientation.z = 0;
    targetPose.pose.orientation.w = 1;
    targetPose.header.frame_id = "/base_link";

    Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

    ASSERT_TRUE(interpolator->interpolate(robotPose, targetPose));

    // ASSERT_TRUE(1 == 1);

    SUCCEED();
}


TEST(suturo_manipulation_move_robot, arrive_1_4_at_once)
{

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    robotPose.pose.position.x = 0;
    robotPose.pose.position.y = 0;
    robotPose.pose.position.z = 0;
    robotPose.pose.orientation.x = 0;
    robotPose.pose.orientation.y = 0;
    robotPose.pose.orientation.z = 0;
    robotPose.pose.orientation.w = 1;
    robotPose.header.frame_id = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    targetPose.pose.position.x = 1;
    targetPose.pose.position.y = 4;
    targetPose.pose.position.z = 0;
    targetPose.pose.orientation.x = 0;
    targetPose.pose.orientation.y = 0;
    targetPose.pose.orientation.z = 0;
    targetPose.pose.orientation.w = 1;
    targetPose.header.frame_id = "/base_link";

    Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

    ASSERT_TRUE(interpolator->interpolate(robotPose, targetPose));

    // ASSERT_TRUE(1 == 1);

    SUCCEED();
}


TEST(suturo_manipulation_move_robot, arrive_4_1_at_once)
{

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    robotPose.pose.position.x = 0;
    robotPose.pose.position.y = 0;
    robotPose.pose.position.z = 0;
    robotPose.pose.orientation.x = 0;
    robotPose.pose.orientation.y = 0;
    robotPose.pose.orientation.z = 0;
    robotPose.pose.orientation.w = 1;
    robotPose.header.frame_id = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    targetPose.pose.position.x = 4;
    targetPose.pose.position.y = 1;
    targetPose.pose.position.z = 0;
    targetPose.pose.orientation.x = 0;
    targetPose.pose.orientation.y = 0;
    targetPose.pose.orientation.z = 0;
    targetPose.pose.orientation.w = 1;
    targetPose.header.frame_id = "/base_link";

    Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

    ASSERT_TRUE(interpolator->interpolate(robotPose, targetPose));

    // ASSERT_TRUE(1 == 1);

    SUCCEED();
}

TEST(suturo_manipulation_move_robot, arrive_1_neg1_at_once)
{

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    robotPose.pose.position.x = 0;
    robotPose.pose.position.y = 0;
    robotPose.pose.position.z = 0;
    robotPose.pose.orientation.x = 0;
    robotPose.pose.orientation.y = 0;
    robotPose.pose.orientation.z = 0;
    robotPose.pose.orientation.w = 1;
    robotPose.header.frame_id = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    targetPose.pose.position.x = 1;
    targetPose.pose.position.y = -1;
    targetPose.pose.position.z = 0;
    targetPose.pose.orientation.x = 0;
    targetPose.pose.orientation.y = 0;
    targetPose.pose.orientation.z = 0;
    targetPose.pose.orientation.w = 1;
    targetPose.header.frame_id = "/base_link";

    Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

    ASSERT_TRUE(interpolator->interpolate(robotPose, targetPose));

    // ASSERT_TRUE(1 == 1);

    SUCCEED();
}

TEST(suturo_manipulation_move_robot, arrive_0_0_at_once)
{

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    robotPose.pose.position.x = 0;
    robotPose.pose.position.y = 0;
    robotPose.pose.position.z = 0;
    robotPose.pose.orientation.x = 0;
    robotPose.pose.orientation.y = 0;
    robotPose.pose.orientation.z = 0;
    robotPose.pose.orientation.w = 1;
    robotPose.header.frame_id = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    targetPose.pose.position.x = 0;
    targetPose.pose.position.y = 0;
    targetPose.pose.position.z = 0;
    targetPose.pose.orientation.x = 0;
    targetPose.pose.orientation.y = 0;
    targetPose.pose.orientation.z = 0;
    targetPose.pose.orientation.w = 1;
    targetPose.header.frame_id = "/base_link";

    Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

    ASSERT_TRUE(interpolator->interpolate(robotPose, targetPose));

    // ASSERT_TRUE(1 == 1);

    SUCCEED();
}

TEST(suturo_manipulation_move_robot, arrive_0_1_at_once)
{

    // Set start and goal
    geometry_msgs::PoseStamped robotPose;
    robotPose.pose.position.x = 0;
    robotPose.pose.position.y = 0;
    robotPose.pose.position.z = 0;
    robotPose.pose.orientation.x = 0;
    robotPose.pose.orientation.y = 0;
    robotPose.pose.orientation.z = 0;
    robotPose.pose.orientation.w = 1;
    robotPose.header.frame_id = "/base_link";

    geometry_msgs::PoseStamped targetPose;
    targetPose.pose.position.x = 0;
    targetPose.pose.position.y = 1;
    targetPose.pose.position.z = 0;
    targetPose.pose.orientation.x = 0;
    targetPose.pose.orientation.y = 0;
    targetPose.pose.orientation.z = 0;
    targetPose.pose.orientation.w = 1;
    targetPose.header.frame_id = "/base_link";

    Suturo_Manipulation_Test_Reflexxes_Interpolator *interpolator = new Suturo_Manipulation_Test_Reflexxes_Interpolator();

    ASSERT_FALSE(interpolator->interpolate(robotPose, targetPose));

    // ASSERT_TRUE(1 == 1);

    SUCCEED();
}