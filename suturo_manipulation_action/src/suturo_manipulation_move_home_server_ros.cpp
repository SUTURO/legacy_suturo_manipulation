

/**

* This class implements the action server to move a

* part of the robot (left_arm, right_arm, head)

* to a definied home position.

*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_homeAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <control_msgs/PointHeadActionGoal.h>
#include <pr2_controllers_msgs/PointHeadActionResult.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <tf/transform_listener.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_homeAction> Server;

const string left_arm_group = "left_arm";
const string right_arm_group = "right_arm";
const string both_arms_group = "both_arms";
const string arms_head_group = "arms_head";

control_msgs::PointHeadActionGoal goal_msg;

Suturo_Manipulation_Planning_Scene_Interface *pi_;

bool moved = 0;

template <class T>
/**
* This method formats a ros time to a string.
* Thanks to https://code.ros.org/trac/ros/ticket/2030
*/
std::string time_to_str(T ros_t)
{
    char buf[1024]      = "";
    time_t t = ros_t.sec;
    struct tm *tms = localtime(&t);
    strftime(buf, 1024, "%Y-%m-%d-%H-%M-%S", tms);
    return std::string(buf);
}

/**
* This method is a callback method and checks, if the head moved.
*/
void moveHeadResult(pr2_controllers_msgs::PointHeadActionResult msg)
{
    if (goal_msg.goal_id.id == msg.status.goal_id.id)
    {
        // ROS_INFO("Get result!");
        moved = 1;
    }
    else
    {
        // ROS_INFO("No result!");
        moved = 0;
    }
}

int move_object_to_cam(move_group_interface::MoveGroup *move_group)
{
    //look
    // lookAt(desired_pose);


    moveit_msgs::AttachedCollisionObject aco;
    //get attached object
    if (!pi_->isAnObjectAttachedToArm("l_wrist_roll_link", aco))
    {
        ROS_INFO_STREAM("Can't move object to webcam: no object attached to the left arm.");
        return 0;
    }


    //calc pose depending on objectposition
    tf::TransformListener listener;
    ros::WallDuration(1.0).sleep();
    geometry_msgs::PoseStamped aco_in_wrist_frame;
    aco_in_wrist_frame.header.frame_id = "/" + aco.object.id;
    aco_in_wrist_frame.pose.orientation.w = 1;

    try
    {
        //transform pose from s to base_link and save it in pose again
        // ROS_INFO_STREAM(aco_in_wrist_frame);
        listener.transformPose("/" + move_group->getEndEffectorLink(), aco_in_wrist_frame, aco_in_wrist_frame);
        // ROS_INFO_STREAM(aco_in_wrist_frame);
    }
    catch (...)
    {
        ROS_INFO("ERROR: Transformation failed.");
        return 0;
    }

    geometry_msgs::PoseStamped desired_pose;
    desired_pose.header.frame_id = "/webcam";
    desired_pose.pose.position.x = 0.27;
    desired_pose.pose.position.y = aco_in_wrist_frame.pose.position.x;
    desired_pose.pose.position.z = 0;
    desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI_2);


    //set marker
    pi_->publishMarker(desired_pose);
    //set goal
    move_group->setPoseTarget(desired_pose);
    //move
    return move_group->move() ? 1 : 0;
    // return 0;
}

int move_arm_in_front_of_robot(move_group_interface::MoveGroup *move_group, double y_oofset)
{
    //look
    // lookAt(desired_pose);


    //calc pose depending on objectposition
    tf::TransformListener listener;
    ros::WallDuration(1.0).sleep();
    geometry_msgs::PoseStamped desired_pose;
    desired_pose.header.frame_id = "/torso_lift_link";
    desired_pose.pose.position.x = 0.45;
    desired_pose.pose.position.y = y_oofset;
    desired_pose.pose.position.z = 0;
    desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI_2);

    // try
    // {
    //     //transform pose from s to base_link and save it in pose again
    //     // ROS_INFO_STREAM(aco_in_wrist_frame);
    //     listener.transformPose("/" + move_group->getEndEffectorLink(), aco_in_wrist_frame, aco_in_wrist_frame);
    //     // ROS_INFO_STREAM(aco_in_wrist_frame);
    // }
    // catch (...)
    // {
    //     ROS_INFO("ERROR: Transformation failed.");
    //     return 0;
    // }

    // geometry_msgs::PoseStamped desired_pose;
    // desired_pose.header.frame_id = "/webcam";
    // desired_pose.pose.position.x = 0.23;
    // desired_pose.pose.position.y = aco_in_wrist_frame.pose.position.x;
    // desired_pose.pose.position.z = 0;
    // desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI_2);


    //set marker
    pi_->publishMarker(desired_pose);
    //set goal
    move_group->setPoseTarget(desired_pose);
    //move
    return move_group->move() ? 1 : 0;
    // return 0;
}

/**
* This method moves the given bodypart to the homeposition.
*/
void moveHome(const suturo_manipulation_msgs::suturo_manipulation_homeGoalConstPtr &goal,
              ros::Publisher *publisher, Server *server_home)
{
    ROS_INFO("callback moveHome() begins...");

    // create a move result message
    suturo_manipulation_msgs::suturo_manipulation_homeResult r;
    // Set part which should be go home
    string body_part = goal->bodypart.bodyPart;
    ROS_INFO("Bodypart: %s", body_part.c_str());
    // Set header
    r.succ.header.stamp = ros::Time();
    // Set Answer for planning to undefined
    r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;

    if (body_part == suturo_manipulation_msgs::RobotBodyPart::HEAD)
    {
        // bodypart = head
        ROS_INFO("Move head home!");

        // Set home Coordinates, time and frame
        goal_msg.header.seq = 1;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "/base_link";
        goal_msg.goal_id.stamp = goal_msg.header.stamp;
        // set unique id with timestamp
        goal_msg.goal_id.id = "goal_" + time_to_str(goal_msg.header.stamp);
        goal_msg.goal.target.header = goal_msg.header;

        //HIER PATRICK!!!!
        goal_msg.goal.target.point.x = 1;
        goal_msg.goal.target.point.y = 0;
        goal_msg.goal.target.point.z = 0;
        //HIER PATRICK!!!!


        goal_msg.goal.pointing_axis.x = 1;
        goal_msg.goal.pointing_axis.y = 0;
        goal_msg.goal.pointing_axis.z = 0;
        goal_msg.goal.pointing_frame = "head_plate_frame";
        goal_msg.goal.min_duration = ros::Duration(1.0);
        goal_msg.goal.max_velocity = 10;

        // Publish goal on topic /suturo/head_controller
        if ( !publisher )
        {
            ROS_INFO("Publisher invalid!\n");
            r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
            server_home->setAborted(r);
        }
        else
        {
            publisher->publish(goal_msg);
            ROS_INFO("Home Goal published!");
            ros::WallDuration(2.0).sleep();
            if (moved == 1)
            {
                ROS_INFO("Head moved!\n");
                r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
                server_home->setSucceeded(r);
            }
            else
            {
                ROS_INFO("Head doesn't move!\n");
                r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
                server_home->setAborted(r);
            }
        }
    }
    else if (body_part == suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM_CAM_POSE)
    {
        if (move_object_to_cam(new move_group_interface::MoveGroup(left_arm_group)))
        {
            r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
            server_home->setSucceeded(r);
        }
        else
        {
            r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
            server_home->setAborted(r);
        }
    }
    else if ((body_part == suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM_PASS_OVER))
    {
        if (move_arm_in_front_of_robot(new move_group_interface::MoveGroup(right_arm_group), -0.15))
        {
            r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
            server_home->setSucceeded(r);
        }
        else
        {
            r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
            server_home->setAborted(r);
        }        
    }
    else if ((body_part == suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM_PASS_OVER))
    {
        if (move_arm_in_front_of_robot(new move_group_interface::MoveGroup(left_arm_group), 0.15))
        {
            r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
            server_home->setSucceeded(r);
        }
        else
        {
            r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
            server_home->setAborted(r);
        }        
    }    
    else
    {

        ROS_INFO_STREAM("Move to " << body_part.c_str());

        // set group to move
        move_group_interface::MoveGroup *group;

        if (body_part == suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM)
        {
            group = new move_group_interface::MoveGroup(left_arm_group);

        }
        else if (body_part == suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM)
        {

            group = new move_group_interface::MoveGroup(right_arm_group);

        }
        else if (body_part == suturo_manipulation_msgs::RobotBodyPart::BOTH_ARMS
                 || body_part == suturo_manipulation_msgs::RobotBodyPart::BOTH_ARMS_MOVE)
        {

            group = new move_group_interface::MoveGroup(both_arms_group);

        }
        else if (body_part == suturo_manipulation_msgs::RobotBodyPart::ALL)
        {

            group = new move_group_interface::MoveGroup(arms_head_group);

        }
        else
        {
            ROS_INFO("Unknown bodypart!\n");
            r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
            server_home->setAborted(r);
            return;
        }

        // set target pose
        group->setNamedTarget(body_part + "_home");

        //move bodypart
        if (group->move())
        {
            ROS_INFO("Moved into Position!\n");
            r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
            server_home->setSucceeded(r);
        }
        else
        {
            ROS_INFO("...or not\n");
            r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
            server_home->setAborted(r);
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "suturo_manipulation_move_home_server");
    ros::NodeHandle n;

    pi_ = new Suturo_Manipulation_Planning_Scene_Interface(&n);

    // Publish a topic for the head controller
    // ros::Publisher head_publisher = n.advertise<geometry_msgs::PoseStamped>("/suturo/head_controller_goal_point", 1000);

    // Subscribe the topic, call moveHeadResult if data is published
    ros::Subscriber sub = n.subscribe("/head_traj_controller/point_head_action/result", 1000, moveHeadResult);

    // Publish a topic for the ros intern head controller
    ros::Publisher head_publisher = n.advertise<control_msgs::PointHeadActionGoal>("/head_traj_controller/point_head_action/goal", 1000);


    // create the action server
    Server server_home(n, "suturo_man_move_home_server", boost::bind(&moveHome, _1, &head_publisher, &server_home), false);
    // start the server
    server_home.start();

    ROS_INFO("Ready to go home!");
    ros::spin();
    return 0;
}
