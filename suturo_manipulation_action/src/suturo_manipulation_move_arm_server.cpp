/**
* This class implements the action server to move an selected arm.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_moveAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <tf/transform_listener.h>
#include <control_msgs/PointHeadActionGoal.h>
#include <pr2_controllers_msgs/PointHeadActionResult.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <suturo_manipulation_grasping.h>


using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_moveAction> Server;

tf::TransformListener *listener = NULL;

//~ control_msgs::PointHeadActionGoal goal_msg;

/**
 * Transform the incoming frame to /base_link
 */
int transform(geometry_msgs::PoseStamped &goalPose)
{
    // goal_frame
    const string goalFrame = "/base_link";

    // ROS_INFO("Beginn der Transformation von %s zu /base_link", s);
    try
    {
        //transform pose from s to base_link and save it in pose again
        listener->transformPose(goalFrame, goalPose, goalPose);
    }
    catch (...)
    {
        ROS_INFO("ERROR: Transformation failed.");
        return 0;
    }

    return 1;
}

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

int lookAt(geometry_msgs::PoseStamped pose, ros::Publisher *head_publisher)
{
    // Goal Message to move the head
    control_msgs::PointHeadActionGoal goal_msg;

    goal_msg.header.seq = 1;
    goal_msg.header.stamp = ros::Time::now();
    // Set Goal to pre grasp position
    goal_msg.header.frame_id =  pose.header.frame_id;
    goal_msg.goal_id.stamp = goal_msg.header.stamp;
    // set unique id with timestamp
    goal_msg.goal_id.id = "goal_" + time_to_str(goal_msg.header.stamp);
    goal_msg.goal.target.header = goal_msg.header;
    // Set position from pre grasp
    goal_msg.goal.target.point = pose.pose.position;
    goal_msg.goal.pointing_axis.x = 1;
    goal_msg.goal.pointing_axis.y = 0;
    goal_msg.goal.pointing_axis.z = 0;
    goal_msg.goal.pointing_frame = "head_plate_frame";
    goal_msg.goal.min_duration = ros::Duration(1.0);
    goal_msg.goal.max_velocity = 10;

    // Publish goal on topic /suturo/head_controller
    if ( !head_publisher )
    {
        ROS_INFO("Publisher invalid!\n");
    }
    else
    {
        head_publisher->publish(goal_msg);
        ROS_INFO_STREAM("current Position: x=" << goal_msg.goal.target.point.x <<
                        ", y=" << goal_msg.goal.target.point.y <<
                        ", z=" << goal_msg.goal.target.point.z <<
                        " in Frame " << goal_msg.goal.pointing_frame.c_str());
    }
    return 1;
}

/**
* This method starts the transformation to the right frame and
* calls move() to move the selected arm.
*/
void moveArm(const suturo_manipulation_msgs::suturo_manipulation_moveGoalConstPtr &goal, ros::NodeHandle *nh, ros::Publisher *publisher, Server *server_arm)
{

    // create a move result message
    suturo_manipulation_msgs::suturo_manipulation_moveResult r;

    // Set header
    r.succ.header.stamp = ros::Time();
    // Set Answer fot planning to undefined
    r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;

    // set Planning Interface and Grasper
    ROS_INFO("Create Planning Scene Interface...");
    Suturo_Manipulation_Planning_Scene_Interface pi(nh);
    ROS_INFO("Done. Create Grasper...") ;
    Grasping grasper(&pi);
    ROS_INFO("Done.");

    // Set arm which should be moved
    string arm = goal->bodypart.bodyPart;
    if (arm != suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM && arm != suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM)
    {
        ROS_INFO("Unknown arm! Please use suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM or suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM as names!\n");
        r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
        server_arm->setAborted(r);
    }

    ROS_INFO("received arm: %s, x: %f, y: %f, z: %f", arm.c_str(), goal->ps.pose.position.x,
             goal->ps.pose.position.y, goal->ps.pose.position.z);

    //tranform pose
    geometry_msgs::PoseStamped transformedPose = goal->ps;;
    if (!transform(transformedPose))
    {
        ROS_INFO("Transformation failed!\n");
        // If tranfsormation fails, update the answer for planning to "FAIL" and set the server aborted
        r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
        server_arm->setAborted(r);
    }
    // transformedPose = goal->ps;
    ROS_INFO("transformed to x: %f, y: %f, z: %f in Frame %s", transformedPose.pose.position.x,
             transformedPose.pose.position.y, transformedPose.pose.position.z, transformedPose.header.frame_id.c_str());

    // set group to move
    move_group_interface::MoveGroup group(arm);

    // set Pose
    pi.publishMarker(transformedPose);
    group.setPoseTarget(transformedPose);
    ROS_INFO_STREAM("current Position: x=" << group.getCurrentPose().pose.position.x <<
                    ", y=" << group.getCurrentPose().pose.position.y <<
                    ", z=" << group.getCurrentPose().pose.position.z <<
                    " in Frame " << group.getCurrentPose().header.frame_id.c_str());

    lookAt(transformedPose, publisher);

    //move arm
    if (group.move())
    {
        ROS_INFO("Arm moved!\n");
        r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
        server_arm->setSucceeded(r);
    }
    else
    {
        ROS_INFO("Moving failed!\n");
        r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
        server_arm->setAborted(r);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "suturo_manipulation_move_arm_server");
    ros::NodeHandle n;
    listener = new (tf::TransformListener);

    // Publish a topic for the head controller
    // ros::Publisher head_publisher = n.advertise<geometry_msgs::PoseStamped>("/suturo/head_controller_goal_point", 1000);
    ros::Publisher head_publisher = n.advertise<control_msgs::PointHeadActionGoal>("/head_traj_controller/point_head_action/goal", 1000);

    // create the action server

    Server server_arm(n, "suturo_man_move_arm_server", boost::bind(&moveArm, _1, &n, &head_publisher, &server_arm), false);

    // start the server
    server_arm.start();

    ROS_INFO("Ready to move the arms!.");
    ros::spin();
    return 0;
}
