#include "suturo_manipulation_grasping.h"

using namespace std;

const string Grasping::RIGHT_ARM = suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM;
const string Grasping::LEFT_ARM = suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM;

Grasping::Grasping(ros::NodeHandle *nh, Suturo_Manipulation_Planning_Scene_Interface *pi, ros::Publisher *head_publisher)
{
    //initialize private variables
    group_r_arm_ = new move_group_interface::MoveGroup(RIGHT_ARM);
    group_r_arm_->setPlanningTime(5.0);


    group_l_arm_ = new move_group_interface::MoveGroup(LEFT_ARM);
    group_l_arm_->setPlanningTime(5.0);

    head_publisher_ = head_publisher;
    r_gripper_ = new Gripper(RIGHT_ARM);
    l_gripper_ = new Gripper(LEFT_ARM);

    //pi nicht selbst erstellen, weil das Weiterreichen des nodehandle über 2 Klassen rumbugt :(
    pi_ = pi;
    nh_ = nh;

    grasp_calculator_ = new Grasp_Calculator(pi);

    //wait because ros
    ros::WallDuration(0.5).sleep();
}

Grasping::~Grasping()
{

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

int Grasping::lookAt(geometry_msgs::PoseStamped pose)
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
    if ( !head_publisher_ )
    {
        ROS_INFO("Publisher invalid!\n");
    }
    else
    {
        head_publisher_->publish(goal_msg);
        ROS_INFO_STREAM("current Position: x=" << goal_msg.goal.target.point.x <<
                        ", y=" << goal_msg.goal.target.point.y <<
                        ", z=" << goal_msg.goal.target.point.z <<
                        " in Frame " << goal_msg.goal.pointing_frame.c_str());
    }
    return 1;
}

int Grasping::move(move_group_interface::MoveGroup *move_group,
                   geometry_msgs::PoseStamped desired_pose,
                   moveit_msgs::CollisionObject co)
{
    //look
    lookAt(desired_pose);
    //set marker
    pi_->publishMarker(desired_pose);
    //set goal
    move_group->setPoseTarget(desired_pose);
    //move
    return move_group->move() ? 1 : 0;
}

bool Grasping::get_move_group(std::string arm, move_group_interface::MoveGroup *&move_group)
{
    if (arm == RIGHT_ARM)
    {
        move_group = new move_group_interface::MoveGroup(*group_r_arm_);
    }
    else if (arm == LEFT_ARM)
    {
        move_group = new move_group_interface::MoveGroup(*group_l_arm_);
    }
    else
    {
        ROS_ERROR_STREAM("Arm value not valide.");
        return false;
    }
    return true;
}

bool Grasping::get_gripper(std::string arm, Gripper *&gripper)
{
    if (arm == RIGHT_ARM)
    {
        gripper = r_gripper_;
    }
    else if (arm == LEFT_ARM)
    {
        gripper = l_gripper_;
    }
    else
    {
        ROS_ERROR_STREAM("Arm value not valide.");
        return false;
    }
    return true;
}

int Grasping::pick(moveit_msgs::CollisionObject co, std::string arm,
                   std::vector<geometry_msgs::PoseStamped> &poses,
                   std::vector<geometry_msgs::PoseStamped> &pre_poses,
                   double force)
{
    string object_name = co.id;

    //select movegroup depending on arm
    move_group_interface::MoveGroup *move_group;
    Gripper *gripper;
    if (!get_move_group(arm, move_group) || !get_gripper(arm, gripper)) return 0;

    //search for valid pregraspposition and move there
    int pos_id = 0;
    while (pos_id < pre_poses.size())
    {
        ROS_INFO_STREAM("Try to move to pregraspposition #" << pos_id);
        if (!move(move_group, pre_poses.at(pos_id), co))
        {
            pos_id++;
            continue;
        }

        //open gripper
        gripper->open_gripper(force);

        //move Arm to goalpose
        ROS_INFO_STREAM("move to goalpose");

        if (!move(move_group, poses.at(pos_id), co))
        {
            pos_id++;
            continue;
        }
        else
        {
            //close gripper
            gripper->close_gripper(force);

            //update collisionobject in planningscene
            if (!pi_->addObject(co))
                return 0;

            //attach object
            ROS_INFO_STREAM("attach object");
            if (!pi_->attachObject(object_name, move_group->getEndEffectorLink(),
                                   gripper->get_gripper_links()))
                return 0;

            ROS_INFO_STREAM("\n\n Picking finished \n");
            return 1;
        }

    }

    if (pos_id == pre_poses.size())
    {
        ROS_ERROR_STREAM("No graspposition reachable for " << object_name);
        return 0;
    }

    return 0;
}

int Grasping::pick(std::string object_name, std::string arm, double force)
{
    //check if there is an object attached to this arm.
    moveit_msgs::CollisionObject co;
    if (int r = get_attached_object(arm, object_name, co) != 2)
    {
        return r;
    }

    std::vector<geometry_msgs::PoseStamped> poses(0);
    std::vector<geometry_msgs::PoseStamped> pre_poses(0);

    Gripper *gripper;

    //calculate graspposition(s)
    return get_gripper(arm, gripper) && grasp_calculator_->calcGraspPosition(co, poses, pre_poses, gripper->get_gripper_palm_length()) ?
           pick(co, arm, poses, pre_poses, force) : 0;
}

int Grasping::pick_above(std::string object_name, std::string arm, double tolerance, double force)
{
    //check if there is an object attached to this arm.
    moveit_msgs::CollisionObject co;
    if (int r = get_attached_object(arm, object_name, co) != 2)
    {
        return r;
    }

    std::vector<geometry_msgs::PoseStamped> poses_tmp(0);
    std::vector<geometry_msgs::PoseStamped> pre_poses_tmp(0);

    //calculate graspposition(s)
    Gripper *gripper;
    if (!get_gripper(arm, gripper) || !grasp_calculator_->calcGraspPosition(co, poses_tmp, pre_poses_tmp, gripper->get_gripper_palm_length()))
    {
        return 0;
    }

    std::vector<geometry_msgs::PoseStamped> poses(0);
    std::vector<geometry_msgs::PoseStamped> pre_poses(0);

    geometry_msgs::PointStamped p = grasp_calculator_->get_point_above_object(co.id);

    for (int i = 0; i < poses_tmp.size(); i++)
    {
        if (Grasp_Calculator::get_angle(poses_tmp[i].pose.position, p.point) <= tolerance)
        {
            poses.push_back(poses_tmp[i]);
            pre_poses.push_back(pre_poses_tmp[i]);
        }

    }


    return pick(co, arm, poses, pre_poses, force);
}

int Grasping::get_attached_object(std::string arm, std::string object_name, moveit_msgs::CollisionObject &co)
{
    moveit_msgs::AttachedCollisionObject aco;
    if (arm == LEFT_ARM && pi_->isAnObjectAttachedToArm(group_l_arm_->getEndEffectorLink(), aco)
            || arm == RIGHT_ARM && pi_->isAnObjectAttachedToArm(group_r_arm_->getEndEffectorLink(), aco))
    {
        ROS_WARN_STREAM(co.id << " already attached to this arm.");
        return 1;
    }
    else if (pi_->getAttachedObject(object_name, aco))
    {
        //object is in other hand
        co = aco.object;
    }
    else if (!pi_->getObject(object_name, co))
    {
        //object not found
        return 0;
    }
    return 2;
}

int Grasping::dropObject(string object_name)
{
    //get object from planningscene
    moveit_msgs::AttachedCollisionObject aco;
    if (!pi_->getAttachedObject(object_name, aco))
    {
        ROS_INFO_STREAM(object_name << " not attached.");
        return 1;
    }

    if (aco.link_name == group_r_arm_->getEndEffectorLink())
    {
        r_gripper_->open_gripper();
    }
    else if (aco.link_name == group_l_arm_->getEndEffectorLink())
    {
        l_gripper_->open_gripper();
    }

    //detach object
    pi_->detachObject(object_name);
    ROS_INFO_STREAM("\n\n Droped " << object_name << " successfully.\n");
    return 1;
}

int Grasping::drop(string arm)
{
    move_group_interface::MoveGroup *move_group;
    Gripper *gripper;
    if (!get_move_group(arm, move_group) || !get_gripper(arm, gripper)) return 0;

    //choose endeffector depending on arm
    std::string eof = move_group->getEndEffectorLink();
    gripper->open_gripper();

    //detach any attached objects
    std::vector<moveit_msgs::AttachedCollisionObject> acos;
    if (!pi_->getAttachedObjects(acos)) return 0;

    for (std::vector<moveit_msgs::AttachedCollisionObject>::iterator it = acos.begin(); it != acos.end(); ++it)
    {
        if (it->link_name == eof)
        {
            pi_->detachObject(it->object.id);
        }
    }

    return 1;
}

