#include "suturo_manipulation_grasping.h"

using namespace std;

const string Grasping::RIGHT_ARM = suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM;
const string Grasping::LEFT_ARM = suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM;

Grasping::Grasping(Suturo_Manipulation_Planning_Scene_Interface *pi, ros::Publisher *head_publisher)
{
    //initialize private variables
    group_r_arm_ = new move_group_interface::MoveGroup(RIGHT_ARM);
    group_r_arm_->setPlanningTime(5.0);

    group_l_arm_ = new move_group_interface::MoveGroup(LEFT_ARM);
    group_l_arm_->setPlanningTime(5.0);

    head_publisher_ = head_publisher;
    gripper_ = new Gripper();

    //pi nicht selbst erstellen, weil das Weiterreichen des nodehandle über 2 Klassen rumbugt :(
    pi_ = pi;

    //wait because ros
    ros::WallDuration(0.5).sleep();
}

Grasping::~Grasping()
{

}

void Grasping::addGraspPositionsZ(double d, double rotation, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses,
                                  std::vector<geometry_msgs::PoseStamped> &pre_poses)
{
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pre_pose;

    pose.header.frame_id = frame_id;
    pre_pose.header.frame_id = frame_id;

    //grasp from above
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, rotation);

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = Gripper::GRIPPER_DEPTH + d;
    pre_pose = pose;
    pre_pose.pose.position.z += Gripper::GRIPPER_DEPTH - 0.05;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);


    //grasp from below
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI_2, rotation);

    pose.pose.position.z = 0 - Gripper::GRIPPER_DEPTH - d;
    pre_pose = pose;
    pre_pose.pose.position.z -= Gripper::GRIPPER_DEPTH - 0.05;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);
}

void Grasping::addGraspPositionsX(double h, double d, double rotation, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses,
                                  std::vector<geometry_msgs::PoseStamped> &pre_poses)
{
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pre_pose;
    pose.header.frame_id = frame_id;
    pre_pose.header.frame_id = frame_id;

    //grasp from the front
    pose.pose.position.x = 0 - Gripper::GRIPPER_DEPTH - d - 0.005;
    pose.pose.position.y = 0;
    pose.pose.position.z = h;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, 0);

    pre_pose = pose;
    pre_pose.pose.position.x -= Gripper::GRIPPER_DEPTH;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);

    //grasp from behind
    pose.pose.position.x = Gripper::GRIPPER_DEPTH + d + 0.005;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, M_PI);

    pre_pose = pose;
    pre_pose.pose.position.x += Gripper::GRIPPER_DEPTH;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);
}

void Grasping::addGraspPositionsY(double h, double d, double rotation, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses,
                                  std::vector<geometry_msgs::PoseStamped> &pre_poses)
{
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pre_pose;
    pose.header.frame_id = frame_id;
    pre_pose.header.frame_id = frame_id;

    //grasp from the left
    pose.pose.position.x = 0;
    pose.pose.position.y = Gripper::GRIPPER_DEPTH + d + 0.005;
    pose.pose.position.z = h;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, -M_PI_2);

    pre_pose = pose;
    pre_pose.pose.position.y += Gripper::GRIPPER_DEPTH;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);

    //grasp from right
    pose.pose.position.y = 0 - Gripper::GRIPPER_DEPTH - d - 0.005;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, M_PI_2);

    pre_pose = pose;
    pre_pose.pose.position.y -= Gripper::GRIPPER_DEPTH;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);
}

int Grasping::calcBoxGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
                                   std::vector<geometry_msgs::PoseStamped> &pre_poses)
{
    ROS_INFO_STREAM("calculate graspposition for " << co.id);

    //test if the object is a box
    if (co.primitives[0].type != shape_msgs::SolidPrimitive::BOX)
    {
        ROS_ERROR_STREAM(co.id << " is not a box.");
        return 0;
    }

    //get object size
    double x = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
    double y = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    double z = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];

    //check if the object can be grasped
    if (x > Gripper::GRIPPER_MAX_POSITION &&
            y > Gripper::GRIPPER_MAX_POSITION &&
            z > Gripper::GRIPPER_MAX_POSITION)
    {
        ROS_ERROR_STREAM("Object is to big!");
        return 0;
    }

    if (x < Gripper::GRIPPER_MAX_POSITION)
    {
        //object can be grasped from above, below, left and right
        addGraspPositionsZ(z / 2, M_PI_2, co.id, poses, pre_poses);

        //~ addBoxGraspPositionsY(y, 0, co.id, poses, pre_poses);
        addGraspPositionsY(0, y / 2, 0, co.id, poses, pre_poses);
    }
    if (y < Gripper::GRIPPER_MAX_POSITION)
    {
        //object can be grasped from above, below, front and behind
        addGraspPositionsZ(z / 2, 0, co.id, poses, pre_poses);

        addGraspPositionsX(0, x / 2, 0, co.id, poses, pre_poses);
    }
    if (z < Gripper::GRIPPER_MAX_POSITION)
    {
        //object can be grasped from the left, right, front and behind
        addGraspPositionsX(0, x / 2, M_PI_2, co.id, poses, pre_poses);

        addGraspPositionsY(0, y / 2, M_PI_2, co.id, poses, pre_poses);
    }

    transform_poses("base_link", poses);
    transform_poses("base_link", pre_poses);

    // ROS_INFO_STREAM("vorher");
    // for (int i = 0; i < poses.size(); i++)
    // {
    // 	tf::Quaternion reference_orientation = tf::createQuaternionFromRPY(0, M_PI_2, 0);

    //     tf::Quaternion tf_pose1(poses[i].pose.orientation.x,
    //                             poses[i].pose.orientation.y,
    //                             poses[i].pose.orientation.z,
    //                             poses[i].pose.orientation.w);
    //     ROS_INFO_STREAM(reference_orientation.angle(tf_pose1));
    // }
    // ROS_INFO_STREAM("nachher");
    std::sort(poses.begin(), poses.end(), &Grasping::sort_base_link_poses);
    // for (int i = 0; i < poses.size(); i++)
    // {
    // 	tf::Quaternion reference_orientation = tf::createQuaternionFromRPY(0, M_PI_2, 0);

    //     tf::Quaternion tf_pose1(poses[i].pose.orientation.x,
    //                             poses[i].pose.orientation.y,
    //                             poses[i].pose.orientation.z,
    //                             poses[i].pose.orientation.w);
    //     ROS_INFO_STREAM(reference_orientation.angle(tf_pose1));
    // }
    // for (int i = 0; i < pre_poses.size(); i++){
    //  ROS_INFO_STREAM(pre_poses[i]);
    // }
    std::sort(pre_poses.begin(), pre_poses.end(), &Grasping::sort_base_link_poses);

    return 1;
}


int Grasping::calcCylinderGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
                                        std::vector<geometry_msgs::PoseStamped> &pre_poses)
{
    ROS_INFO_STREAM("calculate graspposition for " << co.id);

    //test if object is a cylinder
    if (co.primitives[0].type != shape_msgs::SolidPrimitive::CYLINDER)
    {
        ROS_ERROR_STREAM(co.id << " is not a cylinder.");
        return 0;
    }

    //get objectsize
    double r = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
    double h = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];

    //check if the object is too big
    if (r * 2 > Gripper::GRIPPER_MAX_POSITION)
    {
        ROS_ERROR_STREAM("Object is to big!");
        return 0;
    }

    //number of height points where we can grasp
    int grasp_pose_count = (h / cylinder_safty_dist) - 1;
    for (int i = 1; i <= grasp_pose_count; i++)
    {
        addGraspPositionsX((h / 2) - (cylinder_safty_dist * i), r, 0, co.id, poses, pre_poses);
        addGraspPositionsY((h / 2) - (cylinder_safty_dist * i), r, 0, co.id, poses, pre_poses);

    }

    return 1;
}

int Grasping::calcGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
                                std::vector<geometry_msgs::PoseStamped> &pre_poses)
{
    //choose the right grasppositoncalculationfunction
    switch (co.primitives[0].type)
    {
    case shape_msgs::SolidPrimitive::CYLINDER:
        return calcCylinderGraspPosition(co, poses, pre_poses);
        break;

    case shape_msgs::SolidPrimitive::BOX:
        return calcBoxGraspPosition(co, poses, pre_poses);
        break;

    default:
        ROS_ERROR_STREAM("Can't calculate grasppositon for objecttype: " << co.primitives[0].type);
        return 0;
        break;
    }

    ROS_INFO_STREAM("Something bad happened.");
    return 0;
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

int Grasping::move(move_group_interface::MoveGroup *move_group, geometry_msgs::PoseStamped desired_pose)
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

int Grasping::pick(moveit_msgs::CollisionObject co, std::string arm,
                   std::vector<geometry_msgs::PoseStamped> &poses, std::vector<geometry_msgs::PoseStamped> &pre_poses,
                   double force)
{
    string object_name = co.id;

    //select movegroup depending on arm
    move_group_interface::MoveGroup *move_group;
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
        return 0;
    }

    //search for valid pregraspposition and move there
    int pos_id = 0;
    while (pos_id < pre_poses.size())
    {
        ROS_INFO_STREAM("Try to move to pregraspposition #" << pos_id);
        if (!move(move_group, pre_poses.at(pos_id)))
        {
            pos_id++;
            continue;
        }

        //open gripper
        double gripper_state;
        if (arm == RIGHT_ARM)
        {
            gripper_state = gripper_->open_r_gripper();
        }
        else
        {
            gripper_state = gripper_->open_l_gripper();
        }

        //move Arm to goalpose
        ROS_INFO_STREAM("move to goalpose");

        if (!move(move_group, poses.at(pos_id)))
        {
            pos_id++;
            continue;
        }
        else
        {
            //close gripper
            if (arm == RIGHT_ARM)
            {
                gripper_state = gripper_->close_r_gripper(force);
            }
            else
            {
                gripper_state = gripper_->close_l_gripper(force);
            }

            //update collisionobject in planningscene
            if (!pi_->addObject(co))
                return 0;

            //attach object
            ROS_INFO_STREAM("attach object");
            if (arm == RIGHT_ARM)
            {
                if (!pi_->attachObject(object_name, move_group->getEndEffectorLink(),
                                       Gripper::get_r_gripper_links()))
                    return 0;
            }
            else
            {
                if (!pi_->attachObject(object_name, move_group->getEndEffectorLink(),
                                       Gripper::get_l_gripper_links()))
                    return 0;
            }

            ROS_INFO_STREAM("\n\n Picking finished \n");
            return 1;
        }

    }

    if (pos_id == pre_poses.size())
    {
        ROS_ERROR_STREAM("No graspposition reachable for " << object_name);
        return 0;
    }

    return 1;
}

int Grasping::pick(std::string object_name, std::string arm, double force)
{
    if (!gripper_->is_connected_to_controller())
    {
        ROS_ERROR_STREAM("not connected to grippercontroller");
        return 0;
    }

    moveit_msgs::AttachedCollisionObject aco;
    moveit_msgs::CollisionObject co;

    //check if there is an object attached to this arm.
    if (arm == LEFT_ARM && pi_->isAnObjectAttachedToArm(group_l_arm_->getEndEffectorLink(), aco)
            || arm == RIGHT_ARM && pi_->isAnObjectAttachedToArm(group_r_arm_->getEndEffectorLink(), aco))
    {
        ROS_WARN_STREAM(co.id << " already attached to this arm.");
        return 1;
    }
    else if (pi_->getAttachedObject(object_name, aco))
    {
        co = aco.object;
    }
    else if (!pi_->getObject(object_name, co))
    {
        //object not found
        return 0;
    }

    std::vector<geometry_msgs::PoseStamped> poses(0);
    std::vector<geometry_msgs::PoseStamped> pre_poses(0);

    //calculate graspposition(s)
    if (!calcGraspPosition(co, poses, pre_poses))
    {
        return 0;
    }

    return pick(co, arm, poses, pre_poses, force);
}

int Grasping::pick_above(std::string object_name, std::string arm, double force)
{
    if (!gripper_->is_connected_to_controller())
    {
        ROS_ERROR_STREAM("not connected to grippercontroller");
        return 0;
    }

    moveit_msgs::AttachedCollisionObject aco;
    moveit_msgs::CollisionObject co;

    //check if there is an object attached to this arm.
    if (arm == LEFT_ARM && pi_->isAnObjectAttachedToArm(group_l_arm_->getEndEffectorLink(), aco)
            || arm == RIGHT_ARM && pi_->isAnObjectAttachedToArm(group_r_arm_->getEndEffectorLink(), aco))
    {
        ROS_WARN_STREAM(co.id << " already attached to this arm.");
        return 1;
    }
    else if (pi_->getAttachedObject(object_name, aco))
    {
        co = aco.object;
    }
    else if (!pi_->getObject(object_name, co))
    {
        //object not found
        return 0;
    }

    std::vector<geometry_msgs::PoseStamped> poses(0);
    std::vector<geometry_msgs::PoseStamped> pre_poses(0);

    //calculate graspposition(s)
    if (!calcGraspPosition(co, poses, pre_poses))
    {
        return 0;
    }

    return pick(co, arm, poses, pre_poses, force);
}

int Grasping::dropObject(string object_name)
{
    if (!gripper_->is_connected_to_controller())
    {
        ROS_ERROR_STREAM("Not connected to grippercontroller.");
        return 0;
    }

    //get object from planningscene
    moveit_msgs::AttachedCollisionObject aco;
    if (!pi_->getAttachedObject(object_name, aco))
    {
        ROS_INFO_STREAM(object_name << " not attached.");
        return 1;
    }

    if (aco.link_name == group_r_arm_->getEndEffectorLink())
    {
        gripper_->open_r_gripper();
    }
    else if (aco.link_name == group_l_arm_->getEndEffectorLink())
    {
        gripper_->open_l_gripper();
    }

    //detach object
    pi_->detachObject(object_name);
    ROS_INFO_STREAM("\n\n Droped " << object_name << " successfully.\n");
    return 1;
}

int Grasping::drop(string arm)
{
    if (!gripper_->is_connected_to_controller())
    {
        ROS_ERROR_STREAM("Not connected to grippercontroller.");
        return 0;
    }

    //choose endeffector depending on arm
    move_group_interface::MoveGroup *move_group;
    std::string eof;
    if (arm == RIGHT_ARM)
    {
        eof = group_r_arm_->getEndEffectorLink();
        gripper_->open_r_gripper();

    }
    else if (arm == LEFT_ARM)
    {
        eof = group_l_arm_->getEndEffectorLink();
        gripper_->open_l_gripper();

    }
    else
    {
        ROS_ERROR_STREAM("Arm value not valide.");
        return 0;
    }

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

void Grasping::transform_poses(std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses)
{
    try
    {
        //transform pose to base_link
        for (int i = 0; i < poses.size(); i++)
        {
            listener_.transformPose(frame_id, poses[i], poses[i]);

        }
    }
    catch (...)
    {
        ROS_ERROR_STREAM("ERROR: Transformation failed.");
    }
}
