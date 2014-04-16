#include "suturo_manipulation_grasp_calculator.h"

using namespace std;

Grasp_Calculator::Grasp_Calculator(Suturo_Manipulation_Planning_Scene_Interface *pi)
{
    //initialize private variables
    // group_r_arm_ = new move_group_interface::MoveGroup(RIGHT_ARM);
    // group_r_arm_->setPlanningTime(5.0);

    // group_l_arm_ = new move_group_interface::MoveGroup(LEFT_ARM);
    // group_l_arm_->setPlanningTime(5.0);

    // head_publisher_ = head_publisher;
    // gripper_ = new Gripper();

    //pi nicht selbst erstellen, weil das Weiterreichen des nodehandle über 2 Klassen rumbugt :(
    pi_ = pi;

    // //wait because ros
    // ros::WallDuration(0.5).sleep();
}

Grasp_Calculator::~Grasp_Calculator()
{

}

void Grasp_Calculator::addGraspPositionsZ(double d, double rotation, std::string frame_id,
        std::vector<geometry_msgs::PoseStamped> &poses,
        std::vector<geometry_msgs::PoseStamped> &pre_poses,
        double gripper_depth)
{
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pre_pose;

    pose.header.frame_id = frame_id;
    pre_pose.header.frame_id = frame_id;

    //grasp from above
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, rotation);

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = gripper_depth + d;
    pre_pose = pose;
    pre_pose.pose.position.z += gripper_depth - 0.05;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);


    //grasp from below
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI_2, rotation);

    pose.pose.position.z = 0 - gripper_depth - d;
    pre_pose = pose;
    pre_pose.pose.position.z -= gripper_depth - 0.05;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);
}

void Grasp_Calculator::addGraspPositionsX(double h, double d, double rotation, std::string frame_id,
        std::vector<geometry_msgs::PoseStamped> &poses,
        std::vector<geometry_msgs::PoseStamped> &pre_poses,
        double gripper_depth)
{
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pre_pose;
    pose.header.frame_id = frame_id;
    pre_pose.header.frame_id = frame_id;

    //grasp from the front
    pose.pose.position.x = 0 - gripper_depth - d - 0.005;
    pose.pose.position.y = 0;
    pose.pose.position.z = h;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, 0);

    pre_pose = pose;
    pre_pose.pose.position.x -= gripper_depth;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);

    //grasp from behind
    pose.pose.position.x = gripper_depth + d + 0.005;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, M_PI);

    pre_pose = pose;
    pre_pose.pose.position.x += gripper_depth;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);
}

void Grasp_Calculator::addGraspPositionsY(double h, double d, double rotation, std::string frame_id,
        std::vector<geometry_msgs::PoseStamped> &poses,
        std::vector<geometry_msgs::PoseStamped> &pre_poses,
        double gripper_depth)
{
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pre_pose;
    pose.header.frame_id = frame_id;
    pre_pose.header.frame_id = frame_id;

    //grasp from the left
    pose.pose.position.x = 0;
    pose.pose.position.y = gripper_depth + d + 0.005;
    pose.pose.position.z = h;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, -M_PI_2);

    pre_pose = pose;
    pre_pose.pose.position.y += gripper_depth;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);

    //grasp from right
    pose.pose.position.y = 0 - gripper_depth - d - 0.005;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, M_PI_2);

    pre_pose = pose;
    pre_pose.pose.position.y -= gripper_depth;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);
}



bool sort_base_link_poses(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2, geometry_msgs::PointStamped p)
{
    return Grasp_Calculator::get_angle(p.point, pose1.pose.position) < Grasp_Calculator::get_angle(p.point, pose2.pose.position);
}

int Grasp_Calculator::calcBoxGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
        std::vector<geometry_msgs::PoseStamped> &pre_poses,
        double gripper_depth)
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
        addGraspPositionsZ(z / 2, M_PI_2, co.id, poses, pre_poses, gripper_depth);

        //~ addBoxGraspPositionsY(y, 0, co.id, poses, pre_poses);
        addGraspPositionsY(0, y / 2, 0, co.id, poses, pre_poses, gripper_depth);
    }
    if (y < Gripper::GRIPPER_MAX_POSITION)
    {
        //object can be grasped from above, below, front and behind
        addGraspPositionsZ(z / 2, 0, co.id, poses, pre_poses, gripper_depth);

        addGraspPositionsX(0, x / 2, 0, co.id, poses, pre_poses, gripper_depth);
    }
    if (z < Gripper::GRIPPER_MAX_POSITION)
    {
        //object can be grasped from the left, right, front and behind
        addGraspPositionsX(0, x / 2, M_PI_2, co.id, poses, pre_poses, gripper_depth);

        addGraspPositionsY(0, y / 2, M_PI_2, co.id, poses, pre_poses, gripper_depth);
    }

    geometry_msgs::PointStamped p = get_point_above_object(co.id);

    std::sort(poses.begin(), poses.end(), boost::bind(sort_base_link_poses, _1, _2, p));
    std::sort(pre_poses.begin(), pre_poses.end(), boost::bind(sort_base_link_poses, _1, _2, p));

    return 1;
}

geometry_msgs::PointStamped Grasp_Calculator::get_point_above_object(std::string object_id)
{
    geometry_msgs::PointStamped p;
    p.header.stamp = ros::Time();
    p.header.frame_id = object_id;
    try
    {
        listener_.transformPoint("base_link", p, p);
    }
    catch (...)
    {
        ROS_ERROR_STREAM("ERROR: Transformation failed.");
    }
    p.point.z += 1;
    try
    {
        listener_.transformPoint(object_id, p, p);
    }
    catch (...)
    {
        ROS_ERROR_STREAM("ERROR: Transformation failed.");
    }
    return p;
}

int Grasp_Calculator::calcCylinderGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
        std::vector<geometry_msgs::PoseStamped> &pre_poses,
        double gripper_depth)
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
        addGraspPositionsX((h / 2) - (cylinder_safty_dist * i), r, 0, co.id, poses, pre_poses, gripper_depth);
        addGraspPositionsY((h / 2) - (cylinder_safty_dist * i), r, 0, co.id, poses, pre_poses, gripper_depth);

    }

    return 1;
}

int Grasp_Calculator::calcGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
                                        std::vector<geometry_msgs::PoseStamped> &pre_poses,
        double gripper_depth)
{
    //choose the right grasppositoncalculationfunction
    switch (co.primitives[0].type)
    {
    case shape_msgs::SolidPrimitive::CYLINDER:
        return calcCylinderGraspPosition(co, poses, pre_poses, gripper_depth);
        break;

    case shape_msgs::SolidPrimitive::BOX:
        return calcBoxGraspPosition(co, poses, pre_poses, gripper_depth);
        break;

    default:
        ROS_ERROR_STREAM("Can't calculate grasppositon for objecttype: " << co.primitives[0].type);
        return 0;
        break;
    }

    ROS_INFO_STREAM("Something bad happened.");
    return 0;
}

void Grasp_Calculator::transform_poses(std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses)
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
