#include "suturo_manipulation_planning_scene_interface.h"

using namespace std;

Suturo_Manipulation_Planning_Scene_Interface::Suturo_Manipulation_Planning_Scene_Interface(ros::NodeHandle *nodehandle)
{
    nh_ = nodehandle;

    //publisher
    attached_object_publisher_ = nh_->advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
    collision_object_publisher_ = nh_->advertise<moveit_msgs::CollisionObject>("collision_object", 10);
    vis_pub_ = nh_->advertise<visualization_msgs::Marker>( "visualization_marker", 10 );
    planning_scene_publisher = nh_->advertise<moveit_msgs::PlanningScene>("planning_scene", 10);
    test_coll_ps_pub_ = nh_->advertise<moveit_msgs::PlanningScene>("/suturo/collision_ps", 10);


    //service clients
    int i = 0;
    do
    {
        ps_service_client_ = nh_->serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
        i++;
        if (i == 20)
        {
            ROS_ERROR_STREAM("Can not connect to Planningscene Service");

        }
        //wait because ros
        ros::WallDuration(0.5).sleep();
    }
    while (!ps_service_client_.exists() );

    ros::WallDuration(0.5).sleep();

}

Suturo_Manipulation_Planning_Scene_Interface::~Suturo_Manipulation_Planning_Scene_Interface()
{

}

void Suturo_Manipulation_Planning_Scene_Interface::publishTfFrame(std::string frame_id, geometry_msgs::PoseStamped pose)
{

    ROS_DEBUG_STREAM("Publish TF frame " << frame_id);

    transform.setOrigin( tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) );

    transform.setRotation( tf::Quaternion(pose.pose.orientation.x,
                                          pose.pose.orientation.y,
                                          pose.pose.orientation.z,
                                          pose.pose.orientation.w) );

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), pose.header.frame_id, frame_id));
    ros::WallDuration(0.2).sleep();
}

int Suturo_Manipulation_Planning_Scene_Interface::allowCollision(std::string object1, std::string object2)
{
    moveit_msgs::PlanningScene ps;
    if (!getPlanningScene(ps)) return 0;

    collision_detection::AllowedCollisionMatrix acm(ps.allowed_collision_matrix);

    acm.setEntry(object1, object2, true);

    acm.getMessage(ps.allowed_collision_matrix);

    planning_scene_publisher.publish(ps);
    return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::denyCollision(std::string object1, std::string object2)
{
    moveit_msgs::PlanningScene ps;
    if (!getPlanningScene(ps)) return 0;

    collision_detection::AllowedCollisionMatrix acm(ps.allowed_collision_matrix);

    acm.setEntry(object1, object2, false);

    acm.getMessage(ps.allowed_collision_matrix);

    planning_scene_publisher.publish(ps);
    return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::getPlanningScene(moveit_msgs::PlanningScene &ps)
{
    //create msg to get Objectnames and Objectgeometry from planningscene
    moveit_msgs::GetPlanningScene msg;
    msg.request.components.components = 1023;

    //get planningscene

    ps_service_client_.call(msg);
    if (ps_service_client_.call(msg))
    {
        ps = msg.response.scene;
    }
    else
    {
        ROS_ERROR("Failed to call service to get planningscene.");
        return 0;
    }
    return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::publishPlanningScene(moveit_msgs::PlanningScene ps)
{
    planning_scene_publisher.publish(ps);
    return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::attachObject(std::string object_name, std::string link_name,
        std::vector<std::string> gripper_links)
{
    //check if the link name is valid
    if (link_name.empty())
    {
        ROS_ERROR("No link specified to attach the object '%s' to",
                  object_name.c_str());
        return false;
    }

    //check if another object is attached to this link
    std::vector<moveit_msgs::AttachedCollisionObject> acos;
    if (!getAttachedObjects(acos)) return 0;

    for (std::vector<moveit_msgs::AttachedCollisionObject>::iterator it = acos.begin(); it != acos.end(); ++it)
    {
        if (it->object.id == object_name)
        {
            ROS_WARN_STREAM(object_name << " already attached to " << it->link_name << ".");
            if (it->link_name == link_name)
            {
                ROS_WARN_STREAM(object_name << " already attached to this link.");
                return 1;
            }
            else
            {
                ROS_INFO_STREAM("Detaching object from old link.");
                detachObject(object_name);
            }
        }
    }

    //get object from the planningscene
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = link_name;
    getObject(object_name, attached_object.object);
    attached_object.object.operation = attached_object.object.ADD;

    //specify the links that are allowed to touch the object
    attached_object.touch_links = gripper_links;

    //tell the planningscene that the object is attached
    attached_object_publisher_.publish(attached_object);

    // for (int i = 0; i < gripper_links.size(); i++)
    // {
    //  pi_->allowCollision(gripper_links.at(i), object_name);
    // }

    //wait because ros
    ros::WallDuration(1.0).sleep();
    ROS_DEBUG_STREAM("Attached " << object_name << " to " << link_name << ".");
    return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::getObject(std::string object_name, moveit_msgs::CollisionObject &co)
{

    //msg to get Objectnames and Objectgeometry from planningscene
    moveit_msgs::GetPlanningScene msg;
    msg.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY +
                                        moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;

    //get planningscene
    ros::ServiceClient client = nh_->serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    client.call(msg);
    if (client.call(msg))
    {
        //search for objectname
        for (int i = 0; i < msg.response.scene.world.collision_objects.size(); i++)
        {
            moveit_msgs::CollisionObject tempCO = msg.response.scene.world.collision_objects[i];
            if (tempCO.id == object_name)
            {
                co = tempCO;
                break;
            }
        }
        //Object not found
        if (co.id != object_name)
        {
            ROS_WARN_STREAM(" Object: " << object_name << " not found!!");
            return 0;
        }
    }
    else
    {
        ROS_ERROR("Suturo_Manipulation_Planning_Scene_Interface::getObject| Failed to call service move_group::GET_PLANNING_SCENE_SERVICE_NAME");
        return 0;
    }
    ROS_DEBUG_STREAM("Object " << object_name << " found.");
    return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::getAttachedObjects(std::vector<moveit_msgs::AttachedCollisionObject> &acos)
{
    moveit_msgs::PlanningScene ps;
    if (!getPlanningScene(ps))
    {
        ROS_ERROR_STREAM("Failed to get planningscene");
        return 0;
    }
    acos = ps.robot_state.attached_collision_objects;
    return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::getObjects(std::vector<moveit_msgs::CollisionObject> &cos)
{
    moveit_msgs::PlanningScene ps;
    if (!getPlanningScene(ps))
    {
        ROS_ERROR_STREAM("Failed to get planningscene");
        return 0;
    }
    cos = ps.world.collision_objects;
    return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::getAttachedObject(std::string object_name, moveit_msgs::AttachedCollisionObject &co)
{
    std::vector<moveit_msgs::AttachedCollisionObject> attachedObjects;
    if (!getAttachedObjects(attachedObjects)) return 0;
    //search for object_name in the list of attached objects
    for (int i = 0; i < attachedObjects.size(); i++)
    {
        co = attachedObjects.at(i);
        if (co.object.id == object_name)
        {
            return 1;
        }
    }
    return 0;
}


int Suturo_Manipulation_Planning_Scene_Interface::addObject(moveit_msgs::CollisionObject co)
{
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    collision_object_publisher_.publish(co);

    co.operation = moveit_msgs::CollisionObject::ADD;
    collision_object_publisher_.publish(co);
    ROS_INFO_STREAM("Added object " << co.id << " to planningscene.");
    return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::removeObject(moveit_msgs::CollisionObject co)
{
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    collision_object_publisher_.publish(co);
    ROS_INFO_STREAM("Removed " << co.id << " form planningscene.");
    return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::detachObject(std::string object_name)
{

    moveit_msgs::AttachedCollisionObject attached_object;
    moveit_msgs::AttachedCollisionObject detached_object;

    //get the attached object
    if (!getAttachedObject(object_name, attached_object))
    {
        ROS_INFO_STREAM(object_name << " wasn't attached.");
        return 1;
    }
    detached_object.object.operation = attached_object.object.REMOVE;
    detached_object.object.id = attached_object.object.id;
    detached_object.link_name = attached_object.link_name;
    //detach it
    attached_object_publisher_.publish(detached_object);

    //deny collision
    // for (int i = 0; i < gripper_links.size(); i++)
    // {
    //  pi_->denyCollision(gripper_links.at(i), object_name);
    // }

    //wait because ros
    ros::WallDuration(1.0).sleep();
    ROS_INFO_STREAM(object_name << " detached.");
    return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::isAnObjectAttachedToArm(std::string link_name, moveit_msgs::AttachedCollisionObject &aco)
{

    std::vector<moveit_msgs::AttachedCollisionObject> acos;
    if (!getAttachedObjects(acos)) return 0;

    for (std::vector<moveit_msgs::AttachedCollisionObject>::iterator it = acos.begin(); it != acos.end(); ++it)
    {
        if (it->link_name == link_name)
        {
            //if the link names are the same, the object is attached to it
            aco = *it;
            return 1;
        }
    }

    return 0;
}

void get_ps_with_open_gripper(moveit_msgs::PlanningScene &ps)
{
    double gripper_position = 0.51447;
    double gripper_position2 = 0.88;

    //search for gripper joints and open them
    for (int joint_name_id = 0; joint_name_id < ps.robot_state.joint_state.name.size(); ++joint_name_id)
    {
        if (ps.robot_state.joint_state.name[joint_name_id] == "l_gripper_l_finger_joint"
                || ps.robot_state.joint_state.name[joint_name_id] == "r_gripper_l_finger_joint"

                || ps.robot_state.joint_state.name[joint_name_id] == "l_gripper_l_finger_tip_joint"
                || ps.robot_state.joint_state.name[joint_name_id] == "r_gripper_l_finger_tip_joint"

                || ps.robot_state.joint_state.name[joint_name_id] == "l_gripper_r_finger_joint"
                || ps.robot_state.joint_state.name[joint_name_id] == "r_gripper_r_finger_joint"

                || ps.robot_state.joint_state.name[joint_name_id] == "l_gripper_r_finger_tip_joint"
                || ps.robot_state.joint_state.name[joint_name_id] == "r_gripper_r_finger_tip_joint")
        {
            ps.robot_state.joint_state.position[joint_name_id] = gripper_position;
        }
        if (ps.robot_state.joint_state.name[joint_name_id] == "l_gripper_joint"
                || ps.robot_state.joint_state.name[joint_name_id] == "r_gripper_joint")
        {
            ps.robot_state.joint_state.position[joint_name_id] = gripper_position2;
        }
    }

    //clear unnecessary objects
    ps.robot_state.attached_collision_objects.clear();
    ps.world.collision_objects.clear();
    ps.fixed_frame_transforms.clear();

}

bool Suturo_Manipulation_Planning_Scene_Interface::check_group_object_collision(string group_name, geometry_msgs::PoseStamped eef_pose,
        moveit_msgs::CollisionObject co)
{
    // geometry_msgs::PoseStamped frame_pose = eef_pose;
    // try
    // {
    //     listener_.transformPose("/base_link", frame_pose, frame_pose);
    // }
    // catch (tf::TransformException t)
    // {
    //     ROS_ERROR_STREAM(t.what());
    // }
    publishTfFrame("1337", eef_pose);
    moveit_msgs::PlanningScene ps;
    getPlanningScene(ps);
    get_ps_with_open_gripper(ps);
    //add object to planningscene
    geometry_msgs::PoseStamped temp_pose;
    temp_pose.header = co.header;
    temp_pose.pose = co.mesh_poses[0];
    temp_pose.header.stamp = ros::Time(0);

    for (int i = 0; i < 50; ++i)
    {
        try
        {
            publishTfFrame("1337", eef_pose);
            listener_.waitForTransform(co.id, "base_link", temp_pose.header.stamp, ros::Duration(5.0));
            listener_.transformPose("/1337", temp_pose, temp_pose);
            break;
        }
        catch (tf::TransformException t)
        {
            if (i == 50) ROS_ERROR_STREAM(t.what());
        }
        ROS_INFO_STREAM(i);
    }

    co.mesh_poses[0] = temp_pose.pose;
    co.header.stamp = temp_pose.header.stamp;
    if (group_name == "left_gipper")
    {
        co.header.frame_id = "l_wrist_roll_link";
    }
    else co.header.frame_id = "r_wrist_roll_link";
    ps.world.collision_objects.push_back(co);
    test_coll_ps_pub_.publish(ps);
    temp_pose.header.frame_id = "odom_combined";

    //check collision
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    planning_scene.setPlanningSceneMsg(ps);

    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = group_name;
    collision_detection::CollisionResult collision_result;

    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result);

    return collision_result.collision;
}

void Suturo_Manipulation_Planning_Scene_Interface::publishMarker(geometry_msgs::PoseStamped pose, int id)
{
    // Publish the Goalmarker
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = pose.header.frame_id;
    goal_marker.header.stamp = ros::Time();
    goal_marker.ns = "suturo_manipulation/eef_poses";
    goal_marker.id = id;
    goal_marker.type = visualization_msgs::Marker::ARROW;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose = pose.pose;


    goal_marker.color.a = 1.0;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;

    goal_marker.scale.z = 0.01;
    goal_marker.scale.y = 0.02;
    goal_marker.scale.x = 0.045;

    vis_pub_.publish( goal_marker );
}

void Suturo_Manipulation_Planning_Scene_Interface::publishMarkerPoints(std::string frame_id, std::vector<geometry_msgs::Point> points)
{
    //Publish the Goalmarker
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = frame_id;
    goal_marker.header.stamp = ros::Time();
    goal_marker.ns = "suturo_manipulation/points";
    goal_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.color.a = 1.0;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
    goal_marker.scale.x = 0.01;
    goal_marker.scale.y = 0.01;
    goal_marker.scale.z = 0.01;
    goal_marker.pose.orientation.w = 1;
    for (int i = 0; i < points.size(); ++i)
    {
        // ROS_INFO_STREAM(points[i]);

        // for (int p = 0; p < points.size(); ++p)
        // {
        goal_marker.points.push_back(points[i]);
        goal_marker.colors.push_back(goal_marker.color);
        // }
        // goal_marker.pose.position = points[i];


        if (i % 5 == 4 || i == points.size())
        {
            vis_pub_.publish( goal_marker);
            goal_marker.id++;
            goal_marker.points.clear();
            goal_marker.colors.clear();
        }
    }
    vis_pub_.publish( goal_marker);
    goal_marker.id++;
    goal_marker.points.clear();
    goal_marker.colors.clear();
}

void Suturo_Manipulation_Planning_Scene_Interface::publishMarkerLine(std::string frame_id, std::vector<geometry_msgs::Point> points, int id)
{
    //Publish the Goalmarker
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = frame_id;
    goal_marker.header.stamp = ros::Time();
    goal_marker.ns = "suturo_manipulation/line";
    goal_marker.type = visualization_msgs::Marker::LINE_STRIP;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.color.a = 1.0;
    goal_marker.color.r = 0.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 1.0;
    goal_marker.scale.x = 0.005;
    goal_marker.scale.y = 0.01;
    goal_marker.scale.z = 0.01;
    goal_marker.pose.orientation.w = 1;
    goal_marker.id = id;
    for (int i = 0; i < points.size(); ++i)
    {
        // ROS_INFO_STREAM(points[i]);

        // for (int p = 0; p < points.size(); ++p)
        // {
        goal_marker.points.push_back(points[i]);
        goal_marker.colors.push_back(goal_marker.color);
        // }
        // goal_marker.pose.position = points[i];


        if (i % 5 == 4 || i == points.size())
        {
            vis_pub_.publish( goal_marker);
            goal_marker.id++;
            goal_marker.points.clear();
            goal_marker.colors.clear();
            goal_marker.points.push_back(points[i]);
            goal_marker.colors.push_back(goal_marker.color);
        }
    }
    vis_pub_.publish( goal_marker);
    goal_marker.id++;
    goal_marker.points.clear();
    goal_marker.colors.clear();

    // ROS_INFO_STREAM(points[i]);

    // for (int p = 0; p < points.size(); ++p)
    // {

    //         goal_marker.points.push_back(points[p]);
    //         goal_marker.colors.push_back(goal_marker.color);

    // }
    // // goal_marker.pose.position = points[i];

    // goal_marker.id = id;


    // vis_pub_.publish( goal_marker);
}
















