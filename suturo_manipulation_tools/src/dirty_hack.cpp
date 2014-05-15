#include "ros/ros.h"
#include <suturo_manipulation_planning_scene_interface.h>
#include <tf/transform_broadcaster.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>


/**
 * This Programm publishes a tf frame into every collisionobject and attached object.
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_objects_tf_frames");
    ros::NodeHandle n;

    tf::Transform transform;
    tf::TransformBroadcaster br;

    Suturo_Manipulation_Planning_Scene_Interface *pi = new Suturo_Manipulation_Planning_Scene_Interface(&n);
    // std::vector<moveit_msgs::CollisionObject> cos;
    // std::vector<moveit_msgs::AttachedCollisionObject> acos;
    moveit_msgs::PlanningScene ps;
    // boost::this_thread::sleep(boost::posix_time::seconds(3));


    ROS_INFO_STREAM("move_group reanimate startet..........");

    while (true)
    {
        if (!(pi->getPlanningScene(ps)))
        {
            ROS_WARN_STREAM("reanimating move_group.....");
            pi = new Suturo_Manipulation_Planning_Scene_Interface(&n);
            pi->publishPlanningScene(ps);
            ROS_WARN_STREAM("done.");
            continue;
        }

        boost::this_thread::sleep(boost::posix_time::seconds(1));
        ros::spinOnce();
    }

    return 0;
}


















