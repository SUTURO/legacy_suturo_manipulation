#ifndef SUTURO_MANIPULATION_GRASPING_REACTIVE
#define SUTURO_MANIPULATION_GRASPING_REACTIVE

// #include <ros/ros.h>

#include <suturo_manipulation_grasping.h>

// #include <moveit/move_group_interface/move_group.h>

// #include <suturo_manipulation_gripper_controller.h>
// #include <suturo_manipulation_planning_scene_interface.h>
// #include <suturo_manipulation_msgs/RobotBodyPart.h>
// #include <pr2_controllers_msgs/PointHeadActionResult.h>
// #include <control_msgs/PointHeadActionGoal.h>

// #include <visualization_msgs/Marker.h>
#include <suturo_manipulation_collision_checker.h>

using namespace std;

class Grasping_reactive : public Grasping
{

protected:
  int move(move_group_interface::MoveGroup* move_group, geometry_msgs::PoseStamped desired_pose);
  void threaded_move(move_group_interface::MoveGroup* move_group);
  Collision_Checker* cc_;
  bool moveSucces_;

public:
	Grasping_reactive(ros::NodeHandle* nh, Suturo_Manipulation_Planning_Scene_Interface* pi, ros::Publisher* head_publisher=NULL);
	
};
     
#endif
