#include "suturo_manipulation_grasping_reactive.h"

using namespace std;

Grasping_reactive::Grasping_reactive(Suturo_Manipulation_Planning_Scene_Interface* pi, ros::Publisher* head_publisher) : Grasping(pi, head_publisher) {

}

int Grasping_reactive::move(move_group_interface::MoveGroup* move_group, geometry_msgs::PoseStamped desired_pose)
{
	this->Grasping::move(move_group, desired_pose);
}