#include "ros/ros.h"
#include "../../../../devel/include/testnode_manipulation/t_move_srv.h"
#include <stdio.h>
#include <moveit/move_group_interface/move_group.h>

using namespace std;

bool moveb(testnode_manipulation::t_move_srv::Request  &req,
         testnode_manipulation::t_move_srv::Response &res)
{
	cout << req.x << "  " << req.y << " " << req.z << endl;
	
    ros::AsyncSpinner spinner(1);
	spinner.start();
	
	move_group_interface::MoveGroup group(req.arm);
	cout << group.getEndEffectorLink() << endl;
	if (req.extra == "home"){
		group.setNamedTarget(req.arm);
	} else {
		vector<double> a;
		a.push_back(0.1);
		a.push_back(0.1);
		a.push_back(0.1);
		a.push_back(0.1);
		a.push_back(0.1);
		a.push_back(0.1);
		a.push_back(0.1);
		group.setJointValueTarget(a);
		/*cout << group.getGoalPositionTolerance() << endl;
		cout << group.getGoalOrientationTolerance() << endl;
		group.setPositionTarget((double)req.x, (double)req.y, (double)req.z);
		group.setOrientationTarget(group.getCurrentPose().pose.orientation.x,
						group.getCurrentPose().pose.orientation.y,
						group.getCurrentPose().pose.orientation.z,
						group.getCurrentPose().pose.orientation.w);*/
		//group.setPoseTarget(group.getCurrentPose());
		
	}

	
	res.succ = group.move();
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t_move", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("t_move_srv", moveb);
  ROS_INFO("Ready to moveit!.");
  ros::spin();

  return 0;
}
