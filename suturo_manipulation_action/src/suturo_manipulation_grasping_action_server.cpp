/**
* This class implements the action server to grasp/drop a object.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_graspingAction.h>
#include <suturo_manipulation_msgs/GraspingAndDrop.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <suturo_manipulation_grasping.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>
#include <control_msgs/PointHeadActionGoal.h>
#include <pr2_controllers_msgs/PointHeadActionResult.h>


using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_graspingAction> Server;

/**
* This method grasps or drops a object!
*/
void grop(const suturo_manipulation_msgs::suturo_manipulation_graspingGoalConstPtr& graspGoal, ros::Publisher* publisher, ros::NodeHandle* nh, Server* server_grasp)
{	
  suturo_manipulation_msgs::suturo_manipulation_graspingResult r;	

  // Set header
  r.succ.header.stamp = ros::Time();
  // Set Answer for planning to undefined
  r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;

  // set Planning Interface and Grasper
  ROS_INFO("Create Planning Scene Interface...");
  Suturo_Manipulation_Planning_Scene_Interface pi(nh);
  ROS_INFO("Done. Create Grasper...")	;
  Grasping grasper(&pi);
  ROS_INFO("Done.");

  string picking_arm = graspGoal->goal.bodypart.bodyPart;
  string action = graspGoal->goal.action.action;
  if (picking_arm != suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM && 
  		picking_arm != suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM){  
    ROS_INFO("Unknown arm! Please use suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM or suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM as names!\n");
    r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
    server_grasp->setAborted(r); 
  } else {
    ROS_INFO("Arm to pick: %s", picking_arm.c_str());
    string obj_name = graspGoal->goal.objectName;
    ROS_INFO("ObjectName to pick: %s", obj_name.c_str());

    double newton = graspGoal->goal.newton;
    ROS_INFO("Newton: %f", newton);

    // Check if we should grasp or drop...
    if(action == suturo_manipulation_msgs::GraspingAndDrop::GRASP){
      ROS_INFO("Begin to pick object...");
      // Grasp the object
      if (grasper.pick(obj_name, picking_arm, newton, publisher))
      {
        ROS_INFO("Object picked...\n");
        r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
        server_grasp->setSucceeded(r);
      } else {
        ROS_INFO("Picking failed!\n");
        r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
        server_grasp->setAborted(r);
      }
    } else if (action == suturo_manipulation_msgs::GraspingAndDrop::DROP_OBJECT) {
      ROS_INFO("Begin to drop object...");
      // Drop the object
      if (grasper.dropObject(obj_name))
      {
        ROS_INFO("Object droped...\n");
        r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
        server_grasp->setSucceeded(r);
      } else {
        ROS_INFO("Droping failed!\n");
        r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
        server_grasp->setAborted(r);
      }
    } else if (action == suturo_manipulation_msgs::GraspingAndDrop::OPEN_GRIPPER){
      if (grasper.drop(picking_arm))
      {
        ROS_INFO("Opened gripper...\n");
        r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
        server_grasp->setSucceeded(r);
      } else {
        ROS_INFO("Open gripper failed!\n");
        r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
        server_grasp->setAborted(r);
      }
    } else {
	ROS_INFO("Unknown message!");
    }
  }
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_grasp_server");
	ros::NodeHandle nh;
	
  // Publish a topic for the ros intern head controller
  ros::Publisher head_publisher = nh.advertise<control_msgs::PointHeadActionGoal>("/head_traj_controller/point_head_action/goal", 1000);

	Server server_grasp(nh, "suturo_man_grasping_server", boost::bind(&grop, _1, &head_publisher, &nh, &server_grasp), false);
	server_grasp.start();
	

	ROS_INFO("Ready to grasp!!!");

	ros::spin();
	return 0;
}
