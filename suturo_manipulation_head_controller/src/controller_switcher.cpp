#include "ros/ros.h"
#include <cstdlib>
#include <pr2_mechanism_msgs/SwitchController.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "controller_switcher");
    ros::NodeHandle n;
    ros::ServiceClient switch_client = n.serviceClient<pr2_mechanism_msgs::SwitchController>("/pr2_controller_manager/switch_controller");

    pr2_mechanism_msgs::SwitchController to_imped;
    to_imped.request.start_controllers.push_back("suturo_head_controller");
    to_imped.request.stop_controllers.push_back("head_traj_controller");
    to_imped.request.strictness = pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT;
    ROS_INFO("sending the call from inside of the method");
    if (switch_client.call(to_imped)) { 
        ROS_INFO("Switched from head_traj_controller to suturo_head_controller"); 
        return 1;
    } else { 
        ROS_INFO("Failed to switch from head_traj_controller to suturo_head_controller"); 
        return 0;
    } 

    return 0;
}
