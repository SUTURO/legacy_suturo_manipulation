/**
* This Server receives a target where the robot should move.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_baseAction.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_move_robot.h>


using namespace std;

Suturo_Manipulation_Move_Robot *moveRobot_;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_baseAction> Server;

/**
* Sends the goal to Suturo_Manipulation_Move_Robot and returns Success / Fail.
*/
void moveBase(const suturo_manipulation_msgs::suturo_manipulation_baseGoalConstPtr &baseGoal, ros::NodeHandle *nh, Server *server_base)
{

    // create a move result message
    suturo_manipulation_msgs::suturo_manipulation_baseResult r;
    // Set header
    r.succ.header.stamp = ros::Time();
    // Set Answer fot planning to undefined
    r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;
    //~ ROS_INFO_STREAM();
    if (moveRobot_->driveBase(baseGoal->ps, baseGoal->range))
    {
        // Set Answer fot planning to success
        r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
        server_base->setSucceeded(r);
    }
    else
    {
        // Set Answer fot planning to fail
        r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
        server_base->setAborted(r);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "suturo_manipulation_move_base_server");
    ros::NodeHandle nh;

    moveRobot_ = new Suturo_Manipulation_Move_Robot(&nh);

    Server server_base(nh, "suturo_man_move_base_server", boost::bind(&moveBase, _1, &nh, &server_base), false);
    server_base.start();


    ROS_INFO("Ready to move the base!");

    ros::spin();
    return 0;
}
