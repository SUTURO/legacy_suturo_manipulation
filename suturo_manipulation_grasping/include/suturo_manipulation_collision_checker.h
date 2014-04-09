#ifndef SUTURO_MANIPULATION_COLLISION_CHECKER
#define SUTURO_MANIPULATION_COLLISION_CHECKER
#include <ros/ros.h>
#include <pr2_msgs/PressureState.h>

using namespace std;

class Collision_Checker
{

  public:
    Collision_Checker(ros::NodeHandle nh);
    void clear();
    int r_collision();
    int l_collision();
};

#endif
