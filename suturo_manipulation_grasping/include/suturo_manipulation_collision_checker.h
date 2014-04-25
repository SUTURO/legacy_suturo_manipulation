#ifndef SUTURO_MANIPULATION_COLLISION_CHECKER
#define SUTURO_MANIPULATION_COLLISION_CHECKER
#include <ros/ros.h>
#include <pr2_msgs/PressureState.h>

using namespace std;

class Collision_Checker
{
  protected:
    int r_arm_r_finger_[22];
    int r_arm_l_finger_[22];
    int l_arm_r_finger_[22];
    int l_arm_l_finger_[22];
    int r_arm_r_finger_tara_[22];
    int r_arm_l_finger_tara_[22];
    int l_arm_r_finger_tara_[22];
    int l_arm_l_finger_tara_[22];
    ros::Subscriber l_sub_, r_sub_;
  public:
    Collision_Checker(ros::NodeHandle* nh);
    void clear();
    int r_collision();
    int l_collision();
    bool updated_;

  private:
    void l_gripperCallback(const pr2_msgs::PressureState& msg);
    void r_gripperCallback(const pr2_msgs::PressureState& msg);

  };

#endif
