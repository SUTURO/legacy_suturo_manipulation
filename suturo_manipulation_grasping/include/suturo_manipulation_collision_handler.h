#ifndef SUTURO_MANIPULATION_COLLISION_HANDLER
#define SUTURO_MANIPULATION_COLLISION_HANDLER
#include <ros/ros.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <tf/transform_listener.h>

using namespace std;
class Collision_Handler
{
  protected:
    int attempt_, maxAttempts_;
    int *collisionValues_;
    Suturo_Manipulation_Planning_Scene_Interface* pi_;
    tf::TransformListener listener_;

  public:
    Collision_Handler(ros::NodeHandle* nh, int maxAttempts, Suturo_Manipulation_Planning_Scene_Interface* pi);
    void reset();
    void handleCollision(int collisionValue, moveit_msgs::CollisionObject& co);
    bool attemptValid();

};

#endif
