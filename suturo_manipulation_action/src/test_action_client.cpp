#include <suturo_manipulation_msgs/suturo_manipulation_moveAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<suturo_manipulation_msgs::suturo_manipulation_moveAction> Client;

int main(int argc, char** argv)
{
  if (argc != 5)
  {
    ROS_INFO("usage: t_move X Y Z");
    return 1;
  }
  ros::init(argc, argv, "test_action_client");
  Client client("move_action_server", true); // true -> don't need ros::spin()
  client.waitForServer();
  suturo_manipulation_msgs::suturo_manipulation_moveGoal goal;

  goal.arm = argv[1];
  goal.p.c_centroid.x = atof(argv[2]);
  goal.p.c_centroid.y = atof(argv[3]);
  goal.p.c_centroid.z = atof(argv[4]);
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(15.0));

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}
