/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-pr2-object-manipulation-0.7.5/debian/ros-groovy-pr2-object-manipulation/opt/ros/groovy/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_msgs/msg/PR2GripperReleaseGoal.msg */
#ifndef PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERRELEASEGOAL_H
#define PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERRELEASEGOAL_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "pr2_gripper_sensor_msgs/PR2GripperReleaseCommand.h"

namespace pr2_gripper_sensor_msgs
{
template <class ContainerAllocator>
struct PR2GripperReleaseGoal_ {
  typedef PR2GripperReleaseGoal_<ContainerAllocator> Type;

  PR2GripperReleaseGoal_()
  : command()
  {
  }

  PR2GripperReleaseGoal_(const ContainerAllocator& _alloc)
  : command(_alloc)
  {
  }

  typedef  ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator>  _command_type;
   ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator>  command;


  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PR2GripperReleaseGoal
typedef  ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<std::allocator<void> > PR2GripperReleaseGoal;

typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal> PR2GripperReleaseGoalPtr;
typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal const> PR2GripperReleaseGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr2_gripper_sensor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f92a4c7c03d33b62ef7f6041bec6a43d";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf92a4c7c03d33b62ULL;
  static const uint64_t static_value2 = 0xef7f6041bec6a43dULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr2_gripper_sensor_msgs/PR2GripperReleaseGoal";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#goal\n\
PR2GripperReleaseCommand command\n\
\n\
================================================================================\n\
MSG: pr2_gripper_sensor_msgs/PR2GripperReleaseCommand\n\
# the event conditions we would like to trigger the robot to release on\n\
PR2GripperEventDetectorCommand event\n\
================================================================================\n\
MSG: pr2_gripper_sensor_msgs/PR2GripperEventDetectorCommand\n\
# state variable that defines what events we would like to trigger on\n\
# Leaving this field blank will result in the robot triggering when \n\
# anything touches the sides of the finger or an impact is detected\n\
# with the hand/arm.\n\
int8 trigger_conditions\n\
# definitions for our various trigger_conditions values\n\
# trigger on either acceleration contact or finger sensor side impact\n\
int8 FINGER_SIDE_IMPACT_OR_ACC = 0\n\
# tigger once  both slip and acceleration signals occur\n\
int8 SLIP_AND_ACC = 1 \n\
#  trigger on either slip, acceleration, or finger sensor side impact\n\
int8 FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC = 2\n\
# trigger only on slip information\n\
int8 SLIP = 3\n\
# trigger only on acceleration contact information\n\
int8 ACC = 4 \n\
\n\
\n\
# the amount of acceleration to trigger on (acceleration vector magnitude)\n\
# Units = m/s^2\n\
# The user needs to be concerned here about not setting the trigger too\n\
# low so that is set off by the robot's own motions.\n\
#\n\
# For large rapid motions, say by a motion planner, 5 m/s^2 is a good level\n\
# For small delicate controlled motions this can be set MUCH lower (try 2.0)\n\
#\n\
# NOTE: When moving the gripper joint (opening/closing the grippr)\n\
# the high gearing of the PR2 gripper causes large acceleration vibrations\n\
# which will cause triggering to occur. This is a known drawback of the PR2.\n\
#\n\
# NOTE: Leaving this value blank will result in a 0 m/s^2 trigger. If you\n\
# are using a trigger_conditions value that returns on acceleration contact\n\
# events then it will immediately exceed your trigger and return\n\
float64 acceleration_trigger_magnitude\n\
\n\
\n\
# the slip detector gain to trigger on (either finger) : try 0.01\n\
# higher values decrease slip sensitivty (to a point)\n\
# lower values increase sensitivity (to a point)\n\
#\n\
# NOTE: Leaving this value blank will result in the most sensitive slip level.\n\
float64 slip_trigger_magnitude\n\
";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.command);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PR2GripperReleaseGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseGoal_<ContainerAllocator> & v) 
  {
    s << indent << "command: ";
s << std::endl;
    Printer< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> >::stream(s, indent + "  ", v.command);
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERRELEASEGOAL_H

