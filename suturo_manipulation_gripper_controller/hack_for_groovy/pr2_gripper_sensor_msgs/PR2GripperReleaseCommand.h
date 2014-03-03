/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-pr2-object-manipulation-0.7.5/debian/ros-groovy-pr2-object-manipulation/opt/ros/groovy/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_msgs/msg/PR2GripperReleaseCommand.msg */
#ifndef PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERRELEASECOMMAND_H
#define PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERRELEASECOMMAND_H
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

#include "pr2_gripper_sensor_msgs/PR2GripperEventDetectorCommand.h"

namespace pr2_gripper_sensor_msgs
{
template <class ContainerAllocator>
struct PR2GripperReleaseCommand_ {
  typedef PR2GripperReleaseCommand_<ContainerAllocator> Type;

  PR2GripperReleaseCommand_()
  : event()
  {
  }

  PR2GripperReleaseCommand_(const ContainerAllocator& _alloc)
  : event(_alloc)
  {
  }

  typedef  ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand_<ContainerAllocator>  _event_type;
   ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand_<ContainerAllocator>  event;


  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PR2GripperReleaseCommand
typedef  ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<std::allocator<void> > PR2GripperReleaseCommand;

typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand> PR2GripperReleaseCommandPtr;
typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand const> PR2GripperReleaseCommandConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr2_gripper_sensor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e62b08129864bf301ed0a1335e6158dc";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe62b08129864bf30ULL;
  static const uint64_t static_value2 = 0x1ed0a1335e6158dcULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr2_gripper_sensor_msgs/PR2GripperReleaseCommand";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# the event conditions we would like to trigger the robot to release on\n\
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

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.event);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PR2GripperReleaseCommand_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseCommand_<ContainerAllocator> & v) 
  {
    s << indent << "event: ";
s << std::endl;
    Printer< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand_<ContainerAllocator> >::stream(s, indent + "  ", v.event);
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERRELEASECOMMAND_H
