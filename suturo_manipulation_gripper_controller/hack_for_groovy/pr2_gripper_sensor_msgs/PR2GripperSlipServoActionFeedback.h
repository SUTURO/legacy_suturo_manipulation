/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-pr2-object-manipulation-0.7.5/debian/ros-groovy-pr2-object-manipulation/opt/ros/groovy/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_msgs/msg/PR2GripperSlipServoActionFeedback.msg */
#ifndef PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERSLIPSERVOACTIONFEEDBACK_H
#define PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERSLIPSERVOACTIONFEEDBACK_H
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

#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "pr2_gripper_sensor_msgs/PR2GripperSlipServoFeedback.h"

namespace pr2_gripper_sensor_msgs
{
template <class ContainerAllocator>
struct PR2GripperSlipServoActionFeedback_ {
  typedef PR2GripperSlipServoActionFeedback_<ContainerAllocator> Type;

  PR2GripperSlipServoActionFeedback_()
  : header()
  , status()
  , feedback()
  {
  }

  PR2GripperSlipServoActionFeedback_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , status(_alloc)
  , feedback(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::actionlib_msgs::GoalStatus_<ContainerAllocator>  _status_type;
   ::actionlib_msgs::GoalStatus_<ContainerAllocator>  status;

  typedef  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoFeedback_<ContainerAllocator>  _feedback_type;
   ::pr2_gripper_sensor_msgs::PR2GripperSlipServoFeedback_<ContainerAllocator>  feedback;


  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PR2GripperSlipServoActionFeedback
typedef  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<std::allocator<void> > PR2GripperSlipServoActionFeedback;

typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback> PR2GripperSlipServoActionFeedbackPtr;
typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback const> PR2GripperSlipServoActionFeedbackConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr2_gripper_sensor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8819de47d9e7dfd2acefc052395548ad";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8819de47d9e7dfd2ULL;
  static const uint64_t static_value2 = 0xacefc052395548adULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr2_gripper_sensor_msgs/PR2GripperSlipServoActionFeedback";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
PR2GripperSlipServoFeedback feedback\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: pr2_gripper_sensor_msgs/PR2GripperSlipServoFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
#feedback\n\
PR2GripperSlipServoData data\n\
\n\
================================================================================\n\
MSG: pr2_gripper_sensor_msgs/PR2GripperSlipServoData\n\
# time the data was recorded at\n\
time stamp\n\
\n\
# the amount of deformation from action start (in meters)\n\
float64 deformation\n\
\n\
# the force experinced by the finger Pads  (N)\n\
# NOTE:this ignores data from the edges of the finger pressure\n\
float64 left_fingertip_pad_force\n\
float64 right_fingertip_pad_force\n\
\n\
# the current virtual parallel joint effort of the gripper (in N)\n\
float64 joint_effort\n\
\n\
# true if the object recently slipped\n\
bool slip_detected\n\
\n\
# true if we are at or exceeding the deformation limit\n\
# (see wiki page and param server for more info)\n\
bool deformation_limit_reached\n\
\n\
# true if we are at or exceeding our force \n\
# (see wiki page and param server for more info)\n\
bool fingertip_force_limit_reached\n\
\n\
# true if the controller thinks the gripper is empty\n\
# (see wiki page for more info)\n\
bool gripper_empty\n\
\n\
# the control state of our realtime controller\n\
PR2GripperSensorRTState rtstate\n\
================================================================================\n\
MSG: pr2_gripper_sensor_msgs/PR2GripperSensorRTState\n\
# the control state of our realtime controller\n\
int8 realtime_controller_state\n\
\n\
# predefined values to indicate our realtime_controller_state\n\
int8 DISABLED = 0\n\
int8 POSITION_SERVO = 3\n\
int8 FORCE_SERVO = 4\n\
int8 FIND_CONTACT = 5\n\
int8 SLIP_SERVO = 6\n\
";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.status);
    stream.next(m.feedback);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PR2GripperSlipServoActionFeedback_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "status: ";
s << std::endl;
    Printer< ::actionlib_msgs::GoalStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
    s << indent << "feedback: ";
s << std::endl;
    Printer< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.feedback);
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERSLIPSERVOACTIONFEEDBACK_H

