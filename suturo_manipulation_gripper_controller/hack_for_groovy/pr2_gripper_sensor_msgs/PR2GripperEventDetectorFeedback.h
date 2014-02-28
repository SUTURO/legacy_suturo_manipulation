/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-pr2-object-manipulation-0.7.5/debian/ros-groovy-pr2-object-manipulation/opt/ros/groovy/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_msgs/msg/PR2GripperEventDetectorFeedback.msg */
#ifndef PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPEREVENTDETECTORFEEDBACK_H
#define PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPEREVENTDETECTORFEEDBACK_H
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

#include "pr2_gripper_sensor_msgs/PR2GripperEventDetectorData.h"

namespace pr2_gripper_sensor_msgs
{
template <class ContainerAllocator>
struct PR2GripperEventDetectorFeedback_ {
  typedef PR2GripperEventDetectorFeedback_<ContainerAllocator> Type;

  PR2GripperEventDetectorFeedback_()
  : data()
  {
  }

  PR2GripperEventDetectorFeedback_(const ContainerAllocator& _alloc)
  : data(_alloc)
  {
  }

  typedef  ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator>  _data_type;
   ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator>  data;


  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PR2GripperEventDetectorFeedback
typedef  ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<std::allocator<void> > PR2GripperEventDetectorFeedback;

typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback> PR2GripperEventDetectorFeedbackPtr;
typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback const> PR2GripperEventDetectorFeedbackConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr2_gripper_sensor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "817b45a51c75a067eb5dfb8e18b14aa1";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x817b45a51c75a067ULL;
  static const uint64_t static_value2 = 0xeb5dfb8e18b14aa1ULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr2_gripper_sensor_msgs/PR2GripperEventDetectorFeedback";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# feedback\n\
PR2GripperEventDetectorData data\n\
\n\
\n\
\n\
================================================================================\n\
MSG: pr2_gripper_sensor_msgs/PR2GripperEventDetectorData\n\
# Time the data was recorded at\n\
time stamp\n\
\n\
# true if the trigger conditions have been met \n\
# (see PR2GripperEventDetectorCommand)\n\
bool trigger_conditions_met\n\
\n\
# true if the pressure sensors detected a slip event\n\
# slip events occur when the finger pressure sensors\n\
# high-freq. content exceeds the slip_trigger_magnitude variable\n\
# (see PR2GripperEventDetectorCommand)\n\
bool slip_event\n\
\n\
# true if the hand-mounted accelerometer detected a contact acceleration\n\
# acceleration events occur when the palm accelerometer\n\
# high-freq. content exceeds the acc_trigger_magnitude variable\n\
# (see PR2GripperEventDetectorCommand)\n\
bool acceleration_event\n\
\n\
# the high-freq acceleration vector that was last seen (x,y,z)\n\
float64[3] acceleration_vector\n\
";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PR2GripperEventDetectorFeedback_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback_<ContainerAllocator> & v) 
  {
    s << indent << "data: ";
s << std::endl;
    Printer< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> >::stream(s, indent + "  ", v.data);
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPEREVENTDETECTORFEEDBACK_H

