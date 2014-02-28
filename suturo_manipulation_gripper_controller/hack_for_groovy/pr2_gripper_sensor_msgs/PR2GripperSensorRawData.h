/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-pr2-object-manipulation-0.7.5/debian/ros-groovy-pr2-object-manipulation/opt/ros/groovy/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_msgs/msg/PR2GripperSensorRawData.msg */
#ifndef PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERSENSORRAWDATA_H
#define PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERSENSORRAWDATA_H
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


namespace pr2_gripper_sensor_msgs
{
template <class ContainerAllocator>
struct PR2GripperSensorRawData_ {
  typedef PR2GripperSensorRawData_<ContainerAllocator> Type;

  PR2GripperSensorRawData_()
  : stamp()
  , left_finger_pad_force(0.0)
  , right_finger_pad_force(0.0)
  , left_finger_pad_force_filtered(0.0)
  , right_finger_pad_force_filtered(0.0)
  , left_finger_pad_forces()
  , right_finger_pad_forces()
  , left_finger_pad_forces_filtered()
  , right_finger_pad_forces_filtered()
  , acc_x_raw(0.0)
  , acc_y_raw(0.0)
  , acc_z_raw(0.0)
  , acc_x_filtered(0.0)
  , acc_y_filtered(0.0)
  , acc_z_filtered(0.0)
  , left_contact(false)
  , right_contact(false)
  {
    left_finger_pad_forces.assign(0.0);
    right_finger_pad_forces.assign(0.0);
    left_finger_pad_forces_filtered.assign(0.0);
    right_finger_pad_forces_filtered.assign(0.0);
  }

  PR2GripperSensorRawData_(const ContainerAllocator& _alloc)
  : stamp()
  , left_finger_pad_force(0.0)
  , right_finger_pad_force(0.0)
  , left_finger_pad_force_filtered(0.0)
  , right_finger_pad_force_filtered(0.0)
  , left_finger_pad_forces()
  , right_finger_pad_forces()
  , left_finger_pad_forces_filtered()
  , right_finger_pad_forces_filtered()
  , acc_x_raw(0.0)
  , acc_y_raw(0.0)
  , acc_z_raw(0.0)
  , acc_x_filtered(0.0)
  , acc_y_filtered(0.0)
  , acc_z_filtered(0.0)
  , left_contact(false)
  , right_contact(false)
  {
    left_finger_pad_forces.assign(0.0);
    right_finger_pad_forces.assign(0.0);
    left_finger_pad_forces_filtered.assign(0.0);
    right_finger_pad_forces_filtered.assign(0.0);
  }

  typedef ros::Time _stamp_type;
  ros::Time stamp;

  typedef double _left_finger_pad_force_type;
  double left_finger_pad_force;

  typedef double _right_finger_pad_force_type;
  double right_finger_pad_force;

  typedef double _left_finger_pad_force_filtered_type;
  double left_finger_pad_force_filtered;

  typedef double _right_finger_pad_force_filtered_type;
  double right_finger_pad_force_filtered;

  typedef boost::array<double, 22>  _left_finger_pad_forces_type;
  boost::array<double, 22>  left_finger_pad_forces;

  typedef boost::array<double, 22>  _right_finger_pad_forces_type;
  boost::array<double, 22>  right_finger_pad_forces;

  typedef boost::array<double, 22>  _left_finger_pad_forces_filtered_type;
  boost::array<double, 22>  left_finger_pad_forces_filtered;

  typedef boost::array<double, 22>  _right_finger_pad_forces_filtered_type;
  boost::array<double, 22>  right_finger_pad_forces_filtered;

  typedef double _acc_x_raw_type;
  double acc_x_raw;

  typedef double _acc_y_raw_type;
  double acc_y_raw;

  typedef double _acc_z_raw_type;
  double acc_z_raw;

  typedef double _acc_x_filtered_type;
  double acc_x_filtered;

  typedef double _acc_y_filtered_type;
  double acc_y_filtered;

  typedef double _acc_z_filtered_type;
  double acc_z_filtered;

  typedef uint8_t _left_contact_type;
  uint8_t left_contact;

  typedef uint8_t _right_contact_type;
  uint8_t right_contact;


  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PR2GripperSensorRawData
typedef  ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<std::allocator<void> > PR2GripperSensorRawData;

typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData> PR2GripperSensorRawDataPtr;
typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData const> PR2GripperSensorRawDataConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr2_gripper_sensor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "696a1f2e6969deb0bc6998636ae1b17e";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x696a1f2e6969deb0ULL;
  static const uint64_t static_value2 = 0xbc6998636ae1b17eULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr2_gripper_sensor_msgs/PR2GripperSensorRawData";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# NOTE: This message is only for debugging purposes. It is not intended for API usage - and is not published under release code.\n\
\n\
# Standard ROS header\n\
time stamp\n\
\n\
# corrected for zero contact\n\
float64 left_finger_pad_force\n\
float64 right_finger_pad_force\n\
\n\
# filtered values : high pass filter at 5 Hz after correcting for zero contact\n\
float64 left_finger_pad_force_filtered\n\
float64 right_finger_pad_force_filtered\n\
\n\
# corrected for zero contact\n\
float64[22] left_finger_pad_forces\n\
float64[22] right_finger_pad_forces\n\
\n\
# filtered values : high pass filter at 5 Hz after correcting for zero contact\n\
float64[22] left_finger_pad_forces_filtered\n\
float64[22] right_finger_pad_forces_filtered\n\
\n\
# raw acceleration values\n\
float64 acc_x_raw\n\
float64 acc_y_raw\n\
float64 acc_z_raw\n\
\n\
# filtered acceleration values\n\
float64 acc_x_filtered\n\
float64 acc_y_filtered\n\
float64 acc_z_filtered\n\
\n\
# boolean variables indicating whether contact exists or not\n\
bool left_contact\n\
bool right_contact\n\
";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.stamp);
    stream.next(m.left_finger_pad_force);
    stream.next(m.right_finger_pad_force);
    stream.next(m.left_finger_pad_force_filtered);
    stream.next(m.right_finger_pad_force_filtered);
    stream.next(m.left_finger_pad_forces);
    stream.next(m.right_finger_pad_forces);
    stream.next(m.left_finger_pad_forces_filtered);
    stream.next(m.right_finger_pad_forces_filtered);
    stream.next(m.acc_x_raw);
    stream.next(m.acc_y_raw);
    stream.next(m.acc_z_raw);
    stream.next(m.acc_x_filtered);
    stream.next(m.acc_y_filtered);
    stream.next(m.acc_z_filtered);
    stream.next(m.left_contact);
    stream.next(m.right_contact);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PR2GripperSensorRawData_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr2_gripper_sensor_msgs::PR2GripperSensorRawData_<ContainerAllocator> & v) 
  {
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
    s << indent << "left_finger_pad_force: ";
    Printer<double>::stream(s, indent + "  ", v.left_finger_pad_force);
    s << indent << "right_finger_pad_force: ";
    Printer<double>::stream(s, indent + "  ", v.right_finger_pad_force);
    s << indent << "left_finger_pad_force_filtered: ";
    Printer<double>::stream(s, indent + "  ", v.left_finger_pad_force_filtered);
    s << indent << "right_finger_pad_force_filtered: ";
    Printer<double>::stream(s, indent + "  ", v.right_finger_pad_force_filtered);
    s << indent << "left_finger_pad_forces[]" << std::endl;
    for (size_t i = 0; i < v.left_finger_pad_forces.size(); ++i)
    {
      s << indent << "  left_finger_pad_forces[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.left_finger_pad_forces[i]);
    }
    s << indent << "right_finger_pad_forces[]" << std::endl;
    for (size_t i = 0; i < v.right_finger_pad_forces.size(); ++i)
    {
      s << indent << "  right_finger_pad_forces[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.right_finger_pad_forces[i]);
    }
    s << indent << "left_finger_pad_forces_filtered[]" << std::endl;
    for (size_t i = 0; i < v.left_finger_pad_forces_filtered.size(); ++i)
    {
      s << indent << "  left_finger_pad_forces_filtered[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.left_finger_pad_forces_filtered[i]);
    }
    s << indent << "right_finger_pad_forces_filtered[]" << std::endl;
    for (size_t i = 0; i < v.right_finger_pad_forces_filtered.size(); ++i)
    {
      s << indent << "  right_finger_pad_forces_filtered[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.right_finger_pad_forces_filtered[i]);
    }
    s << indent << "acc_x_raw: ";
    Printer<double>::stream(s, indent + "  ", v.acc_x_raw);
    s << indent << "acc_y_raw: ";
    Printer<double>::stream(s, indent + "  ", v.acc_y_raw);
    s << indent << "acc_z_raw: ";
    Printer<double>::stream(s, indent + "  ", v.acc_z_raw);
    s << indent << "acc_x_filtered: ";
    Printer<double>::stream(s, indent + "  ", v.acc_x_filtered);
    s << indent << "acc_y_filtered: ";
    Printer<double>::stream(s, indent + "  ", v.acc_y_filtered);
    s << indent << "acc_z_filtered: ";
    Printer<double>::stream(s, indent + "  ", v.acc_z_filtered);
    s << indent << "left_contact: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.left_contact);
    s << indent << "right_contact: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.right_contact);
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERSENSORRAWDATA_H

