/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-pr2-common-1.10.3/debian/ros-groovy-pr2-common/opt/ros/groovy/stacks/pr2_common/pr2_msgs/msg/BatteryState2.msg */
#ifndef PR2_MSGS_MESSAGE_BATTERYSTATE2_H
#define PR2_MSGS_MESSAGE_BATTERYSTATE2_H
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


namespace pr2_msgs
{
template <class ContainerAllocator>
struct BatteryState2_ {
  typedef BatteryState2_<ContainerAllocator> Type;

  BatteryState2_()
  : present(false)
  , charging(false)
  , discharging(false)
  , power_present(false)
  , power_no_good(false)
  , inhibited(false)
  , last_battery_update()
  , battery_register()
  , battery_update_flag()
  , battery_register_update()
  {
    battery_register.assign(0);
    battery_update_flag.assign(false);
  }

  BatteryState2_(const ContainerAllocator& _alloc)
  : present(false)
  , charging(false)
  , discharging(false)
  , power_present(false)
  , power_no_good(false)
  , inhibited(false)
  , last_battery_update()
  , battery_register()
  , battery_update_flag()
  , battery_register_update()
  {
    battery_register.assign(0);
    battery_update_flag.assign(false);
  }

  typedef uint8_t _present_type;
  uint8_t present;

  typedef uint8_t _charging_type;
  uint8_t charging;

  typedef uint8_t _discharging_type;
  uint8_t discharging;

  typedef uint8_t _power_present_type;
  uint8_t power_present;

  typedef uint8_t _power_no_good_type;
  uint8_t power_no_good;

  typedef uint8_t _inhibited_type;
  uint8_t inhibited;

  typedef ros::Time _last_battery_update_type;
  ros::Time last_battery_update;

  typedef boost::array<int16_t, 48>  _battery_register_type;
  boost::array<int16_t, 48>  battery_register;

  typedef boost::array<uint8_t, 48>  _battery_update_flag_type;
  boost::array<uint8_t, 48>  battery_update_flag;

  typedef boost::array<ros::Time, 48>  _battery_register_update_type;
  boost::array<ros::Time, 48>  battery_register_update;


  typedef boost::shared_ptr< ::pr2_msgs::BatteryState2_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_msgs::BatteryState2_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BatteryState2
typedef  ::pr2_msgs::BatteryState2_<std::allocator<void> > BatteryState2;

typedef boost::shared_ptr< ::pr2_msgs::BatteryState2> BatteryState2Ptr;
typedef boost::shared_ptr< ::pr2_msgs::BatteryState2 const> BatteryState2ConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr2_msgs::BatteryState2_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr2_msgs::BatteryState2_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr2_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr2_msgs::BatteryState2_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr2_msgs::BatteryState2_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr2_msgs::BatteryState2_<ContainerAllocator> > {
  static const char* value() 
  {
    return "91b4acb000aa990ac3006834f9a99669";
  }

  static const char* value(const  ::pr2_msgs::BatteryState2_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x91b4acb000aa990aULL;
  static const uint64_t static_value2 = 0xc3006834f9a99669ULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_msgs::BatteryState2_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr2_msgs/BatteryState2";
  }

  static const char* value(const  ::pr2_msgs::BatteryState2_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr2_msgs::BatteryState2_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# This message communicates the state of a single battery.\n\
# Battery Controller Flags, one per battery\n\
bool present       # is this pack present\n\
bool charging      # is this pack charging\n\
bool discharging   # is this pack discharging\n\
bool power_present # is there an input voltage\n\
bool power_no_good # is there a fault (No Good)\n\
bool inhibited     # is this pack disabled for some reason\n\
# These registers are per battery\n\
time      last_battery_update     # last time any battery update occurred\n\
int16[48] battery_register        # value of this register in the battery\n\
bool[48]  battery_update_flag     # Has this register ever been updated\n\
time[48]  battery_register_update # last time this specific register was updated\n\
\n\
";
  }

  static const char* value(const  ::pr2_msgs::BatteryState2_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::pr2_msgs::BatteryState2_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr2_msgs::BatteryState2_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.present);
    stream.next(m.charging);
    stream.next(m.discharging);
    stream.next(m.power_present);
    stream.next(m.power_no_good);
    stream.next(m.inhibited);
    stream.next(m.last_battery_update);
    stream.next(m.battery_register);
    stream.next(m.battery_update_flag);
    stream.next(m.battery_register_update);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BatteryState2_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_msgs::BatteryState2_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr2_msgs::BatteryState2_<ContainerAllocator> & v) 
  {
    s << indent << "present: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.present);
    s << indent << "charging: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.charging);
    s << indent << "discharging: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.discharging);
    s << indent << "power_present: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.power_present);
    s << indent << "power_no_good: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.power_no_good);
    s << indent << "inhibited: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.inhibited);
    s << indent << "last_battery_update: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.last_battery_update);
    s << indent << "battery_register[]" << std::endl;
    for (size_t i = 0; i < v.battery_register.size(); ++i)
    {
      s << indent << "  battery_register[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.battery_register[i]);
    }
    s << indent << "battery_update_flag[]" << std::endl;
    for (size_t i = 0; i < v.battery_update_flag.size(); ++i)
    {
      s << indent << "  battery_update_flag[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.battery_update_flag[i]);
    }
    s << indent << "battery_register_update[]" << std::endl;
    for (size_t i = 0; i < v.battery_register_update.size(); ++i)
    {
      s << indent << "  battery_register_update[" << i << "]: ";
      Printer<ros::Time>::stream(s, indent + "  ", v.battery_register_update[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR2_MSGS_MESSAGE_BATTERYSTATE2_H

