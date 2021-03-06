/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/ATICA_Bobcat/Driving_Bobcat/msg/msg_switcher.msg */
#ifndef DRIVING_BOBCAT_MESSAGE_MSG_SWITCHER_H
#define DRIVING_BOBCAT_MESSAGE_MSG_SWITCHER_H
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


namespace Driving_Bobcat
{
template <class ContainerAllocator>
struct msg_switcher_ {
  typedef msg_switcher_<ContainerAllocator> Type;

  msg_switcher_()
  : switcher(0)
  {
  }

  msg_switcher_(const ContainerAllocator& _alloc)
  : switcher(0)
  {
  }

  typedef uint8_t _switcher_type;
  uint8_t switcher;


  typedef boost::shared_ptr< ::Driving_Bobcat::msg_switcher_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Driving_Bobcat::msg_switcher_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct msg_switcher
typedef  ::Driving_Bobcat::msg_switcher_<std::allocator<void> > msg_switcher;

typedef boost::shared_ptr< ::Driving_Bobcat::msg_switcher> msg_switcherPtr;
typedef boost::shared_ptr< ::Driving_Bobcat::msg_switcher const> msg_switcherConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::Driving_Bobcat::msg_switcher_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::Driving_Bobcat::msg_switcher_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace Driving_Bobcat

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::Driving_Bobcat::msg_switcher_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::Driving_Bobcat::msg_switcher_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::Driving_Bobcat::msg_switcher_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6f9f7ff47283f474d33dbf2d2963b97e";
  }

  static const char* value(const  ::Driving_Bobcat::msg_switcher_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6f9f7ff47283f474ULL;
  static const uint64_t static_value2 = 0xd33dbf2d2963b97eULL;
};

template<class ContainerAllocator>
struct DataType< ::Driving_Bobcat::msg_switcher_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Driving_Bobcat/msg_switcher";
  }

  static const char* value(const  ::Driving_Bobcat::msg_switcher_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::Driving_Bobcat::msg_switcher_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 switcher\n\
";
  }

  static const char* value(const  ::Driving_Bobcat::msg_switcher_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::Driving_Bobcat::msg_switcher_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::Driving_Bobcat::msg_switcher_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.switcher);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct msg_switcher_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::Driving_Bobcat::msg_switcher_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::Driving_Bobcat::msg_switcher_<ContainerAllocator> & v) 
  {
    s << indent << "switcher: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.switcher);
  }
};


} // namespace message_operations
} // namespace ros

#endif // DRIVING_BOBCAT_MESSAGE_MSG_SWITCHER_H

