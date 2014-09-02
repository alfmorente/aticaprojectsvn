/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication/msg/msg_irinfo.msg */
#ifndef CITIUS_CONTROL_COMMUNICATION_MESSAGE_MSG_IRINFO_H
#define CITIUS_CONTROL_COMMUNICATION_MESSAGE_MSG_IRINFO_H
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


namespace CITIUS_Control_Communication
{
template <class ContainerAllocator>
struct msg_irinfo_ {
  typedef msg_irinfo_<ContainerAllocator> Type;

  msg_irinfo_()
  : currentDZoom(0)
  , currentPolarity(0)
  {
  }

  msg_irinfo_(const ContainerAllocator& _alloc)
  : currentDZoom(0)
  , currentPolarity(0)
  {
  }

  typedef int8_t _currentDZoom_type;
  int8_t currentDZoom;

  typedef int8_t _currentPolarity_type;
  int8_t currentPolarity;


  typedef boost::shared_ptr< ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct msg_irinfo
typedef  ::CITIUS_Control_Communication::msg_irinfo_<std::allocator<void> > msg_irinfo;

typedef boost::shared_ptr< ::CITIUS_Control_Communication::msg_irinfo> msg_irinfoPtr;
typedef boost::shared_ptr< ::CITIUS_Control_Communication::msg_irinfo const> msg_irinfoConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace CITIUS_Control_Communication

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cd6587ca073694f8b8aa0f66b1d602b8";
  }

  static const char* value(const  ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xcd6587ca073694f8ULL;
  static const uint64_t static_value2 = 0xb8aa0f66b1d602b8ULL;
};

template<class ContainerAllocator>
struct DataType< ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/msg_irinfo";
  }

  static const char* value(const  ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 currentDZoom\n\
int8 currentPolarity\n\
\n\
";
  }

  static const char* value(const  ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.currentDZoom);
    stream.next(m.currentPolarity);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct msg_irinfo_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::CITIUS_Control_Communication::msg_irinfo_<ContainerAllocator> & v) 
  {
    s << indent << "currentDZoom: ";
    Printer<int8_t>::stream(s, indent + "  ", v.currentDZoom);
    s << indent << "currentPolarity: ";
    Printer<int8_t>::stream(s, indent + "  ", v.currentPolarity);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CITIUS_CONTROL_COMMUNICATION_MESSAGE_MSG_IRINFO_H

