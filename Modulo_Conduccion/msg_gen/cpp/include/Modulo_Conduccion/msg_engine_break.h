/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/Modulo_Conduccion/msg/msg_engine_break.msg */
#ifndef MODULO_CONDUCCION_MESSAGE_MSG_ENGINE_BREAK_H
#define MODULO_CONDUCCION_MESSAGE_MSG_ENGINE_BREAK_H
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


namespace Modulo_Conduccion
{
template <class ContainerAllocator>
struct msg_engine_break_ {
  typedef msg_engine_break_<ContainerAllocator> Type;

  msg_engine_break_()
  : command(false)
  , value(false)
  {
  }

  msg_engine_break_(const ContainerAllocator& _alloc)
  : command(false)
  , value(false)
  {
  }

  typedef uint8_t _command_type;
  uint8_t command;

  typedef uint8_t _value_type;
  uint8_t value;


  typedef boost::shared_ptr< ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct msg_engine_break
typedef  ::Modulo_Conduccion::msg_engine_break_<std::allocator<void> > msg_engine_break;

typedef boost::shared_ptr< ::Modulo_Conduccion::msg_engine_break> msg_engine_breakPtr;
typedef boost::shared_ptr< ::Modulo_Conduccion::msg_engine_break const> msg_engine_breakConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace Modulo_Conduccion

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> > {
  static const char* value() 
  {
    return "afabe79133bf3b438c176b9a4d0a4902";
  }

  static const char* value(const  ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xafabe79133bf3b43ULL;
  static const uint64_t static_value2 = 0x8c176b9a4d0a4902ULL;
};

template<class ContainerAllocator>
struct DataType< ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Modulo_Conduccion/msg_engine_break";
  }

  static const char* value(const  ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool command\n\
bool value      \n\
\n\
";
  }

  static const char* value(const  ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.command);
    stream.next(m.value);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct msg_engine_break_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::Modulo_Conduccion::msg_engine_break_<ContainerAllocator> & v) 
  {
    s << indent << "command: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.command);
    s << indent << "value: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.value);
  }
};


} // namespace message_operations
} // namespace ros

#endif // MODULO_CONDUCCION_MESSAGE_MSG_ENGINE_BREAK_H

