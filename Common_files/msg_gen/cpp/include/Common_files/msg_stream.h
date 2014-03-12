/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/Common_files/msg/msg_stream.msg */
#ifndef COMMON_FILES_MESSAGE_MSG_STREAM_H
#define COMMON_FILES_MESSAGE_MSG_STREAM_H
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


namespace Common_files
{
template <class ContainerAllocator>
struct msg_stream_ {
  typedef msg_stream_<ContainerAllocator> Type;

  msg_stream_()
  : id_file(0)
  , stream()
  {
  }

  msg_stream_(const ContainerAllocator& _alloc)
  : id_file(0)
  , stream(_alloc)
  {
  }

  typedef uint8_t _id_file_type;
  uint8_t id_file;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _stream_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  stream;


  typedef boost::shared_ptr< ::Common_files::msg_stream_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Common_files::msg_stream_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct msg_stream
typedef  ::Common_files::msg_stream_<std::allocator<void> > msg_stream;

typedef boost::shared_ptr< ::Common_files::msg_stream> msg_streamPtr;
typedef boost::shared_ptr< ::Common_files::msg_stream const> msg_streamConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::Common_files::msg_stream_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::Common_files::msg_stream_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace Common_files

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::Common_files::msg_stream_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::Common_files::msg_stream_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::Common_files::msg_stream_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3a57dd0fae35fb2128ca62b071bdcfb2";
  }

  static const char* value(const  ::Common_files::msg_stream_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3a57dd0fae35fb21ULL;
  static const uint64_t static_value2 = 0x28ca62b071bdcfb2ULL;
};

template<class ContainerAllocator>
struct DataType< ::Common_files::msg_stream_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Common_files/msg_stream";
  }

  static const char* value(const  ::Common_files::msg_stream_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::Common_files::msg_stream_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 id_file\n\
string stream\n\
";
  }

  static const char* value(const  ::Common_files::msg_stream_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::Common_files::msg_stream_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.id_file);
    stream.next(m.stream);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct msg_stream_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::Common_files::msg_stream_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::Common_files::msg_stream_<ContainerAllocator> & v) 
  {
    s << indent << "id_file: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id_file);
    s << indent << "stream: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.stream);
  }
};


} // namespace message_operations
} // namespace ros

#endif // COMMON_FILES_MESSAGE_MSG_STREAM_H
