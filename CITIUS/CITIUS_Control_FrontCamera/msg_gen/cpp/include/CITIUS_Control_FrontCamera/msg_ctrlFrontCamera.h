/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/msg/msg_ctrlFrontCamera.msg */
#ifndef CITIUS_CONTROL_FRONTCAMERA_MESSAGE_MSG_CTRLFRONTCAMERA_H
#define CITIUS_CONTROL_FRONTCAMERA_MESSAGE_MSG_CTRLFRONTCAMERA_H
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


namespace CITIUS_Control_FrontCamera
{
template <class ContainerAllocator>
struct msg_ctrlFrontCamera_ {
  typedef msg_ctrlFrontCamera_<ContainerAllocator> Type;

  msg_ctrlFrontCamera_()
  : pan(0.0)
  , tilt(0.0)
  {
  }

  msg_ctrlFrontCamera_(const ContainerAllocator& _alloc)
  : pan(0.0)
  , tilt(0.0)
  {
  }

  typedef float _pan_type;
  float pan;

  typedef float _tilt_type;
  float tilt;


  typedef boost::shared_ptr< ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct msg_ctrlFrontCamera
typedef  ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<std::allocator<void> > msg_ctrlFrontCamera;

typedef boost::shared_ptr< ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera> msg_ctrlFrontCameraPtr;
typedef boost::shared_ptr< ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera const> msg_ctrlFrontCameraConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace CITIUS_Control_FrontCamera

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> > {
  static const char* value() 
  {
    return "938e11f380abc0513a5b7367d0f157bf";
  }

  static const char* value(const  ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x938e11f380abc051ULL;
  static const uint64_t static_value2 = 0x3a5b7367d0f157bfULL;
};

template<class ContainerAllocator>
struct DataType< ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_FrontCamera/msg_ctrlFrontCamera";
  }

  static const char* value(const  ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 pan\n\
float32 tilt\n\
";
  }

  static const char* value(const  ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.pan);
    stream.next(m.tilt);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct msg_ctrlFrontCamera_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::CITIUS_Control_FrontCamera::msg_ctrlFrontCamera_<ContainerAllocator> & v) 
  {
    s << indent << "pan: ";
    Printer<float>::stream(s, indent + "  ", v.pan);
    s << indent << "tilt: ";
    Printer<float>::stream(s, indent + "  ", v.tilt);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CITIUS_CONTROL_FRONTCAMERA_MESSAGE_MSG_CTRLFRONTCAMERA_H

