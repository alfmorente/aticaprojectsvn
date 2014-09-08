/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication/srv/srv_panAbsolutePosition.srv */
#ifndef CITIUS_CONTROL_COMMUNICATION_SERVICE_SRV_PANABSOLUTEPOSITION_H
#define CITIUS_CONTROL_COMMUNICATION_SERVICE_SRV_PANABSOLUTEPOSITION_H
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

#include "ros/service_traits.h"




namespace CITIUS_Control_Communication
{
template <class ContainerAllocator>
struct srv_panAbsolutePositionRequest_ {
  typedef srv_panAbsolutePositionRequest_<ContainerAllocator> Type;

  srv_panAbsolutePositionRequest_()
  : panPosition(0)
  {
  }

  srv_panAbsolutePositionRequest_(const ContainerAllocator& _alloc)
  : panPosition(0)
  {
  }

  typedef uint16_t _panPosition_type;
  uint16_t panPosition;


  typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct srv_panAbsolutePositionRequest
typedef  ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<std::allocator<void> > srv_panAbsolutePositionRequest;

typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest> srv_panAbsolutePositionRequestPtr;
typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest const> srv_panAbsolutePositionRequestConstPtr;



template <class ContainerAllocator>
struct srv_panAbsolutePositionResponse_ {
  typedef srv_panAbsolutePositionResponse_<ContainerAllocator> Type;

  srv_panAbsolutePositionResponse_()
  : ret(false)
  {
  }

  srv_panAbsolutePositionResponse_(const ContainerAllocator& _alloc)
  : ret(false)
  {
  }

  typedef uint8_t _ret_type;
  uint8_t ret;


  typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct srv_panAbsolutePositionResponse
typedef  ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<std::allocator<void> > srv_panAbsolutePositionResponse;

typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse> srv_panAbsolutePositionResponsePtr;
typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse const> srv_panAbsolutePositionResponseConstPtr;


struct srv_panAbsolutePosition
{

typedef srv_panAbsolutePositionRequest Request;
typedef srv_panAbsolutePositionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct srv_panAbsolutePosition
} // namespace CITIUS_Control_Communication

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "120d37aa497909e44d6b132a2614a3c9";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x120d37aa497909e4ULL;
  static const uint64_t static_value2 = 0x4d6b132a2614a3c9ULL;
};

template<class ContainerAllocator>
struct DataType< ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_panAbsolutePositionRequest";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint16 panPosition\n\
\n\
";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e2cc9e9d8c464550830df49c160979ad";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe2cc9e9d8c464550ULL;
  static const uint64_t static_value2 = 0x830df49c160979adULL;
};

template<class ContainerAllocator>
struct DataType< ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_panAbsolutePositionResponse";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool ret\n\
\n\
\n\
";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.panPosition);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct srv_panAbsolutePositionRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ret);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct srv_panAbsolutePositionResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<CITIUS_Control_Communication::srv_panAbsolutePosition> {
  static const char* value() 
  {
    return "00c8e1dcba95ff90c779d8aea4279dc8";
  }

  static const char* value(const CITIUS_Control_Communication::srv_panAbsolutePosition&) { return value(); } 
};

template<>
struct DataType<CITIUS_Control_Communication::srv_panAbsolutePosition> {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_panAbsolutePosition";
  }

  static const char* value(const CITIUS_Control_Communication::srv_panAbsolutePosition&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "00c8e1dcba95ff90c779d8aea4279dc8";
  }

  static const char* value(const CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_panAbsolutePosition";
  }

  static const char* value(const CITIUS_Control_Communication::srv_panAbsolutePositionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "00c8e1dcba95ff90c779d8aea4279dc8";
  }

  static const char* value(const CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_panAbsolutePosition";
  }

  static const char* value(const CITIUS_Control_Communication::srv_panAbsolutePositionResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // CITIUS_CONTROL_COMMUNICATION_SERVICE_SRV_PANABSOLUTEPOSITION_H
