/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication/srv/srv_tiltAbsolutePosition.srv */
#ifndef CITIUS_CONTROL_COMMUNICATION_SERVICE_SRV_TILTABSOLUTEPOSITION_H
#define CITIUS_CONTROL_COMMUNICATION_SERVICE_SRV_TILTABSOLUTEPOSITION_H
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
struct srv_tiltAbsolutePositionRequest_ {
  typedef srv_tiltAbsolutePositionRequest_<ContainerAllocator> Type;

  srv_tiltAbsolutePositionRequest_()
  : tiltPosition(0)
  {
  }

  srv_tiltAbsolutePositionRequest_(const ContainerAllocator& _alloc)
  : tiltPosition(0)
  {
  }

  typedef int16_t _tiltPosition_type;
  int16_t tiltPosition;


  typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct srv_tiltAbsolutePositionRequest
typedef  ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<std::allocator<void> > srv_tiltAbsolutePositionRequest;

typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest> srv_tiltAbsolutePositionRequestPtr;
typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest const> srv_tiltAbsolutePositionRequestConstPtr;



template <class ContainerAllocator>
struct srv_tiltAbsolutePositionResponse_ {
  typedef srv_tiltAbsolutePositionResponse_<ContainerAllocator> Type;

  srv_tiltAbsolutePositionResponse_()
  : ret(false)
  {
  }

  srv_tiltAbsolutePositionResponse_(const ContainerAllocator& _alloc)
  : ret(false)
  {
  }

  typedef uint8_t _ret_type;
  uint8_t ret;


  typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct srv_tiltAbsolutePositionResponse
typedef  ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<std::allocator<void> > srv_tiltAbsolutePositionResponse;

typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse> srv_tiltAbsolutePositionResponsePtr;
typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse const> srv_tiltAbsolutePositionResponseConstPtr;


struct srv_tiltAbsolutePosition
{

typedef srv_tiltAbsolutePositionRequest Request;
typedef srv_tiltAbsolutePositionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct srv_tiltAbsolutePosition
} // namespace CITIUS_Control_Communication

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fc4ee16a9338d267bc70877b065095ae";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xfc4ee16a9338d267ULL;
  static const uint64_t static_value2 = 0xbc70877b065095aeULL;
};

template<class ContainerAllocator>
struct DataType< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_tiltAbsolutePositionRequest";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int16 tiltPosition\n\
\n\
";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e2cc9e9d8c464550830df49c160979ad";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe2cc9e9d8c464550ULL;
  static const uint64_t static_value2 = 0x830df49c160979adULL;
};

template<class ContainerAllocator>
struct DataType< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_tiltAbsolutePositionResponse";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool ret\n\
\n\
\n\
";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.tiltPosition);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct srv_tiltAbsolutePositionRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ret);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct srv_tiltAbsolutePositionResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<CITIUS_Control_Communication::srv_tiltAbsolutePosition> {
  static const char* value() 
  {
    return "2b3eede2c56ea928c920c1e9b71325fe";
  }

  static const char* value(const CITIUS_Control_Communication::srv_tiltAbsolutePosition&) { return value(); } 
};

template<>
struct DataType<CITIUS_Control_Communication::srv_tiltAbsolutePosition> {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_tiltAbsolutePosition";
  }

  static const char* value(const CITIUS_Control_Communication::srv_tiltAbsolutePosition&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2b3eede2c56ea928c920c1e9b71325fe";
  }

  static const char* value(const CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_tiltAbsolutePosition";
  }

  static const char* value(const CITIUS_Control_Communication::srv_tiltAbsolutePositionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2b3eede2c56ea928c920c1e9b71325fe";
  }

  static const char* value(const CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_tiltAbsolutePosition";
  }

  static const char* value(const CITIUS_Control_Communication::srv_tiltAbsolutePositionResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // CITIUS_CONTROL_COMMUNICATION_SERVICE_SRV_TILTABSOLUTEPOSITION_H
