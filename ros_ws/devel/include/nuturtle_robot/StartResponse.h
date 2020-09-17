// Generated by gencpp from file nuturtle_robot/StartResponse.msg
// DO NOT EDIT!


#ifndef NUTURTLE_ROBOT_MESSAGE_STARTRESPONSE_H
#define NUTURTLE_ROBOT_MESSAGE_STARTRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace nuturtle_robot
{
template <class ContainerAllocator>
struct StartResponse_
{
  typedef StartResponse_<ContainerAllocator> Type;

  StartResponse_()
    {
    }
  StartResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::nuturtle_robot::StartResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nuturtle_robot::StartResponse_<ContainerAllocator> const> ConstPtr;

}; // struct StartResponse_

typedef ::nuturtle_robot::StartResponse_<std::allocator<void> > StartResponse;

typedef boost::shared_ptr< ::nuturtle_robot::StartResponse > StartResponsePtr;
typedef boost::shared_ptr< ::nuturtle_robot::StartResponse const> StartResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nuturtle_robot::StartResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::nuturtle_robot::StartResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace nuturtle_robot

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::nuturtle_robot::StartResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nuturtle_robot::StartResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nuturtle_robot::StartResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nuturtle_robot::StartResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nuturtle_robot::StartResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nuturtle_robot::StartResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nuturtle_robot::StartResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::nuturtle_robot::StartResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::nuturtle_robot::StartResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nuturtle_robot/StartResponse";
  }

  static const char* value(const ::nuturtle_robot::StartResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nuturtle_robot::StartResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::nuturtle_robot::StartResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nuturtle_robot::StartResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StartResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nuturtle_robot::StartResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::nuturtle_robot::StartResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // NUTURTLE_ROBOT_MESSAGE_STARTRESPONSE_H
