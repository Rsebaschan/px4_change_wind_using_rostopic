// Generated by gencpp from file mavros_msgs/SetMavFrameResponse.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_SETMAVFRAMERESPONSE_H
#define MAVROS_MSGS_MESSAGE_SETMAVFRAMERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mavros_msgs
{
template <class ContainerAllocator>
struct SetMavFrameResponse_
{
  typedef SetMavFrameResponse_<ContainerAllocator> Type;

  SetMavFrameResponse_()
    : success(false)  {
    }
  SetMavFrameResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SetMavFrameResponse_

typedef ::mavros_msgs::SetMavFrameResponse_<std::allocator<void> > SetMavFrameResponse;

typedef boost::shared_ptr< ::mavros_msgs::SetMavFrameResponse > SetMavFrameResponsePtr;
typedef boost::shared_ptr< ::mavros_msgs::SetMavFrameResponse const> SetMavFrameResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator1> & lhs, const ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator1> & lhs, const ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/SetMavFrameResponse";
  }

  static const char* value(const ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"\n"
;
  }

  static const char* value(const ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetMavFrameResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mavros_msgs::SetMavFrameResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_SETMAVFRAMERESPONSE_H
