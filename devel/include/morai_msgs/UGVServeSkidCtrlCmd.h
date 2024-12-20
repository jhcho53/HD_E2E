// Generated by gencpp from file morai_msgs/UGVServeSkidCtrlCmd.msg
// DO NOT EDIT!


#ifndef MORAI_MSGS_MESSAGE_UGVSERVESKIDCTRLCMD_H
#define MORAI_MSGS_MESSAGE_UGVSERVESKIDCTRLCMD_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace morai_msgs
{
template <class ContainerAllocator>
struct UGVServeSkidCtrlCmd_
{
  typedef UGVServeSkidCtrlCmd_<ContainerAllocator> Type;

  UGVServeSkidCtrlCmd_()
    : steer_mode(0)
    , forward(0.0)
    , skid_steering(0.0)
    , ackermann_steering(0.0)
    , ackermann_steering_rear_ratio(0.0)  {
    }
  UGVServeSkidCtrlCmd_(const ContainerAllocator& _alloc)
    : steer_mode(0)
    , forward(0.0)
    , skid_steering(0.0)
    , ackermann_steering(0.0)
    , ackermann_steering_rear_ratio(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _steer_mode_type;
  _steer_mode_type steer_mode;

   typedef float _forward_type;
  _forward_type forward;

   typedef float _skid_steering_type;
  _skid_steering_type skid_steering;

   typedef float _ackermann_steering_type;
  _ackermann_steering_type ackermann_steering;

   typedef float _ackermann_steering_rear_ratio_type;
  _ackermann_steering_rear_ratio_type ackermann_steering_rear_ratio;





  typedef boost::shared_ptr< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> const> ConstPtr;

}; // struct UGVServeSkidCtrlCmd_

typedef ::morai_msgs::UGVServeSkidCtrlCmd_<std::allocator<void> > UGVServeSkidCtrlCmd;

typedef boost::shared_ptr< ::morai_msgs::UGVServeSkidCtrlCmd > UGVServeSkidCtrlCmdPtr;
typedef boost::shared_ptr< ::morai_msgs::UGVServeSkidCtrlCmd const> UGVServeSkidCtrlCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator1> & lhs, const ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator2> & rhs)
{
  return lhs.steer_mode == rhs.steer_mode &&
    lhs.forward == rhs.forward &&
    lhs.skid_steering == rhs.skid_steering &&
    lhs.ackermann_steering == rhs.ackermann_steering &&
    lhs.ackermann_steering_rear_ratio == rhs.ackermann_steering_rear_ratio;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator1> & lhs, const ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace morai_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4f54afce325790f6ec15cfae04a11605";
  }

  static const char* value(const ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4f54afce325790f6ULL;
  static const uint64_t static_value2 = 0xec15cfae04a11605ULL;
};

template<class ContainerAllocator>
struct DataType< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "morai_msgs/UGVServeSkidCtrlCmd";
  }

  static const char* value(const ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 steer_mode\n"
"float32 forward\n"
"float32 skid_steering\n"
"float32 ackermann_steering\n"
"float32 ackermann_steering_rear_ratio\n"
;
  }

  static const char* value(const ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.steer_mode);
      stream.next(m.forward);
      stream.next(m.skid_steering);
      stream.next(m.ackermann_steering);
      stream.next(m.ackermann_steering_rear_ratio);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UGVServeSkidCtrlCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::morai_msgs::UGVServeSkidCtrlCmd_<ContainerAllocator>& v)
  {
    s << indent << "steer_mode: ";
    Printer<int32_t>::stream(s, indent + "  ", v.steer_mode);
    s << indent << "forward: ";
    Printer<float>::stream(s, indent + "  ", v.forward);
    s << indent << "skid_steering: ";
    Printer<float>::stream(s, indent + "  ", v.skid_steering);
    s << indent << "ackermann_steering: ";
    Printer<float>::stream(s, indent + "  ", v.ackermann_steering);
    s << indent << "ackermann_steering_rear_ratio: ";
    Printer<float>::stream(s, indent + "  ", v.ackermann_steering_rear_ratio);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MORAI_MSGS_MESSAGE_UGVSERVESKIDCTRLCMD_H
