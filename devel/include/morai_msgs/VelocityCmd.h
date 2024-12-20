// Generated by gencpp from file morai_msgs/VelocityCmd.msg
// DO NOT EDIT!


#ifndef MORAI_MSGS_MESSAGE_VELOCITYCMD_H
#define MORAI_MSGS_MESSAGE_VELOCITYCMD_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace morai_msgs
{
template <class ContainerAllocator>
struct VelocityCmd_
{
  typedef VelocityCmd_<ContainerAllocator> Type;

  VelocityCmd_()
    : linear()
    , angular()  {
    }
  VelocityCmd_(const ContainerAllocator& _alloc)
    : linear(_alloc)
    , angular(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _linear_type;
  _linear_type linear;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _angular_type;
  _angular_type angular;





  typedef boost::shared_ptr< ::morai_msgs::VelocityCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::morai_msgs::VelocityCmd_<ContainerAllocator> const> ConstPtr;

}; // struct VelocityCmd_

typedef ::morai_msgs::VelocityCmd_<std::allocator<void> > VelocityCmd;

typedef boost::shared_ptr< ::morai_msgs::VelocityCmd > VelocityCmdPtr;
typedef boost::shared_ptr< ::morai_msgs::VelocityCmd const> VelocityCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::morai_msgs::VelocityCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::morai_msgs::VelocityCmd_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::morai_msgs::VelocityCmd_<ContainerAllocator1> & lhs, const ::morai_msgs::VelocityCmd_<ContainerAllocator2> & rhs)
{
  return lhs.linear == rhs.linear &&
    lhs.angular == rhs.angular;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::morai_msgs::VelocityCmd_<ContainerAllocator1> & lhs, const ::morai_msgs::VelocityCmd_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace morai_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::morai_msgs::VelocityCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::morai_msgs::VelocityCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::morai_msgs::VelocityCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::morai_msgs::VelocityCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::morai_msgs::VelocityCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::morai_msgs::VelocityCmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::morai_msgs::VelocityCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9f195f881246fdfa2798d1d3eebca84a";
  }

  static const char* value(const ::morai_msgs::VelocityCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9f195f881246fdfaULL;
  static const uint64_t static_value2 = 0x2798d1d3eebca84aULL;
};

template<class ContainerAllocator>
struct DataType< ::morai_msgs::VelocityCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "morai_msgs/VelocityCmd";
  }

  static const char* value(const ::morai_msgs::VelocityCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::morai_msgs::VelocityCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Vector3 linear\n"
"geometry_msgs/Vector3 angular\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::morai_msgs::VelocityCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::morai_msgs::VelocityCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.linear);
      stream.next(m.angular);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VelocityCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::morai_msgs::VelocityCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::morai_msgs::VelocityCmd_<ContainerAllocator>& v)
  {
    s << indent << "linear: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.linear);
    s << indent << "angular: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.angular);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MORAI_MSGS_MESSAGE_VELOCITYCMD_H