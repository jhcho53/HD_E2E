// Generated by gencpp from file morai_msgs/FaultStatusInfo.msg
// DO NOT EDIT!


#ifndef MORAI_MSGS_MESSAGE_FAULTSTATUSINFO_H
#define MORAI_MSGS_MESSAGE_FAULTSTATUSINFO_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <morai_msgs/FaultStatusInfo_Vehicle.h>
#include <morai_msgs/FaultStatusInfo_Sensor.h>

namespace morai_msgs
{
template <class ContainerAllocator>
struct FaultStatusInfo_
{
  typedef FaultStatusInfo_<ContainerAllocator> Type;

  FaultStatusInfo_()
    : header()
    , unique_id(0)
    , vehicle()
    , sensors()  {
    }
  FaultStatusInfo_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , unique_id(0)
    , vehicle(_alloc)
    , sensors(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _unique_id_type;
  _unique_id_type unique_id;

   typedef  ::morai_msgs::FaultStatusInfo_Vehicle_<ContainerAllocator>  _vehicle_type;
  _vehicle_type vehicle;

   typedef std::vector< ::morai_msgs::FaultStatusInfo_Sensor_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::morai_msgs::FaultStatusInfo_Sensor_<ContainerAllocator> >> _sensors_type;
  _sensors_type sensors;





  typedef boost::shared_ptr< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> const> ConstPtr;

}; // struct FaultStatusInfo_

typedef ::morai_msgs::FaultStatusInfo_<std::allocator<void> > FaultStatusInfo;

typedef boost::shared_ptr< ::morai_msgs::FaultStatusInfo > FaultStatusInfoPtr;
typedef boost::shared_ptr< ::morai_msgs::FaultStatusInfo const> FaultStatusInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::morai_msgs::FaultStatusInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::morai_msgs::FaultStatusInfo_<ContainerAllocator1> & lhs, const ::morai_msgs::FaultStatusInfo_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.unique_id == rhs.unique_id &&
    lhs.vehicle == rhs.vehicle &&
    lhs.sensors == rhs.sensors;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::morai_msgs::FaultStatusInfo_<ContainerAllocator1> & lhs, const ::morai_msgs::FaultStatusInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace morai_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "514c099d9ef1148c019de61e11a471bb";
  }

  static const char* value(const ::morai_msgs::FaultStatusInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x514c099d9ef1148cULL;
  static const uint64_t static_value2 = 0x019de61e11a471bbULL;
};

template<class ContainerAllocator>
struct DataType< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "morai_msgs/FaultStatusInfo";
  }

  static const char* value(const ::morai_msgs::FaultStatusInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"int32 unique_id\n"
"FaultStatusInfo_Vehicle vehicle\n"
"FaultStatusInfo_Sensor[] sensors\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: morai_msgs/FaultStatusInfo_Vehicle\n"
"FaultStatusInfo_Overall accel\n"
"FaultStatusInfo_Overall brake\n"
"FaultStatusInfo_Overall steer\n"
"FaultStatusInfo_Overall[] tires\n"
"\n"
"\n"
"================================================================================\n"
"MSG: morai_msgs/FaultStatusInfo_Overall\n"
"bool status\n"
"int32[] fault_subclass\n"
"\n"
"================================================================================\n"
"MSG: morai_msgs/FaultStatusInfo_Sensor\n"
"int32 sensor_id\n"
"FaultStatusInfo_Overall sensor\n"
"\n"
;
  }

  static const char* value(const ::morai_msgs::FaultStatusInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.unique_id);
      stream.next(m.vehicle);
      stream.next(m.sensors);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FaultStatusInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::morai_msgs::FaultStatusInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::morai_msgs::FaultStatusInfo_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "unique_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.unique_id);
    s << indent << "vehicle: ";
    s << std::endl;
    Printer< ::morai_msgs::FaultStatusInfo_Vehicle_<ContainerAllocator> >::stream(s, indent + "  ", v.vehicle);
    s << indent << "sensors[]" << std::endl;
    for (size_t i = 0; i < v.sensors.size(); ++i)
    {
      s << indent << "  sensors[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::morai_msgs::FaultStatusInfo_Sensor_<ContainerAllocator> >::stream(s, indent + "    ", v.sensors[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MORAI_MSGS_MESSAGE_FAULTSTATUSINFO_H
