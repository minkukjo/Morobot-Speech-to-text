// Generated by gencpp from file ar_track_alvar_msgs/AlvarMarker.msg
// DO NOT EDIT!


#ifndef AR_TRACK_ALVAR_MSGS_MESSAGE_ALVARMARKER_H
#define AR_TRACK_ALVAR_MSGS_MESSAGE_ALVARMARKER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>

namespace ar_track_alvar_msgs
{
template <class ContainerAllocator>
struct AlvarMarker_
{
  typedef AlvarMarker_<ContainerAllocator> Type;

  AlvarMarker_()
    : header()
    , id(0)
    , confidence(0)
    , pose()  {
    }
  AlvarMarker_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , id(0)
    , confidence(0)
    , pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint32_t _id_type;
  _id_type id;

   typedef uint32_t _confidence_type;
  _confidence_type confidence;

   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _pose_type;
  _pose_type pose;





  typedef boost::shared_ptr< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> const> ConstPtr;

}; // struct AlvarMarker_

typedef ::ar_track_alvar_msgs::AlvarMarker_<std::allocator<void> > AlvarMarker;

typedef boost::shared_ptr< ::ar_track_alvar_msgs::AlvarMarker > AlvarMarkerPtr;
typedef boost::shared_ptr< ::ar_track_alvar_msgs::AlvarMarker const> AlvarMarkerConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ar_track_alvar_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'ar_track_alvar_msgs': ['/home/cyj/catkin_ws/src/ar_track_alvar-kinetic-devel/ar_track_alvar_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ef2b6ad42bcb18e16b22fefb5c0fb85f";
  }

  static const char* value(const ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xef2b6ad42bcb18e1ULL;
  static const uint64_t static_value2 = 0x6b22fefb5c0fb85fULL;
};

template<class ContainerAllocator>
struct DataType< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ar_track_alvar_msgs/AlvarMarker";
  }

  static const char* value(const ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
uint32 id\n\
uint32 confidence\n\
geometry_msgs/PoseStamped pose\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.id);
      stream.next(m.confidence);
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AlvarMarker_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ar_track_alvar_msgs::AlvarMarker_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
    s << indent << "confidence: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.confidence);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AR_TRACK_ALVAR_MSGS_MESSAGE_ALVARMARKER_H
