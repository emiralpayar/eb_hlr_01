/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-diamondback-ros-tutorials-0.2.6/debian/ros-diamondback-ros-tutorials/opt/ros/diamondback/stacks/ros_tutorials/rospy_tutorials/msg/Floats.msg */
#ifndef ROSPY_TUTORIALS_MESSAGE_FLOATS_H
#define ROSPY_TUTORIALS_MESSAGE_FLOATS_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"


namespace rospy_tutorials
{
template <class ContainerAllocator>
struct Floats_ : public ros::Message
{
  typedef Floats_<ContainerAllocator> Type;

  Floats_()
  : data()
  {
  }

  Floats_(const ContainerAllocator& _alloc)
  : data(_alloc)
  {
  }

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _data_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  data;


  ROS_DEPRECATED uint32_t get_data_size() const { return (uint32_t)data.size(); }
  ROS_DEPRECATED void set_data_size(uint32_t size) { data.resize((size_t)size); }
  ROS_DEPRECATED void get_data_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->data; }
  ROS_DEPRECATED void set_data_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->data = vec; }
private:
  static const char* __s_getDataType_() { return "rospy_tutorials/Floats"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "420cd38b6b071cd49f2970c3e2cee511"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float32[] data\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, data);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, data);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(data);
    return size;
  }

  typedef boost::shared_ptr< ::rospy_tutorials::Floats_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rospy_tutorials::Floats_<ContainerAllocator>  const> ConstPtr;
}; // struct Floats
typedef  ::rospy_tutorials::Floats_<std::allocator<void> > Floats;

typedef boost::shared_ptr< ::rospy_tutorials::Floats> FloatsPtr;
typedef boost::shared_ptr< ::rospy_tutorials::Floats const> FloatsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::rospy_tutorials::Floats_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::rospy_tutorials::Floats_<ContainerAllocator> >::stream(s, "", v);
  return s;
}

} // namespace rospy_tutorials

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::rospy_tutorials::Floats_<ContainerAllocator> > {
  static const char* value() 
  {
    return "420cd38b6b071cd49f2970c3e2cee511";
  }

  static const char* value(const  ::rospy_tutorials::Floats_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x420cd38b6b071cd4ULL;
  static const uint64_t static_value2 = 0x9f2970c3e2cee511ULL;
};

template<class ContainerAllocator>
struct DataType< ::rospy_tutorials::Floats_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rospy_tutorials/Floats";
  }

  static const char* value(const  ::rospy_tutorials::Floats_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rospy_tutorials::Floats_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32[] data\n\
\n\
";
  }

  static const char* value(const  ::rospy_tutorials::Floats_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rospy_tutorials::Floats_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Floats_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rospy_tutorials::Floats_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::rospy_tutorials::Floats_<ContainerAllocator> & v) 
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.data[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // ROSPY_TUTORIALS_MESSAGE_FLOATS_H
