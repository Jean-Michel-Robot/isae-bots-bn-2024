// Generated by gencpp from file message/ActionnersMsg.msg
// DO NOT EDIT!


#ifndef MESSAGE_MESSAGE_ACTIONNERSMSG_H
#define MESSAGE_MESSAGE_ACTIONNERSMSG_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace message
{
template <class ContainerAllocator>
struct ActionnersMsg_
{
  typedef ActionnersMsg_<ContainerAllocator> Type;

  ActionnersMsg_()
    : act()
    , fail(false)
    , end(false)  {
    }
  ActionnersMsg_(const ContainerAllocator& _alloc)
    : act(_alloc)
    , fail(false)
    , end(false)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _act_type;
  _act_type act;

   typedef uint8_t _fail_type;
  _fail_type fail;

   typedef uint8_t _end_type;
  _end_type end;





  typedef boost::shared_ptr< ::message::ActionnersMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::message::ActionnersMsg_<ContainerAllocator> const> ConstPtr;

}; // struct ActionnersMsg_

typedef ::message::ActionnersMsg_<std::allocator<void> > ActionnersMsg;

typedef boost::shared_ptr< ::message::ActionnersMsg > ActionnersMsgPtr;
typedef boost::shared_ptr< ::message::ActionnersMsg const> ActionnersMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::message::ActionnersMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::message::ActionnersMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::message::ActionnersMsg_<ContainerAllocator1> & lhs, const ::message::ActionnersMsg_<ContainerAllocator2> & rhs)
{
  return lhs.act == rhs.act &&
    lhs.fail == rhs.fail &&
    lhs.end == rhs.end;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::message::ActionnersMsg_<ContainerAllocator1> & lhs, const ::message::ActionnersMsg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace message

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::message::ActionnersMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::message::ActionnersMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::message::ActionnersMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::message::ActionnersMsg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::message::ActionnersMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::message::ActionnersMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::message::ActionnersMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3dd2e8d97f70009cf8e47cbf3e745186";
  }

  static const char* value(const ::message::ActionnersMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3dd2e8d97f70009cULL;
  static const uint64_t static_value2 = 0xf8e47cbf3e745186ULL;
};

template<class ContainerAllocator>
struct DataType< ::message::ActionnersMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "message/ActionnersMsg";
  }

  static const char* value(const ::message::ActionnersMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::message::ActionnersMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string act\n"
"bool fail\n"
"bool end\n"
;
  }

  static const char* value(const ::message::ActionnersMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::message::ActionnersMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.act);
      stream.next(m.fail);
      stream.next(m.end);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ActionnersMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::message::ActionnersMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::message::ActionnersMsg_<ContainerAllocator>& v)
  {
    s << indent << "act: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.act);
    s << indent << "fail: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.fail);
    s << indent << "end: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.end);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MESSAGE_MESSAGE_ACTIONNERSMSG_H
