// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aim_interfaces:msg/AimInfo.idl
// generated code does not contain a copyright notice

#ifndef AIM_INTERFACES__MSG__DETAIL__AIM_INFO__BUILDER_HPP_
#define AIM_INTERFACES__MSG__DETAIL__AIM_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "aim_interfaces/msg/detail/aim_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_AimInfo_type
{
public:
  explicit Init_AimInfo_type(::aim_interfaces::msg::AimInfo & msg)
  : msg_(msg)
  {}
  ::aim_interfaces::msg::AimInfo type(::aim_interfaces::msg::AimInfo::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aim_interfaces::msg::AimInfo msg_;
};

class Init_AimInfo_coordinate
{
public:
  Init_AimInfo_coordinate()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AimInfo_type coordinate(::aim_interfaces::msg::AimInfo::_coordinate_type arg)
  {
    msg_.coordinate = std::move(arg);
    return Init_AimInfo_type(msg_);
  }

private:
  ::aim_interfaces::msg::AimInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::aim_interfaces::msg::AimInfo>()
{
  return aim_interfaces::msg::builder::Init_AimInfo_coordinate();
}

}  // namespace aim_interfaces

#endif  // AIM_INTERFACES__MSG__DETAIL__AIM_INFO__BUILDER_HPP_
