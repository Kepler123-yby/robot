// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aim_interfaces:msg/AimInfo.idl
// generated code does not contain a copyright notice

#ifndef AIM_INTERFACES__MSG__DETAIL__AIM_INFO__TRAITS_HPP_
#define AIM_INTERFACES__MSG__DETAIL__AIM_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "aim_interfaces/msg/detail/aim_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const AimInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: coordinate
  {
    if (msg.coordinate.size() == 0) {
      out << "coordinate: []";
    } else {
      out << "coordinate: [";
      size_t pending_items = msg.coordinate.size();
      for (auto item : msg.coordinate) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AimInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: coordinate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.coordinate.size() == 0) {
      out << "coordinate: []\n";
    } else {
      out << "coordinate:\n";
      for (auto item : msg.coordinate) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AimInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace aim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use aim_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const aim_interfaces::msg::AimInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const aim_interfaces::msg::AimInfo & msg)
{
  return aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<aim_interfaces::msg::AimInfo>()
{
  return "aim_interfaces::msg::AimInfo";
}

template<>
inline const char * name<aim_interfaces::msg::AimInfo>()
{
  return "aim_interfaces/msg/AimInfo";
}

template<>
struct has_fixed_size<aim_interfaces::msg::AimInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aim_interfaces::msg::AimInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aim_interfaces::msg::AimInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIM_INTERFACES__MSG__DETAIL__AIM_INFO__TRAITS_HPP_
