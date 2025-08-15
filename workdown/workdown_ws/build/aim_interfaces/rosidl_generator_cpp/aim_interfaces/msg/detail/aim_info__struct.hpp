// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from aim_interfaces:msg/AimInfo.idl
// generated code does not contain a copyright notice

#ifndef AIM_INTERFACES__MSG__DETAIL__AIM_INFO__STRUCT_HPP_
#define AIM_INTERFACES__MSG__DETAIL__AIM_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__aim_interfaces__msg__AimInfo __attribute__((deprecated))
#else
# define DEPRECATED__aim_interfaces__msg__AimInfo __declspec(deprecated)
#endif

namespace aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AimInfo_
{
  using Type = AimInfo_<ContainerAllocator>;

  explicit AimInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
    }
  }

  explicit AimInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
    }
  }

  // field types and members
  using _coordinate_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _coordinate_type coordinate;
  using _type_type =
    uint16_t;
  _type_type type;

  // setters for named parameter idiom
  Type & set__coordinate(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->coordinate = _arg;
    return *this;
  }
  Type & set__type(
    const uint16_t & _arg)
  {
    this->type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    aim_interfaces::msg::AimInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const aim_interfaces::msg::AimInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<aim_interfaces::msg::AimInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<aim_interfaces::msg::AimInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      aim_interfaces::msg::AimInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<aim_interfaces::msg::AimInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      aim_interfaces::msg::AimInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<aim_interfaces::msg::AimInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<aim_interfaces::msg::AimInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<aim_interfaces::msg::AimInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__aim_interfaces__msg__AimInfo
    std::shared_ptr<aim_interfaces::msg::AimInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__aim_interfaces__msg__AimInfo
    std::shared_ptr<aim_interfaces::msg::AimInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AimInfo_ & other) const
  {
    if (this->coordinate != other.coordinate) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    return true;
  }
  bool operator!=(const AimInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AimInfo_

// alias to use template instance with default allocator
using AimInfo =
  aim_interfaces::msg::AimInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace aim_interfaces

#endif  // AIM_INTERFACES__MSG__DETAIL__AIM_INFO__STRUCT_HPP_
