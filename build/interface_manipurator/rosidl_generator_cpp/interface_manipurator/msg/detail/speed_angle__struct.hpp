// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface_manipurator:msg/SpeedAngle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interface_manipurator/msg/speed_angle.hpp"


#ifndef INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__STRUCT_HPP_
#define INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interface_manipurator__msg__SpeedAngle __attribute__((deprecated))
#else
# define DEPRECATED__interface_manipurator__msg__SpeedAngle __declspec(deprecated)
#endif

namespace interface_manipurator
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SpeedAngle_
{
  using Type = SpeedAngle_<ContainerAllocator>;

  explicit SpeedAngle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0ll;
      this->angle = 0ll;
    }
  }

  explicit SpeedAngle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0ll;
      this->angle = 0ll;
    }
  }

  // field types and members
  using _speed_type =
    int64_t;
  _speed_type speed;
  using _angle_type =
    int64_t;
  _angle_type angle;

  // setters for named parameter idiom
  Type & set__speed(
    const int64_t & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__angle(
    const int64_t & _arg)
  {
    this->angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_manipurator::msg::SpeedAngle_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_manipurator::msg::SpeedAngle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_manipurator::msg::SpeedAngle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_manipurator::msg::SpeedAngle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_manipurator::msg::SpeedAngle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_manipurator::msg::SpeedAngle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_manipurator::msg::SpeedAngle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_manipurator::msg::SpeedAngle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_manipurator::msg::SpeedAngle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_manipurator::msg::SpeedAngle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_manipurator__msg__SpeedAngle
    std::shared_ptr<interface_manipurator::msg::SpeedAngle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_manipurator__msg__SpeedAngle
    std::shared_ptr<interface_manipurator::msg::SpeedAngle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SpeedAngle_ & other) const
  {
    if (this->speed != other.speed) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const SpeedAngle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SpeedAngle_

// alias to use template instance with default allocator
using SpeedAngle =
  interface_manipurator::msg::SpeedAngle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interface_manipurator

#endif  // INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__STRUCT_HPP_
