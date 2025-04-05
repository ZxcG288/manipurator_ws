// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_manipurator:msg/SpeedAngle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interface_manipurator/msg/speed_angle.hpp"


#ifndef INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__BUILDER_HPP_
#define INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_manipurator/msg/detail/speed_angle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_manipurator
{

namespace msg
{

namespace builder
{

class Init_SpeedAngle_angle
{
public:
  explicit Init_SpeedAngle_angle(::interface_manipurator::msg::SpeedAngle & msg)
  : msg_(msg)
  {}
  ::interface_manipurator::msg::SpeedAngle angle(::interface_manipurator::msg::SpeedAngle::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_manipurator::msg::SpeedAngle msg_;
};

class Init_SpeedAngle_speed
{
public:
  Init_SpeedAngle_speed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SpeedAngle_angle speed(::interface_manipurator::msg::SpeedAngle::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_SpeedAngle_angle(msg_);
  }

private:
  ::interface_manipurator::msg::SpeedAngle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_manipurator::msg::SpeedAngle>()
{
  return interface_manipurator::msg::builder::Init_SpeedAngle_speed();
}

}  // namespace interface_manipurator

#endif  // INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__BUILDER_HPP_
