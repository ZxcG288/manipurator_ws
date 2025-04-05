// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface_manipurator:msg/SpeedAngle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interface_manipurator/msg/speed_angle.hpp"


#ifndef INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__TRAITS_HPP_
#define INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interface_manipurator/msg/detail/speed_angle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interface_manipurator
{

namespace msg
{

inline void to_flow_style_yaml(
  const SpeedAngle & msg,
  std::ostream & out)
{
  out << "{";
  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SpeedAngle & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SpeedAngle & msg, bool use_flow_style = false)
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

}  // namespace interface_manipurator

namespace rosidl_generator_traits
{

[[deprecated("use interface_manipurator::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interface_manipurator::msg::SpeedAngle & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_manipurator::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_manipurator::msg::to_yaml() instead")]]
inline std::string to_yaml(const interface_manipurator::msg::SpeedAngle & msg)
{
  return interface_manipurator::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interface_manipurator::msg::SpeedAngle>()
{
  return "interface_manipurator::msg::SpeedAngle";
}

template<>
inline const char * name<interface_manipurator::msg::SpeedAngle>()
{
  return "interface_manipurator/msg/SpeedAngle";
}

template<>
struct has_fixed_size<interface_manipurator::msg::SpeedAngle>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interface_manipurator::msg::SpeedAngle>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interface_manipurator::msg::SpeedAngle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__TRAITS_HPP_
