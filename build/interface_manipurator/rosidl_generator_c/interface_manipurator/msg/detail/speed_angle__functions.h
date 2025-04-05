// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from interface_manipurator:msg/SpeedAngle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interface_manipurator/msg/speed_angle.h"


#ifndef INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__FUNCTIONS_H_
#define INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "interface_manipurator/msg/rosidl_generator_c__visibility_control.h"

#include "interface_manipurator/msg/detail/speed_angle__struct.h"

/// Initialize msg/SpeedAngle message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interface_manipurator__msg__SpeedAngle
 * )) before or use
 * interface_manipurator__msg__SpeedAngle__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
bool
interface_manipurator__msg__SpeedAngle__init(interface_manipurator__msg__SpeedAngle * msg);

/// Finalize msg/SpeedAngle message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
void
interface_manipurator__msg__SpeedAngle__fini(interface_manipurator__msg__SpeedAngle * msg);

/// Create msg/SpeedAngle message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interface_manipurator__msg__SpeedAngle__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
interface_manipurator__msg__SpeedAngle *
interface_manipurator__msg__SpeedAngle__create(void);

/// Destroy msg/SpeedAngle message.
/**
 * It calls
 * interface_manipurator__msg__SpeedAngle__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
void
interface_manipurator__msg__SpeedAngle__destroy(interface_manipurator__msg__SpeedAngle * msg);

/// Check for msg/SpeedAngle message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
bool
interface_manipurator__msg__SpeedAngle__are_equal(const interface_manipurator__msg__SpeedAngle * lhs, const interface_manipurator__msg__SpeedAngle * rhs);

/// Copy a msg/SpeedAngle message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
bool
interface_manipurator__msg__SpeedAngle__copy(
  const interface_manipurator__msg__SpeedAngle * input,
  interface_manipurator__msg__SpeedAngle * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
const rosidl_type_hash_t *
interface_manipurator__msg__SpeedAngle__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
const rosidl_runtime_c__type_description__TypeDescription *
interface_manipurator__msg__SpeedAngle__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
const rosidl_runtime_c__type_description__TypeSource *
interface_manipurator__msg__SpeedAngle__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_manipurator__msg__SpeedAngle__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/SpeedAngle messages.
/**
 * It allocates the memory for the number of elements and calls
 * interface_manipurator__msg__SpeedAngle__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
bool
interface_manipurator__msg__SpeedAngle__Sequence__init(interface_manipurator__msg__SpeedAngle__Sequence * array, size_t size);

/// Finalize array of msg/SpeedAngle messages.
/**
 * It calls
 * interface_manipurator__msg__SpeedAngle__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
void
interface_manipurator__msg__SpeedAngle__Sequence__fini(interface_manipurator__msg__SpeedAngle__Sequence * array);

/// Create array of msg/SpeedAngle messages.
/**
 * It allocates the memory for the array and calls
 * interface_manipurator__msg__SpeedAngle__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
interface_manipurator__msg__SpeedAngle__Sequence *
interface_manipurator__msg__SpeedAngle__Sequence__create(size_t size);

/// Destroy array of msg/SpeedAngle messages.
/**
 * It calls
 * interface_manipurator__msg__SpeedAngle__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
void
interface_manipurator__msg__SpeedAngle__Sequence__destroy(interface_manipurator__msg__SpeedAngle__Sequence * array);

/// Check for msg/SpeedAngle message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
bool
interface_manipurator__msg__SpeedAngle__Sequence__are_equal(const interface_manipurator__msg__SpeedAngle__Sequence * lhs, const interface_manipurator__msg__SpeedAngle__Sequence * rhs);

/// Copy an array of msg/SpeedAngle messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
bool
interface_manipurator__msg__SpeedAngle__Sequence__copy(
  const interface_manipurator__msg__SpeedAngle__Sequence * input,
  interface_manipurator__msg__SpeedAngle__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__FUNCTIONS_H_
