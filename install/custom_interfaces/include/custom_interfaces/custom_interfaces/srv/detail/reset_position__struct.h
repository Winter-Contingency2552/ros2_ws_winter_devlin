// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:srv/ResetPosition.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__RESET_POSITION__STRUCT_H_
#define CUSTOM_INTERFACES__SRV__DETAIL__RESET_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/ResetPosition in the package custom_interfaces.
typedef struct custom_interfaces__srv__ResetPosition_Request
{
  geometry_msgs__msg__Pose pose;
} custom_interfaces__srv__ResetPosition_Request;

// Struct for a sequence of custom_interfaces__srv__ResetPosition_Request.
typedef struct custom_interfaces__srv__ResetPosition_Request__Sequence
{
  custom_interfaces__srv__ResetPosition_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__ResetPosition_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ResetPosition in the package custom_interfaces.
typedef struct custom_interfaces__srv__ResetPosition_Response
{
  bool success;
} custom_interfaces__srv__ResetPosition_Response;

// Struct for a sequence of custom_interfaces__srv__ResetPosition_Response.
typedef struct custom_interfaces__srv__ResetPosition_Response__Sequence
{
  custom_interfaces__srv__ResetPosition_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__ResetPosition_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__RESET_POSITION__STRUCT_H_
