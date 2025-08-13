// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msgs:msg/ImuData.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/imu_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `accel`
#include "custom_msgs/msg/detail/accel_data__functions.h"
// Member `gyro`
#include "custom_msgs/msg/detail/gyro_data__functions.h"

bool
custom_msgs__msg__ImuData__init(custom_msgs__msg__ImuData * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // accel
  if (!custom_msgs__msg__AccelData__init(&msg->accel)) {
    custom_msgs__msg__ImuData__fini(msg);
    return false;
  }
  // gyro
  if (!custom_msgs__msg__GyroData__init(&msg->gyro)) {
    custom_msgs__msg__ImuData__fini(msg);
    return false;
  }
  return true;
}

void
custom_msgs__msg__ImuData__fini(custom_msgs__msg__ImuData * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // accel
  custom_msgs__msg__AccelData__fini(&msg->accel);
  // gyro
  custom_msgs__msg__GyroData__fini(&msg->gyro);
}

bool
custom_msgs__msg__ImuData__are_equal(const custom_msgs__msg__ImuData * lhs, const custom_msgs__msg__ImuData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // accel
  if (!custom_msgs__msg__AccelData__are_equal(
      &(lhs->accel), &(rhs->accel)))
  {
    return false;
  }
  // gyro
  if (!custom_msgs__msg__GyroData__are_equal(
      &(lhs->gyro), &(rhs->gyro)))
  {
    return false;
  }
  return true;
}

bool
custom_msgs__msg__ImuData__copy(
  const custom_msgs__msg__ImuData * input,
  custom_msgs__msg__ImuData * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // accel
  if (!custom_msgs__msg__AccelData__copy(
      &(input->accel), &(output->accel)))
  {
    return false;
  }
  // gyro
  if (!custom_msgs__msg__GyroData__copy(
      &(input->gyro), &(output->gyro)))
  {
    return false;
  }
  return true;
}

custom_msgs__msg__ImuData *
custom_msgs__msg__ImuData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__ImuData * msg = (custom_msgs__msg__ImuData *)allocator.allocate(sizeof(custom_msgs__msg__ImuData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msgs__msg__ImuData));
  bool success = custom_msgs__msg__ImuData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msgs__msg__ImuData__destroy(custom_msgs__msg__ImuData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msgs__msg__ImuData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msgs__msg__ImuData__Sequence__init(custom_msgs__msg__ImuData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__ImuData * data = NULL;

  if (size) {
    data = (custom_msgs__msg__ImuData *)allocator.zero_allocate(size, sizeof(custom_msgs__msg__ImuData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msgs__msg__ImuData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msgs__msg__ImuData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_msgs__msg__ImuData__Sequence__fini(custom_msgs__msg__ImuData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_msgs__msg__ImuData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_msgs__msg__ImuData__Sequence *
custom_msgs__msg__ImuData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__ImuData__Sequence * array = (custom_msgs__msg__ImuData__Sequence *)allocator.allocate(sizeof(custom_msgs__msg__ImuData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msgs__msg__ImuData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msgs__msg__ImuData__Sequence__destroy(custom_msgs__msg__ImuData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msgs__msg__ImuData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msgs__msg__ImuData__Sequence__are_equal(const custom_msgs__msg__ImuData__Sequence * lhs, const custom_msgs__msg__ImuData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msgs__msg__ImuData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msgs__msg__ImuData__Sequence__copy(
  const custom_msgs__msg__ImuData__Sequence * input,
  custom_msgs__msg__ImuData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msgs__msg__ImuData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_msgs__msg__ImuData * data =
      (custom_msgs__msg__ImuData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msgs__msg__ImuData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_msgs__msg__ImuData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msgs__msg__ImuData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
