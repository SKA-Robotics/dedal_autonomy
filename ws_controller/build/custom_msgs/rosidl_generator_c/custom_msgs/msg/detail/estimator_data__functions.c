// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msgs:msg/EstimatorData.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/estimator_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `possition`
// Member `speed`
// Member `accel`
// Member `orientation`
// Member `raw_data`
#include "custom_msgs/msg/detail/data_xyz__functions.h"

bool
custom_msgs__msg__EstimatorData__init(custom_msgs__msg__EstimatorData * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // possition
  if (!custom_msgs__msg__DataXYZ__init(&msg->possition)) {
    custom_msgs__msg__EstimatorData__fini(msg);
    return false;
  }
  // speed
  if (!custom_msgs__msg__DataXYZ__init(&msg->speed)) {
    custom_msgs__msg__EstimatorData__fini(msg);
    return false;
  }
  // accel
  if (!custom_msgs__msg__DataXYZ__init(&msg->accel)) {
    custom_msgs__msg__EstimatorData__fini(msg);
    return false;
  }
  // orientation
  if (!custom_msgs__msg__DataXYZ__init(&msg->orientation)) {
    custom_msgs__msg__EstimatorData__fini(msg);
    return false;
  }
  // raw_data
  if (!custom_msgs__msg__DataXYZ__init(&msg->raw_data)) {
    custom_msgs__msg__EstimatorData__fini(msg);
    return false;
  }
  return true;
}

void
custom_msgs__msg__EstimatorData__fini(custom_msgs__msg__EstimatorData * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // possition
  custom_msgs__msg__DataXYZ__fini(&msg->possition);
  // speed
  custom_msgs__msg__DataXYZ__fini(&msg->speed);
  // accel
  custom_msgs__msg__DataXYZ__fini(&msg->accel);
  // orientation
  custom_msgs__msg__DataXYZ__fini(&msg->orientation);
  // raw_data
  custom_msgs__msg__DataXYZ__fini(&msg->raw_data);
}

bool
custom_msgs__msg__EstimatorData__are_equal(const custom_msgs__msg__EstimatorData * lhs, const custom_msgs__msg__EstimatorData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // possition
  if (!custom_msgs__msg__DataXYZ__are_equal(
      &(lhs->possition), &(rhs->possition)))
  {
    return false;
  }
  // speed
  if (!custom_msgs__msg__DataXYZ__are_equal(
      &(lhs->speed), &(rhs->speed)))
  {
    return false;
  }
  // accel
  if (!custom_msgs__msg__DataXYZ__are_equal(
      &(lhs->accel), &(rhs->accel)))
  {
    return false;
  }
  // orientation
  if (!custom_msgs__msg__DataXYZ__are_equal(
      &(lhs->orientation), &(rhs->orientation)))
  {
    return false;
  }
  // raw_data
  if (!custom_msgs__msg__DataXYZ__are_equal(
      &(lhs->raw_data), &(rhs->raw_data)))
  {
    return false;
  }
  return true;
}

bool
custom_msgs__msg__EstimatorData__copy(
  const custom_msgs__msg__EstimatorData * input,
  custom_msgs__msg__EstimatorData * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // possition
  if (!custom_msgs__msg__DataXYZ__copy(
      &(input->possition), &(output->possition)))
  {
    return false;
  }
  // speed
  if (!custom_msgs__msg__DataXYZ__copy(
      &(input->speed), &(output->speed)))
  {
    return false;
  }
  // accel
  if (!custom_msgs__msg__DataXYZ__copy(
      &(input->accel), &(output->accel)))
  {
    return false;
  }
  // orientation
  if (!custom_msgs__msg__DataXYZ__copy(
      &(input->orientation), &(output->orientation)))
  {
    return false;
  }
  // raw_data
  if (!custom_msgs__msg__DataXYZ__copy(
      &(input->raw_data), &(output->raw_data)))
  {
    return false;
  }
  return true;
}

custom_msgs__msg__EstimatorData *
custom_msgs__msg__EstimatorData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__EstimatorData * msg = (custom_msgs__msg__EstimatorData *)allocator.allocate(sizeof(custom_msgs__msg__EstimatorData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msgs__msg__EstimatorData));
  bool success = custom_msgs__msg__EstimatorData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msgs__msg__EstimatorData__destroy(custom_msgs__msg__EstimatorData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msgs__msg__EstimatorData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msgs__msg__EstimatorData__Sequence__init(custom_msgs__msg__EstimatorData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__EstimatorData * data = NULL;

  if (size) {
    data = (custom_msgs__msg__EstimatorData *)allocator.zero_allocate(size, sizeof(custom_msgs__msg__EstimatorData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msgs__msg__EstimatorData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msgs__msg__EstimatorData__fini(&data[i - 1]);
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
custom_msgs__msg__EstimatorData__Sequence__fini(custom_msgs__msg__EstimatorData__Sequence * array)
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
      custom_msgs__msg__EstimatorData__fini(&array->data[i]);
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

custom_msgs__msg__EstimatorData__Sequence *
custom_msgs__msg__EstimatorData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__EstimatorData__Sequence * array = (custom_msgs__msg__EstimatorData__Sequence *)allocator.allocate(sizeof(custom_msgs__msg__EstimatorData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msgs__msg__EstimatorData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msgs__msg__EstimatorData__Sequence__destroy(custom_msgs__msg__EstimatorData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msgs__msg__EstimatorData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msgs__msg__EstimatorData__Sequence__are_equal(const custom_msgs__msg__EstimatorData__Sequence * lhs, const custom_msgs__msg__EstimatorData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msgs__msg__EstimatorData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msgs__msg__EstimatorData__Sequence__copy(
  const custom_msgs__msg__EstimatorData__Sequence * input,
  custom_msgs__msg__EstimatorData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msgs__msg__EstimatorData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_msgs__msg__EstimatorData * data =
      (custom_msgs__msg__EstimatorData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msgs__msg__EstimatorData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_msgs__msg__EstimatorData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msgs__msg__EstimatorData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
