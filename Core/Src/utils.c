/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: utils.c
 * Created Date: Monday, February 26th 2024, 2:37:05 pm
 * Author: Florian Hye
 * Description: This file implements the utils functions.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#include "utils.h"

#include "rcutils/time.h"
#include "rmw_microros/time_sync.h"

void stamp_header(builtin_interfaces__msg__Time *stamp) {
  rcutils_time_point_value_t now;
  RCUTILS_STEADY_TIME(&now);

  stamp->sec = rmw_uros_epoch_millis();
  stamp->nanosec = rmw_uros_epoch_nanos();
}

double clamp(double d, double min, double max) {
  const double t = d < min ? min : d;
  return t > max ? max : t;
}