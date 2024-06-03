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
  stamp->sec = (int32_t)(rmw_uros_epoch_millis() / 1000);
  stamp->nanosec = (int32_t)rmw_uros_epoch_nanos();
}

double clamp(double d, double min, double max) {
  const double t = d < min ? min : d;
  return t > max ? max : t;
}

//***************************************************************************
/// Round to nearest integer. 'Half' value is rounded up (to infinity).
/// Uses 'symmetric up' rounding.
/// \param value Scaled integral.
/// \param SCALING Scaling must be divisible by 2
/// \return Unscaled, rounded integral.
//***************************************************************************
uint8_t round_half_up_unscaled(double value, uint8_t SCALING) {
  if (value >= 0) {
    return (value + (SCALING / 2U)) / SCALING;
  } else {
    return (value - (SCALING / 2U)) / SCALING;
  }
}