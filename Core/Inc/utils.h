/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: utils.h
 * Created Date: Monday, February 26th 2024, 2:37:05 pm
 * Author: Florian Hye
 * Description: This file defines some utils functions.
 *               These are used throughout the firmware.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#ifndef __UTILS_H__
#define __UTILS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <builtin_interfaces/msg/time.h>

void stamp_header(builtin_interfaces__msg__Time *stamp);

static uint16_t twos_complement(int16_t num) { return (uint16_t)(~num + 1); }

double clamp(double d, double min, double max);

uint8_t round_half_up_unscaled(double value, uint8_t SCALING);

#ifdef __cplusplus
}
#endif
#endif /*__UTILS_H__ */
