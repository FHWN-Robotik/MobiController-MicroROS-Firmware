#ifndef __UTILS_H__
#define __UTILS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <builtin_interfaces/msg/time.h>

void stamp_header(builtin_interfaces__msg__Time *stamp);

static uint16_t twos_complement(int16_t num) { return (uint16_t)(~num + 1); }

double clamp(double d, double min, double max);

#ifdef __cplusplus
}
#endif
#endif /*__UTILS_H__ */
