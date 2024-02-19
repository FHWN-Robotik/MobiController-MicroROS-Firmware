#include "utils.h"

#include "rcutils/time.h"

void stamp_header(builtin_interfaces__msg__Time* stamp) {
  rcutils_time_point_value_t now;
  RCUTILS_STEADY_TIME(&now);

  stamp->sec = RCUTILS_NS_TO_S(now);
  stamp->nanosec = now;
}