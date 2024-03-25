/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: hcsr04.h
 * Created Date: Tuesday, March 12th 2024, 3:51:43 pm
 * Author: Florian Hye
 * Description: Define functions for the HCSR04 ultrasonic sensor.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#ifndef __HCSR04_H__
#define __HCSR04_H__

#include "sensor_msgs/msg/range.h"
#include "stdbool.h"
#include "stm32l4xx.h"

typedef struct hcsr04_s {
  TIM_HandleTypeDef *htim;
  uint16_t tim_channel;
  HAL_TIM_ActiveChannel active_channel;
  uint32_t interrupt_channel;
  GPIO_TypeDef *gpio_trig_port;
  uint16_t gpio_trig_pin;

  uint32_t val_1;
  uint32_t val_2;
  uint8_t overflow_count;
  uint32_t difference;
  bool is_first_captured; // is the first value captured ?
  float range;

} hcsr04_t;

void hcsr04_init(hcsr04_t *hcsr04, TIM_HandleTypeDef *htim, uint16_t tim_channel, HAL_TIM_ActiveChannel active_channel,
                 uint32_t interrupt_channel, GPIO_TypeDef *gpio_trig_port, uint16_t gpio_trig_pin);

void hcsr04_measure(hcsr04_t *hcsr04);

void hcsr04_handle_capture_complete(hcsr04_t *hcsr04);

void hcsr04_handle_period_elapsed_interrupt(hcsr04_t *hcsr04);

void hcsr04_init_range_msg(sensor_msgs__msg__Range *msg, rosidl_runtime_c__String frame_id);

// https://controllerstech.com/hcsr04-ultrasonic-sensor-and-stm32/

#endif /* __HCSR04_H__ */
