/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: encoder.h
 * Created Date: Monday, March 4th 2024, 12:01:21 pm
 * Author: Florian Hye
 * Description: This file defines the encoder functions.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "stm32l4xx.h"

typedef struct encoder_s {
  GPIO_TypeDef *gpio_a_port;
  uint16_t gpio_a_pin;

  GPIO_TypeDef *gpio_b_port;
  uint16_t gpio_b_pin;

  int32_t counter;

} encoder_t;

void encoder_init(encoder_t *encoder, GPIO_TypeDef *gpio_a_port, uint16_t gpio_a_pin, GPIO_TypeDef *gpio_b_port,
                  uint16_t gpio_b_pin);

void encoder_handle_interrupt(encoder_t *encoder);

#endif /* __ENCODER_H_ */
