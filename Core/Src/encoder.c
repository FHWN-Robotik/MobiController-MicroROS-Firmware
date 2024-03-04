/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: encoder.c
 * Created Date: Monday, March 4th 2024, 12:12:44 pm
 * Author: Florian Hye
 * Description: This file implemnts the encoder functions
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#include "encoder.h"

void encoder_init(encoder_t *encoder, GPIO_TypeDef *gpio_a_port, uint16_t gpio_a_pin, GPIO_TypeDef *gpio_b_port,
                  uint16_t gpio_b_pin) {
  encoder->gpio_a_port = gpio_a_port;
  encoder->gpio_a_pin = gpio_a_pin;
  encoder->gpio_b_port = gpio_b_port;
  encoder->gpio_b_pin = gpio_b_pin;
  encoder->counter = 0;
}

void encoder_handle_interrupt(encoder_t *encoder) {
  // Get the state of the B pin
  GPIO_PinState state_b = HAL_GPIO_ReadPin(encoder->gpio_b_port, encoder->gpio_b_pin);

  // Check B pin state and adjust the counter accordingly.
  if (state_b == GPIO_PIN_SET)
    encoder->counter++;
  else if (state_b == GPIO_PIN_RESET)
    encoder->counter--;
}
