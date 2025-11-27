/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: encoder.c
 * Created Date: Monday, March 4th 2024, 12:12:44 pm
 * Author: Florian Hye
 * Description: This file implemnts the encoder functions
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#include "encoder.h"

//static GPIO_PinState prev_state_b;
uint8_t encoder_old_state = 0;
uint8_t encoder_new_state = 0;

void encoder_init(encoder_t *encoder, GPIO_TypeDef *gpio_a_port, uint16_t gpio_a_pin, GPIO_TypeDef *gpio_b_port,
                  uint16_t gpio_b_pin) {
  encoder->gpio_a_port = gpio_a_port;
  encoder->gpio_a_pin = gpio_a_pin;
  encoder->gpio_b_port = gpio_b_port;
  encoder->gpio_b_pin = gpio_b_pin;
  encoder->counter = 0;
  encoder->b_signal = 0;
  //prev_state_b = HAL_GPIO_ReadPin(gpio_b_port,gpio_b_pin);
}

void encoder_handle_interrupt(encoder_t *encoder) {
  
  //__HAL_GPIO_EXTI_GET_RISING_IT(encoder->gpio_a_pin)!=0x00u;

  if (HAL_GPIO_ReadPin(encoder->gpio_a_port, encoder->gpio_a_pin) == GPIO_PIN_SET){
    // Check B pin state and adjust the counter accordingly.
    encoder->b_signal = HAL_GPIO_ReadPin(encoder->gpio_b_port, encoder->gpio_b_pin);

  } 


  if (HAL_GPIO_ReadPin(encoder->gpio_a_port, encoder->gpio_a_pin) == GPIO_PIN_RESET){
    // Check B pin state and adjust the counter accordingly.
    if (encoder->b_signal == GPIO_PIN_RESET){

      encoder->counter++;

    }
    
    if (encoder->b_signal == GPIO_PIN_SET){

      encoder->counter--;
    
    }
  }


/*
  // Get the state of the B pin
  GPIO_PinState state_b = HAL_GPIO_ReadPin(encoder->gpio_b_port, encoder->gpio_b_pin);

  
  if (state_b != prev_state_b){
    // Check B pin state and adjust the counter accordingly.
    if (state_b == GPIO_PIN_SET)
    encoder->counter++;
    else if (state_b == GPIO_PIN_RESET)
    encoder->counter--;
  }

  prev_state_b = state_b;

  ////////////////////////////////////

  HAL_Delay(5);

  encoder_new_state = (uint8_t)((HAL_GPIO_ReadPin(encoder->gpio_b_port,encoder->gpio_b_pin)<<1)|(HAL_GPIO_ReadPin(encoder->gpio_a_port,encoder->gpio_a_pin)));

  if(encoder_old_state == 3 && encoder_new_state == 2){
    encoder->counter++;
  }else if(encoder_old_state == 2 && encoder_new_state == 0){
    encoder->counter++;
  }else if(encoder_old_state == 0 && encoder_new_state == 1){
    encoder->counter++;
  }else if(encoder_old_state == 1 && encoder_new_state == 3){
    encoder->counter++;
  }else if(encoder_old_state == 3 && encoder_new_state == 1){
    encoder->counter--;
  }else if(encoder_old_state == 1 && encoder_new_state == 0){
    encoder->counter--;
  }else if(encoder_old_state == 0 && encoder_new_state == 2){
    encoder->counter--;
  }else if(encoder_old_state == 2 && encoder_new_state == 3){
    encoder->counter--;
  }

  encoder_old_state = encoder_new_state;
*/ 
  
}
