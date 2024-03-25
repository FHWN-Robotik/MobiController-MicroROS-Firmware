/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: hcsr04.c
 * Created Date: Thursday, March 14th 2024, 8:11:00 am
 * Author: Florian Hye
 * Description: Implement the functions for the HCSR04.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#include "hcsr04.h"
#include "cmsis_os.h"
#include "tim.h"

void hcsr04_init(hcsr04_t *hcsr04, TIM_HandleTypeDef *htim, uint16_t tim_channel, HAL_TIM_ActiveChannel active_channel,
                 uint32_t interrupt_channel, GPIO_TypeDef *gpio_trig_port, uint16_t gpio_trig_pin) {
  hcsr04->htim = htim;
  hcsr04->tim_channel = tim_channel;
  hcsr04->active_channel = active_channel;
  hcsr04->interrupt_channel = interrupt_channel;
  hcsr04->gpio_trig_port = gpio_trig_port;
  hcsr04->gpio_trig_pin = gpio_trig_pin;

  hcsr04->is_first_captured = false;
  hcsr04->overflow_count = 0;
  hcsr04->difference = 0;
  hcsr04->val_1 = 0;
  hcsr04->val_2 = 0;
  hcsr04->range = 0;

  // Start capture compare
  HAL_TIM_IC_Start_IT(hcsr04->htim, hcsr04->tim_channel);
}

void hcsr04_measure(hcsr04_t *hcsr04) {
  // Trigger
  HAL_GPIO_WritePin(hcsr04->gpio_trig_port, hcsr04->gpio_trig_pin, GPIO_PIN_SET);   // pull the TRIG pin HIGH
  osDelay(1);                                                                       // wait for 10 us
  HAL_GPIO_WritePin(hcsr04->gpio_trig_port, hcsr04->gpio_trig_pin, GPIO_PIN_RESET); // pull the TRIG pin low

  __HAL_TIM_ENABLE_IT(hcsr04->htim, hcsr04->interrupt_channel);
}

void hcsr04_handle_capture_complete(hcsr04_t *hcsr04) {
  if (!hcsr04->is_first_captured) {                                               // Rising edge capture
    hcsr04->val_1 = HAL_TIM_ReadCapturedValue(hcsr04->htim, hcsr04->tim_channel); // read the first value
    hcsr04->is_first_captured = true;                                             // set the first captured as true
    hcsr04->overflow_count = 0;                                                   // reset overflow count

    // Now change the polarity to falling edge
    __HAL_TIM_SET_CAPTUREPOLARITY(hcsr04->htim, hcsr04->tim_channel, TIM_INPUTCHANNELPOLARITY_FALLING);

  } else {                                                                        // Falling edge capture
    hcsr04->val_2 = HAL_TIM_ReadCapturedValue(hcsr04->htim, hcsr04->tim_channel); // read second value
    hcsr04->val_2 += hcsr04->overflow_count * (hcsr04->htim->Instance->ARR + 1);  // handle overflow

    hcsr04->difference = hcsr04->val_2 - hcsr04->val_1;

    hcsr04->is_first_captured = false;
    hcsr04->overflow_count = 0;

    hcsr04->range = (hcsr04->difference * 0.034 / 2) / 100; // NOTE: The devision will be optimized by the compiler

    // set polarity to rising edge
    __HAL_TIM_SET_CAPTUREPOLARITY(hcsr04->htim, hcsr04->tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_DISABLE_IT(hcsr04->htim, hcsr04->interrupt_channel); // Disable the interrupt
  }
}

void hcsr04_handle_period_elapsed_interrupt(hcsr04_t *hcsr04) { hcsr04->overflow_count++; }

void hcsr04_init_range_msg(sensor_msgs__msg__Range *msg, rosidl_runtime_c__String frame_id) {
  msg->radiation_type = 0;
  msg->header.frame_id = frame_id;
  msg->min_range = 0.02;
  msg->max_range = 4;
  msg->field_of_view = 0.2617994;
  msg->range = 0;
}
